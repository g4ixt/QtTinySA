import os
import sys
import queue
import threading
import time

from rtlsdr import RtlSdr
import numpy as np
import scipy.signal as signal
import pyaudio

# This is a set of routines that support enabling an attached SDR (e.g. RTLSDR)
# set to a specific Frequency that is passed in.  Then using Demodulation to listen
# to the selected Frequency.
#
# spw  12/22/2025


# --- CONFIGURATION OPTIMIZED FOR NOAA NFM ---
# Target Audio output rate (Standard audio rate)  44100 (eats too much CPU)
AUDIO_RATE = 8000

# Stable, low sample rate for maximum performance
SDR_SAMPLE_RATE = 1024000.0 

# Audio fidelity fix: Wide NFM Bandwidth (12 kHz) to allow full audio tones
NFM_BANDWIDTH = 12000 

# Chunk size for reading samples (1.0 second of data)
SAMPLES_PER_READ = int(SDR_SAMPLE_RATE * 1.0) 

# PyAudio Buffer Size (3 seconds of audio data safety margin)
P_BUFFER_SIZE = AUDIO_RATE * 3 

# --- GLOBAL STATE VARIABLES ---
audio_queue = queue.Queue()
stop_event = threading.Event()
filter_zi = None 
taps = None      
sdr = None       
center_freq_hz = None

# PyAudio global variables - ENSURE these are truly global and managed
p = None
stream = None
sdr_thread = None

# New global variables for dynamic frequency change
_frequency_lock = threading.Lock()
_sdr_retune_event = threading.Event()

# --- FUNCTIONS ---

def nfm_demod(samples):
    """
    Performs NFM demodulation, filtering, and high-fidelity resampling.
    """
    global filter_zi
    global taps
    
    # 1. FIR Filter Setup (Only done once)
    if taps is None:
        # Taps remain at 32 for reduced CPU load.
        cutoff_freq = NFM_BANDWIDTH / 2.0 / SDR_SAMPLE_RATE
        taps = signal.firwin(32, cutoff_freq * 2.0, window=('blackman')) 
        
    # 2. State Management and Filtering
    iq_data = samples.astype(np.complex64)
    if filter_zi is None:
        filter_zi = signal.lfilter_zi(taps, 1.0)
        
    filtered_samples, filter_zf = signal.lfilter(taps, 1.0, iq_data, zi=filter_zi * iq_data[0])
    filter_zi = filter_zf 

    # 3. Quadrature Demodulation
    demodulated_data = np.angle(filtered_samples[1:] * np.conj(filtered_samples[:-1]))
    
    # 4. Resample (Audio Fidelity Fix)
    # Replaced signal.decimate with signal.resample_poly for better anti-aliasing behavior
    from_rate = SDR_SAMPLE_RATE
    to_rate = AUDIO_RATE
    
    # resample_poly performs high-quality resampling to the target audio rate
    audio_data = signal.resample_poly(demodulated_data, int(to_rate), int(from_rate))

    return audio_data.astype(np.float32)
    
    
def am_demod(samples, sdr_sample_rate):
    """
    Performs AM demodulation (Envelope Detection) and resampling.
    """
    # 1. Envelope Detection
    # Taking the magnitude (abs) of the complex signal extracts the AM envelope.
    audio_data = np.abs(samples)
    
    # 2. DC Offset Removal
    # AM signals have a large constant carrier (the DC component). 
    # Subtracting the mean centers the audio around zero.
    audio_data = audio_data - np.mean(audio_data)
    
    # 3. Resample/Decimate to the audio rate (44100 Hz)
    decimation_factor = int(sdr_sample_rate / AUDIO_RATE) 
    audio_output = signal.decimate(audio_data, decimation_factor)
    
    return audio_output.astype(np.float32)
    

def sdr_worker():
    """Worker thread: Initializes SDR, reads data, processes, and fills the queue."""
    global sdr
    global taps
    global filter_zi
    global center_freq_hz
    start_time = time.time()

    try:
        # --- SDR Initialization ---
        sdr = RtlSdr()
        sdr.sample_rate = SDR_SAMPLE_RATE
        # Initial tuning
        with _frequency_lock:
            sdr.center_freq = center_freq_hz
        sdr.gain = 'auto'
        print(f"[Worker: INFO] Tuned to {sdr.center_freq/1e6:.3f} MHz at {sdr.sample_rate/1e6:.3f} MS/s\n")

        # Initialize NFM filter state only if current frequency is NOT AM
        with _frequency_lock:
            current_freq = center_freq_hz

        is_am = (530e3 <= current_freq <= 1710e3) or (118e6 <= current_freq <= 137e6)

        if (taps is None) and not is_am:
            nfm_demod(np.zeros(1024, dtype=np.complex64))
        
        while not stop_event.is_set():
            # Check for frequency retune request
            if _sdr_retune_event.is_set():
                with _frequency_lock:
                    sdr.center_freq = center_freq_hz
                print(f"[Worker: INFO] SDR retuned to {sdr.center_freq/1e6:.3f} MHz\n") # Added print statement
                _sdr_retune_event.clear()

            # 1. READ SAMPLES (Blocking)
            read_start = time.time()
            # print(f"[Worker: READ] Starting read at T={read_start - start_time:.2f}s...\n")
            samples = sdr.read_samples(SAMPLES_PER_READ)
            
            # 2. DEMODULATE AND RESAMPLE (DSP)
            with _frequency_lock:
                current_freq = center_freq_hz

            # AM broadcast band (0.53–1.71 MHz) OR Airband AM (118–137 MHz)
            if (530e3 <= current_freq <= 1710e3) or (118e6 <= current_freq <= 137e6):
                print("[Worker: INFO] Using AM demodulator (AM band or Airband)")
                audio_output = am_demod(samples, SDR_SAMPLE_RATE)
            else:
                print("[Worker: INFO] Using NFM demodulator")
                audio_output = nfm_demod(samples)

    
            # 3. PUT IN QUEUE
            audio_output /= np.max(np.abs(audio_output))
            audio_queue.put(audio_output.tobytes())
            put_time = time.time()
            # Confirm process time is below 1.000s
            # print(f"[Worker: PUT] Audio chunk deposited. Read+Process Time: {put_time - read_start:.3f}s\n")
            
    except Exception as e:
        if not stop_event.is_set():
             print(f"[Worker: ERROR] FATAL SDR Worker Error: {e}\n")
    finally:
        if sdr:
            sdr.close()
        print("[Worker: INFO] SDR worker finished.\n")

def set_sdr_frequency(new_frequency_hz):
    global center_freq_hz
    with _frequency_lock:
        center_freq_hz = new_frequency_hz
    _sdr_retune_event.set()
    print(f"[FM5: INFO] SDR frequency change requested to {new_frequency_hz / 1e6:.3f} MHz.\n") # Ensure this print is present

def start_sdr_playback(frequency_hz):
    print(f"start_sdr_playback called with frequency_hz={frequency_hz}")

    global center_freq_hz, stop_event, audio_queue, p, stream, sdr_thread
    with _frequency_lock: # Ensure initial frequency is set safely
        center_freq_hz = frequency_hz
    
    # Reset event and queue for a new session
    stop_event.clear()
    _sdr_retune_event.clear() # Clear any pending retune requests
    while not audio_queue.empty():
        audio_queue.get()
        audio_queue.task_done()

    try:
        # Initialize PyAudio only if not already initialized
        if p is None:
            p = pyaudio.PyAudio()
        stream = p.open(format=pyaudio.paFloat32,
                        channels=1,
                        rate=AUDIO_RATE,
                        output=True,
                        frames_per_buffer=P_BUFFER_SIZE) 

        # 2. Start the Background Worker Thread
        sdr_thread = threading.Thread(target=sdr_worker, name="SDR-DSP-Worker")
        sdr_thread.start()
        
        # 3. *** CRITICAL PRE-BUFFERING STEP (3 seconds) ***
        required_chunks = 3 
        print(f"[Main: INFO] Pre-buffering audio (waiting for {required_chunks} seconds of data)...\n")
        chunks_received = 0
        while chunks_received < required_chunks:
            try:
                audio_queue.get(timeout=1.0) 
                audio_queue.task_done()
                chunks_received += 1
            except queue.Empty:
                print("[Main: WARNING] Worker thread is stalled during pre-buffer.\n")
                pass
            
        # 4. Main Audio Playback Loop
        print("[Main: INFO] Pre-buffering complete. Listening... Press Ctrl+C to stop.\n")
        while not stop_event.is_set():
            try:
                audio_data = audio_queue.get(timeout=0.01) 
                stream.write(audio_data)
                audio_queue.task_done()
            except queue.Empty:
                pass
            
    except Exception as e:
        print(f"\n[Main: ERROR] An unexpected error occurred in start_sdr_playback: {e}\n")
    finally:
        # Only cleanup resources that belong to this specific playback session
        if stream:
            stream.stop_stream()
            stream.close()
        
        # Signal sdr_worker to stop and join its thread
        if stop_event:
            stop_event.set()
        if sdr_thread and sdr_thread.is_alive():
            sdr_thread.join(timeout=5)
        
        print("[Main: INFO] SDR playback session cleanup complete.\n")


def stop_sdr_playback():
    global stop_event
    print("[Main: INFO] Signaling SDR playback to stop...\n")
    stop_event.set()
    print("[Main: INFO] SDR playback stop signal sent.\n")