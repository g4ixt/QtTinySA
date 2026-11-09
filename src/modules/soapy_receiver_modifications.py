#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov  3 16:10:52 2026

@author: ian
"""

import time

from PySide6.QtCore import QElapsedTimer
# ----------------------------------------------------------------------------------------------------------------

    def _run_single_loop(self, fft_size: int, window: np.ndarray, read_buf: np.ndarray, sample_rate: float) -> None:
        """Ordinary single-frequency acquisition: one capture per iteration at self._s["center_freq"], which
        may change live between iterations (see set_center_frequency())."""
        
        center_freq = self._s["center_freq"]
        freqs = np.fft.fftshift(np.fft.fftfreq(fft_size, d=1.0 / sample_rate)) + center_freq
        freqs = freqs.astype(np.float64)

        # set-up for averaging
        running_sum = np.zeros(fft_size, dtype=np.float32)
        counter = 0
        
        info_count = 0
        update_nS = 50 * 1e6
        ref_nS = time.perf_counter_ns()  # lower overhead than a QElapsedTimer
        #
        # Allocate SIMD-aligned memory buffers
        fft_input = pyfftw.empty_aligned(fft_size, dtype='complex64')
        fft_output = pyfftw.empty_aligned(fft_size, dtype='complex64')

        # Create a re-usable FFT plan
        # 'FFTW_MEASURE' spends a few seconds upfront to find the fastest algorithm for the CPU
        logging.info("Planning FFT... may take a few seconds...")

        fft_object = pyfftw.FFTW(fft_input, fft_output, direction='FFTW_FORWARD', flags=('FFTW_MEASURE',), threads=1)
        # Match threads to CPU's physical core count
        
        while not self._stop_event.is_set():
            self._drain_commands()

            if self._s["center_freq"] != center_freq:
                center_freq = self._s["center_freq"]
                freqs = np.fft.fftshift(np.fft.fftfreq(fft_size, d=1.0 / sample_rate)) + center_freq
                freqs = freqs.astype(np.float64)
                
                # reset the averaging
                running_sum.fill(0.0)
                counter = 0
                #

            if self._device is None or self._rx_stream is None:
                time.sleep(IDLE_POLL_INTERVAL_S)
                continue

            power = self._capture_block(fft_object, fft_input, fft_output, fft_size, window, read_buf)
            if power is None:
                logging.info("power is None")
                continue
            
            cal_freqs = self._s.get("cal_freqs_hz")
            cal_offsets = self._s.get("cal_offsets_db")
            if cal_freqs is not None and cal_offsets is not None:
                power = power - np.interp(freqs, cal_freqs, cal_offsets).astype(np.float32)

            # update the total
            running_sum += power
            counter += 1

            # calculate the average and put it into the queue if a reading is due, otherwise loop
            elapsed = time.perf_counter_ns() - ref_nS
            if elapsed >= update_nS:  # this needs to be related or equal to 'interval'
                ref_nS = time.perf_counter_ns()
                info_count += 1
                if info_count > 20:
                    logging.info(f'{counter} measurement avg = {round(elapsed * 1e-6 / counter, 2)}mS')
                    info_count = 0

                average = running_sum / counter
                self._emit(freqs, average)
                running_sum.fill(0.0)
                counter = 0
    
# ----------------------------------------------------------------------------------------------------------------

    def _run_sweep_loop(self, fft_size: int, window: np.ndarray, read_buf: np.ndarray, sample_rate: float) -> None:
        """
        Multi-segment sweep: repeatedly steps through every segment's center frequency, capturing one block per
        segment. Segment layout (centers, per-segment frequency axes, crop amount for composite mode) is fixed
        for the lifetime of a configured sweep -- live retuning isn't supported while sweeping, so none of this
        needs to be recomputed mid-loop the way the single-capture path's freqs axis does.

        The one exception: if the LO can't lock at a segment's planned center and _apply_frequency() retunes
        nearby instead, that segment's bins are labeled with the frequency actually tuned (so its axis stays
        truthful) and a status message is queued. In composite mode the array length is still exactly
        composite_size (the crop is a fixed bin count), but seam tiling is no longer guaranteed perfect at that
        segment's edges: if the retune offset exceeds the overlap margin there, the stitched frequency axis can
        show a small gap or a tiny backward step at that seam. Accepted deliberately -- discarding the whole
        cycle instead would mean a persistently unlockable frequency never produces any composite output.
        """
        start_hz = self._s["sweep_start_hz"]
        num_segments = self._s["num_segments"]
        segment_span_nominal = self._s["segment_span_nominal_hz"]
        output_mode = self._s["sweep_output_mode"]
        crop_bins = compute_segment_crop_bins(fft_size, sample_rate, segment_span_nominal)

        segment_centers = [start_hz + segment_span_nominal * (i + 0.5) for i in range(num_segments)]
        segment_freqs = [
            (np.fft.fftshift(np.fft.fftfreq(fft_size, d=1.0 / sample_rate)) + c).astype(np.float64)
            for c in segment_centers
        ]

        while not self._stop_event.is_set():
            composite_power_parts: list[np.ndarray] = []
            composite_freqs_parts: list[np.ndarray] = []
            cycle_ok = True

            for i, center_freq in enumerate(segment_centers):
                if self._stop_event.is_set():
                    return
                self._drain_commands()

                if self._device is None or self._rx_stream is None:
                    time.sleep(IDLE_POLL_INTERVAL_S)
                    cycle_ok = False
                    continue

                actual_center = self._apply_frequency(center_freq)
                if actual_center is None:
                    cycle_ok = False  # tuning failed even after retries -- treat like any other failed capture
                    continue

                power = self._capture_block(fft_size, window, read_buf)
                if power is None:
                    cycle_ok = False
                    continue

                if actual_center == center_freq:
                    freqs = segment_freqs[i]
                else:
                    # The LO couldn't lock at the planned center and retuned nearby (see _apply_frequency()):
                    # label this segment's bins with the frequency actually tuned, not the one planned.
                    freqs = (np.fft.fftshift(np.fft.fftfreq(fft_size, d=1.0 / sample_rate)) + actual_center
                             ).astype(np.float64)
                cal_freqs = self._s.get("cal_freqs_hz")
                cal_offsets = self._s.get("cal_offsets_db")
                if cal_freqs is not None and cal_offsets is not None:
                    power = power - np.interp(freqs, cal_freqs, cal_offsets).astype(np.float32)

                if output_mode == "segments":
                    self._emit(freqs, power)
                elif cycle_ok:
                    if i == 0:
                        lo, hi = 0, fft_size - crop_bins
                    elif i == num_segments - 1:
                        lo, hi = crop_bins, fft_size
                    else:
                        lo, hi = crop_bins, fft_size - crop_bins
                    composite_freqs_parts.append(freqs[lo:hi])
                    composite_power_parts.append(power[lo:hi])

            if output_mode == "composite" and cycle_ok and composite_power_parts and not self._stop_event.is_set():
                self._emit(np.concatenate(composite_freqs_parts), np.concatenate(composite_power_parts))


# ----------------------------------------------------------------------------------------------------------------
# pyfftw example

import numpy as np
import pyfftw

# 1. Initialization and Parameter Setup
fft_size = 2048                # Must be a power of 2 for optimal speed
sample_rate = 10e6             # 10 MSps from LimeSDR

# 2. Allocate SIMD-aligned memory buffers
# LimeSDR streams complex data (I + jQ)
fft_input = pyfftw.empty_aligned(fft_size, dtype='complex64')
fft_output = pyfftw.empty_aligned(fft_size, dtype='complex64')

# 3. Create a re-usable FFT plan
# 'FFTW_MEASURE' spends a few seconds upfront to find the fastest algorithm for your CPU
logging.info("Planning FFT... may take a few seconds...")

fft_object = pyfftw.FFTW(fft_input, fft_output, direction='FFTW_FORWARD', flags=('FFTW_MEASURE',), threads=4)
# Match threads to CPU's physical core count

# 4. Generate a window function (e.g., Hann) to prevent spectral leakage
window = pyfftw.interfaces.numpy_fft.fftshift(np.hanning(fft_size).astype('float32'))

# --- SDR STREAMING LOOP EXAMPLE ---
# Assume `sdr` is your initialized LimeSDR object
try:
    while True:
        # A. Receive raw IQ samples from the LimeSDR buffer
        # Ensure your SDR API reads exactly `fft_size` into a temporary numpy array
        raw_iq_samples = sdr.read_samples(fft_size) 
        
        # B. Apply window and inject data directly into the aligned input buffer
        # The [:] syntax copies the data into the already allocated aligned memory
        fft_input[:] = raw_iq_samples * window
        
        # C. Execute the pre-planned FFT (highly optimized C/Assembly level)
        fft_object()
        
        # D. Convert complex output to Power Spectral Density (dB)
        # Shift frequencies so DC (0 Hz) is in the middle
        shifted_output = np.fft.fftshift(fft_output)
        
        # Calculate magnitude squared, normalize, and convert to dB scale
        psd_db = 10 * np.log10(np.abs(shifted_output)**2 / fft_size)
        
        # E. Push `psd_db` to your UI/Visualization Queue (e.g., PyQtGraph or Matplotlib)
        # update_spectrum_plot(psd_db)

except KeyboardInterrupt:
    print("Streaming stopped.")
 

 
# ----------------------------------------------------------------------------------------------------------------
  
    def _capture_block(self, fft_object, fft_input, fft_output, fft_size: int,
                       window: np.ndarray, read_buf: np.ndarray) -> np.ndarray | None:
        """
        Fill read_buf with fft_size IQ samples via readStream(), apply software DC removal if configured, and
        return the computed power spectrum (before calibration correction or any sweep-mode cropping). Returns
        None on timeout, an unrecoverable readStream error, or if stop_event fires mid-read -- callers should
        treat None as "skip this capture, try again" (or, in composite sweep mode, "discard this whole cycle").
        """
        n_collected = 0
        timed_out = False
        while n_collected < fft_size:
            sr = self._device.readStream(
                self._rx_stream, [read_buf[n_collected:]], fft_size - n_collected, timeoutUs=200_000,
            )
            if sr.ret > 0:
                n_collected += sr.ret
            elif sr.ret == SoapySDR.SOAPY_SDR_TIMEOUT:
                timed_out = True
                break
            elif sr.ret == SoapySDR.SOAPY_SDR_OVERFLOW:
                logger.debug("SoapySDR overflow, continuing")
                continue
            else:
                self._event_queue.put(("error", f"SoapySDR readStream error: {sr.ret}"))
                timed_out = True
                break
            if self._stop_event.is_set():
                return None

        if self._stop_event.is_set() or timed_out or n_collected < fft_size:
            return None
          
        if self._s["dc_offset_mode"] in ("software", "both"):
           # Direct in-place subtraction preserves memory block
           read_buf -= np.mean(read_buf)
        
        return compute_spectrum(fft_object, fft_input, fft_output, read_buf, window, fft_size)


# ----------------------------------------------------------------------------------------------------------------

def compute_spectrum(fft_object, fft_input, fft_output, iq: np.ndarray, window: np.ndarray, fft_size: int) -> np.ndarray:
    """Apply the window, FFT, and convert to dB efficiently."""
    
    # Inject data into the aligned pyfftw buffer and execute the pre-planned FFT 
    fft_input[:] = iq * window
    fft_object()

    # Calculate absolute magnitude first on the raw output buffer.
    # This turns complex64 down into float32, cutting the memory footprint in half.
    power = 20 * np.log10(np.abs(fft_output) / fft_size + 1e-20)
    
    # 4. Shift the final real float32 array instead of the heavier complex one.
    return np.fft.fftshift(power).astype(np.float32)


# ----------------------------------------------------------------------------------------------------------------


numpy fft single loop

    def _run_single_loop(
        self, fft_size: int, window: np.ndarray, read_buf: np.ndarray, sample_rate: float) -> None:
        """Ordinary single-frequency acquisition: one capture per iteration at self._s["center_freq"], which
        may change live between iterations (see set_center_frequency())."""
        center_freq = self._s["center_freq"]
        freqs = np.fft.fftshift(np.fft.fftfreq(fft_size, d=1.0 / sample_rate)) + center_freq
        freqs = freqs.astype(np.float64)
        #
        running_sum = np.zeros(fft_size, dtype=np.float32)
        counter = 0
        info_count = 0

        update_nS = 50 * 1e6
        ref_nS = time.perf_counter_ns()  # lower overhead than a QElapsedTimer
        #
        while not self._stop_event.is_set():
            self._drain_commands()

            if self._s["center_freq"] != center_freq:
                center_freq = self._s["center_freq"]
                freqs = np.fft.fftshift(np.fft.fftfreq(fft_size, d=1.0 / sample_rate)) + center_freq
                freqs = freqs.astype(np.float64)
                
                running_sum.fill(0.0)
                counter = 0

            if self._device is None or self._rx_stream is None:
                time.sleep(IDLE_POLL_INTERVAL_S)
                continue

            power = self._capture_block(fft_size, window, read_buf)
            if power is None:
                continue

            cal_freqs = self._s.get("cal_freqs_hz")
            cal_offsets = self._s.get("cal_offsets_db")
            if cal_freqs is not None and cal_offsets is not None:
                power = power - np.interp(freqs, cal_freqs, cal_offsets).astype(np.float32)

            #
            running_sum += power
            counter += 1
            elapsed = time.perf_counter_ns() - ref_nS
            if elapsed >= update_nS:  # this needs to be related or equal to 'interval'
                ref_nS = time.perf_counter_ns()
                info_count += 1
                if info_count > 20:
                    logging.info(f'{counter} measurement avg = {round(elapsed * 1e-6 / counter, 2)}mS')
                    info_count = 0
                average = running_sum / counter
                self._emit(freqs, average)
                running_sum.fill(0.0)
                counter = 0
                
# ----------------------------------------------------------------------------------------------------------------