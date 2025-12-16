## 📡 QtTinySDR: SDR Audio Integration for Spectrum Analysis

### Project Overview

This repository is a **fork of the original QtTinySA project**, enhanced with the crucial capability to **demodulate and play audio** from signals displayed on the spectrum analyzer.

By combining the wide-ranging analysis of the TinySA with the real-time audio capability of an RTL-SDR dongle, this project transforms the spectrum analyzer into a dynamic receiver.

### 🌟 Key Features

* **Audio Monitoring Integration:** Seamlessly listen to any signal displayed on the TinySA spectrum.
* **Marker-Based Tuning:** Simply move **Marker 1** on the QtTinySA display to the signal of interest, and the application tunes the SDR to that frequency.
* **Automatic Modulation Selection:** The application automatically determines the demodulation mode:
    * **AM Demodulation** is used for frequencies within the conventional AM broadcast band.
    * **FM Demodulation (NFM)** is used for all other frequencies.
* **Fine-Tuning Controls:** An "SDR Audio" popup window provides controls (`+` and `-` buttons) for minor frequency adjustments to fine-tune the listening frequency without moving the Marker on the main display.

### ⚙️ How It Works (The Setup)

This capability is achieved by integrating a standard RTL-SDR dongle into your setup. To use this fork, you must have **both** devices connected to your PC simultaneously:

1.  **TinySA (via USB):** Used as the **Spectrum Analyzer** to capture and display the frequency range (the visual component).
2.  **RTL-SDR (via USB):** Used as the **Software Defined Radio** to capture the specific waveform and demodulate the audio (the listening component).

> **Pro Tip:** Before running this code, ensure your RTL-SDR is functional using popular software like **SDR++** or **Gqrx**.

### 🛠 Implementation Details

Minimal changes were made to the original QtTinySA structure:

* **`QtTinySA.py`:** Small hook added to launch the new audio window.
* **`spectrum.ui`:** Updated to add the new "SDR Audio" menu item under the **Measurements** menu.
* **`sdr-audio.ui`:** Defines the user interface for the dedicated audio control popup.
* **`sdr-audio.py`:** The core engine containing the **`rtlsdr`** code responsible for:
    * Waveform capture and buffering.
    * Real-time AM/FM demodulation.
    * Audio playback via **`PyAudio`**.

### ⚠️ Known Issue: Audio Gaps (Underrun)

Due to processing overhead in the Python environment, the DSP pipeline sometimes runs slower than real-time:

* **Observation:** It currently takes approximately **1.1 seconds** to process and buffer **1.0 second** of raw RF data.
* **Result:** This results in a periodic **audio buffer underrun**, causing a small, temporary gap in the audio playback every few seconds. *This is an area for future optimization using techniques like circular buffers and cascaded decimation.*

***
The source code updates to be merged by first week of Jan 2026. 
An updated Screenshot will be forthcoming
<img width="1063" height="831" alt="image" src="https://github.com/user-attachments/assets/809dcdf3-7ac3-430b-9dce-1b4f9c9354e8" />
