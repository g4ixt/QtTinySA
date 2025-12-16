# QtTinySA-SDRAudio

See: https://github.com/g4ixt/QtTinySA     for source fork for this repo (on 12/16/2025)

This Repo adds capability to play the audio from the selected frequency defined by Marker1.   It can demodulate both AM and FM signals (will AM demodulate if within the AM band, otherwise will FM demodulate)

So why do this?   Well with QtTinySA you see the spectrum between the start/stop frequency.  if you see a signal, you can move Marker 1 to that point and enable the audio.  You will then be able to listen to that frequency via the SDR.  You can also adjust the monitoring frequency by either moving Marker1 via the QtTinySA display, or use the "+" or "-"  buttons on the SDR Audio popup to do minor adjustment to the frequency.  Yes, SDR++ and others allows one to do this but is limited to the frequency range that can be monitored.  The TinySA has a much larger range.

This capability is supported by adding an RTL SDR to your system.   Suggest getting SDR++ or similar to get your RTL SDR working first before trying this code.   So you will have both the TinySA and RTL SDR connected to USB to your PC.

New Menu item added under the Measurements menu to select "SDR Audio" which will provide a popup window allowing you to enable Audio.  In that popup you can also adjust the frequency up or down to fine tune what frequency Marker 1 is set to.

Minimal changes were done to QtTinySA.py to provide the hook and an update to spectrum.ui to add menu item.  New file sdr-audio.ui defines the popup window and sdr-audio.py contains the RTLSDR code that uses RTLSDR to perform waveform capture and then audio playback.

Note: On my PC it takes ~1.1seconds to capture/buffer 1.0 seconds of the frequency data from the SDR.  So every few seconds there is a small audio gap since we end up with a underrun buffer.

<An updated Screen will be forthcoming>
<img width="1063" height="831" alt="image" src="https://github.com/user-attachments/assets/809dcdf3-7ac3-430b-9dce-1b4f9c9354e8" />
