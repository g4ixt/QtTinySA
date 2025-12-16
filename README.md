# QtTinySA-SDRAudio

See: https://github.com/g4ixt/QtTinySA     for source for this repo

This Repo adds capability to play the audio from the selected frequency defined by Marker1.   It can demodulate both AM and FM signals (will AM demodulate if within the AM band, otherwise will FM demodulate)

This capability is supported by adding an RTL SDR to your system.   Suggest getting SDR++ or similar to get your RTL SDR working first before trying this code.

New Menu item added under the Measurements menu to select "SDR Audio" which will provide a popup window allowing you to enable Audio.  In that popup you can also adjust the frequency up or down to fine tune what frequency Marker 1 is set to.

Minimal changes were done to QtTinySA to provide the hook.  New file sdr-audio.ui defines the popup window and <fm5>.py contains the RTLSDR code that uses RTLSDR to perform audio decoding

<An updated Screen will be forthcoming>
<img width="1063" height="831" alt="image" src="https://github.com/user-attachments/assets/809dcdf3-7ac3-430b-9dce-1b4f9c9354e8" />
