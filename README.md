# QtTinySA  
![Main_screen](https://github.com/user-attachments/assets/cb39e630-c1b1-4299-a2d3-c843c63f7591)

A Python 'TinySA Ultra' (and original 'TinySA') GUI programme using Qt5 and PyQt5. Designed to run in Linux but also works in mac (no testing)
and Windows.  
  
The code attempts to replicate some of the TinySA Ultra on-screen commands on the PC.  Generator control seemed pointless so I have not added it.
Development and testing are now on Kubuntu 24.04LTS with Python 3.11.8 and PyQt5 using Spyder.

'TinySA', 'TinySA Ultra' and the TinySA icon are trademarks of Erik Kaashoek and are used with his permission.

TinySA commands are based on Erik's Python examples:
http://athome.kaashoek.com/tinySA/python/

The serial communication commands are based on Martin's Python NanoVNA/TinySA Toolset
https://github.com/Ho-Ro
  
For running the Python code, install the modules listed in the 'requirements.txt' file.  
You may need to add the Qt5 SQLite 3 database driver (libqt5sql5-sqlite) and Python3 bindings for QT5's SQL Module.

The GUI was originally designed for a 7" 1024 x 600 screen but should maximise properly.  The GUI appearence may change significantly due to development.
