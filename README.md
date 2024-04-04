# QtTinySA  
![Screenshot_20240404_143416](https://github.com/g4ixt/QtTinySA/assets/76836635/47d9e6e9-daeb-4ae3-8e9c-91b5f041f6ff)

A Python 'TinySA Ultra' (and original 'TinySA') GUI programme using Qt5 and PyQt5. Designed to run in Linux but works in Windows (minimal testing) and mac (no testing).  
The Windows executable does not work on Windows versions < 10. 

The code attempts to replicate some of the TinySA Ultra on-screen commands on the PC.  Generator control seemed pointless so I have not added it.
Development took place on Kubuntu 22.04LTS with Python 3.9.18 and PyQt5 using Spyder in Anaconda.

'TinySA', 'TinySA Ultra' and the TinySA icon are trademarks of Erik Kaashoek and are used with his permission.

TinySA commands are based on Erik's Python examples:
http://athome.kaashoek.com/tinySA/python/

The serial communication commands are based on Martin's Python NanoVNA/TinySA Toolset
https://github.com/Ho-Ro

Dependencies: Install from your repository - numpy, pyqtgraph, pyopengl, pyqt5, pyserial, platformdirs.  
You may need to add the Qt5 SQLite 3 database driver (libqt5sql5-sqlite) and Python3 bindings for QT5's SQL Module.  You may need to update 'platformdirs'.
If you get a serial exception on startup, add your username to the 'dialout' group.  
The 3D (time) spectrum requires OpenGL

The GUI was originally designed for a 7" 1024 x 600 screen but should maximise properly.  The GUI appearence may change significantly due to development.

There is limited error trapping.
