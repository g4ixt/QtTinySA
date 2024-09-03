# QtTinySA  
![v0-11-7](https://github.com/user-attachments/assets/2d9ebcd7-d91c-49c7-a89e-8e53a9587727)

A Python 'TinySA Ultra' (and original 'TinySA') GUI programme using Qt5 and PyQt5. Designed to run in Linux but also works in Windows (minimal testing) and mac (no testing).  
The Windows executable does not work on Windows versions < 10. 

The code attempts to replicate some of the TinySA Ultra on-screen commands on the PC.  Generator control seemed pointless so I have not added it.
Development and testing are now on Kubuntu 24.04LTS with Python 3.11.8 and PyQt5 using Spyder.

'TinySA', 'TinySA Ultra' and the TinySA icon are trademarks of Erik Kaashoek and are used with his permission.

TinySA commands are based on Erik's Python examples:
http://athome.kaashoek.com/tinySA/python/

The serial communication commands are based on Martin's Python NanoVNA/TinySA Toolset
https://github.com/Ho-Ro

Dependencies: Install from your repository - numpy, pyqtgraph, pyopengl, pyqt5, pyserial, platformdirs.  
You may need to add the Qt5 SQLite 3 database driver (libqt5sql5-sqlite) and Python3 bindings for QT5's SQL Module.
The 3D (time) spectrum requires OpenGL

The GUI was originally designed for a 7" 1024 x 600 screen but should maximise properly.  The GUI appearence may change significantly due to development.
