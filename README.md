# QtTinySA  
![autoRBW_20230908_170224](https://github.com/g4ixt/QtTinySA/assets/76836635/a6c471e5-5de0-4e9c-b2fa-87d4fa0484d5)

A Python TinySA Ultra GUI programme using Qt5 and PyQt5 designed to run in Linux.  Should also now work with the original TinySA.  
See 'Wiki' for information about using the traces, markers, saving the spectrum, etc.  

The code attempts to replicate some of the TinySA Ultra on-screen commands on the PC.  Generator control seemed pointless so I have not added it.
Development took place on Kubuntu 22.04LTS with Python 3.9 and PyQt5 using Spyder in Anaconda.
Not tested in any version of Windows or Mac but it should work on both.

'TinySA' and 'TinySA Ultra' are trademarks of Erik Kaashoek and are used with his permission.

TinySA commands are based on Erik's Python examples:
http://athome.kaashoek.com/tinySA/python/

The serial communication commands are based on Martin's Python NanoVNA/TinySA Toolset
https://github.com/Ho-Ro

Dependencies: Install from the repository - python3-pyqt5, python3-numpy, python3-pyqtgraph
The 3D (time) spectrum requires OpenGL

The GUI was originally designed for a 7" 1024 x 600 screen but should maximise properly.  The GUI appearence may change significantly due to development.

There is limited error trapping.  The 3D spectrum can sometimes crash the Application, but this only happens if it's enabled.  

Working Features:  

Trace updates as tinySA scans  
LNA on/off  
SPUR auto/off  
Scan points selectable between 100 and 3,500.  
RBW control  
4 different colour traces, all can be turned on/off and set to average, max hold or min hold  
Selectable averaging  
Selectable memory depth.  
Amateur band frequency selection  
Start/stop frequency selection  
4 markers.  Each trace has 1 marker associated with it. Markers can be set to normal, delta, or one of the four largest peaks
with a selectable threshold.  
The selected marker(s) can be set to sweep start frequency with a button, or can be draggged to any desired frequency.  
All standard pyqtgraph features, selectable by right-click on the graph display, including:  
    Export as CSV, HDF5, Image file of various types, Matplotlib window, SVG  
3D spectrum with measurements over time represented as a surface plot.  Can be clicked/zoomed/dragged/rotated using the mouse.  
The 3D spectrum can be enabled/disabled on its tab page.  Disabled by default.  

To do:  

Build an executable.
