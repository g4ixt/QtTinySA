"""
==========
Polar plot
==========

Demo of a line plot on a polar axis.
"""
import os
import matplotlib.pyplot as plt
import numpy as np
# import pyqtgraph
import QtTSApattern
# from PyQt5 import QtWidgets, QtCore
# from PyQt5.QtGui import QIcon

import sys
import matplotlib
matplotlib.use('Qt5Agg')

from PyQt5 import QtCore, QtWidgets

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure


# create QApplication for the GUI
# app.setApplicationName('QtTinySA')
# app.setApplicationVersion(' v1.1.2')
ui = QtTSApattern.Ui_Pattern()
# ui.setupUi(window)

# pnwindow is the phase noise display window
# polarwindow = QtWidgets.QDialog()
# pnwindow.setWindowIcon(QIcon(os.path.join(basedir, 'tinySAsmall.png')))
# polar = QtTSApattern.Ui_Pattern()
# polar.setupUi(polarwindow)

r = np.arange(0, 2, 0.01)
theta = 2 * np.pi * r

fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
ax.plot(theta, r)
ax.set_rmax(2)
ax.set_rticks([0.5, 1, 1.5, 2])  # Less radial ticks
ax.set_rlabel_position(-22.5)  # Move radial labels away from plotted line
ax.grid(True)

ax.set_title("A line plot on a polar axis", va='bottom')
plt.show()

QtTSApattern.pattern.polarplot.ax.plot(theta, r)
