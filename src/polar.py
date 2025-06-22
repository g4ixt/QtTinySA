#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jun 12 09:09:10 2025

@author: ian
"""
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtChart import QPolarChart, QChartView, QValueAxis, QScatterSeries

    self.polar = QPolarChart()
    chartView = QChartView(self.polar)

    layout = QVBoxLayout()
    layout.addWidget(chartView)

    #Let's create container widget for our chart, for example QFrame
    #Instead the MainWindow you should to substitute your own Widget or Main Form

    self.MyFrame = QtWidgets.QFrame(MainWindow)
    self.MyFrame.setGeometry(QtCore.QRect(0, 0, 1000, 1000))
    self.MyFrame.setLayout(layout)

    #setting axis
    axisy = QValueAxis()
    axisx = QValueAxis()

    axisy.setRange(0,500)
    axisy.setTickCount(4)
    self.polar.setAxisY(axisy)

    axisx.setRange(0,360)
    axisx.setTickCount(5)
    self.polar.setAxisX(axisx)

    #Let's draw scatter series
    self.polar_series = QScatterSeries()
    self.polar_series.setMarkerSize(5.0)

    self.polar_series.append(0, 0);
    self.polar_series.append(360, 500);

    #Why not draw archimedes spiral
    for i in range(0,360,10):
        self.polar_series.append(i, i)

    self.polar.addSeries(self.polar_series)