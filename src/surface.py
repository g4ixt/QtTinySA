#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Nov 22 14:31:50 2025

@author: ian"""

# Based on an example, which is:
# Copyright (C) 2023 The Qt Company Ltd.
# SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

import sys

from PySide6.QtCore import QSize
from PySide6.QtGui import QVector3D
from PySide6.QtGraphs import (QSurfaceDataItem, QSurface3DSeries)
from PySide6.QtGraphsWidgets import (Q3DSurfaceWidgetItem)
from PySide6.QtQuickWidgets import QQuickWidget
from PySide6.QtWidgets import QApplication

# if __name__ == '__main__':
#     app = QApplication(sys.argv)


#     window = QQuickWidget()
#     surface.setWidget(window)

class graph_3d():
    def __init__(self):
        self.surface = Q3DSurfaceWidgetItem()

    def create(self):
        axis = self.surface.axisX()
        axis.setTitle("Frequency")
        axis.setTitleVisible(True)
        axis = self.surface.axisY()
        axis.setTitle("Level")
        axis.setTitleVisible(True)
        axis = self.surface.axisZ()
        axis.setTitle("Time")
        axis.setTitleVisible(True)

        data = []
        data_row1 = [QSurfaceDataItem(QVector3D(0, 0.1, 0.5)),
                     QSurfaceDataItem(QVector3D(1, 0.5, 0.5))]
        data.append(data_row1)
        data_row2 = [QSurfaceDataItem(QVector3D(0, 1.8, 1)),
                     QSurfaceDataItem(QVector3D(1, 1.2, 1))]
        data.append(data_row2)

        series = QSurface3DSeries()
        series.dataProxy().resetArray(data)
        surface.addSeries(series)

    available_height = app.primaryScreen().availableGeometry().height()
    width = available_height * 4 / 5

