# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'preferences.ui'
#
# Created by: PyQt5 UI code generator 5.15.10
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Preferences(object):
    def setupUi(self, Preferences):
        Preferences.setObjectName("Preferences")
        Preferences.resize(817, 600)
        self.layoutWidget = QtWidgets.QWidget(Preferences)
        self.layoutWidget.setGeometry(QtCore.QRect(10, 10, 411, 572))
        self.layoutWidget.setObjectName("layoutWidget")
        self.gridLayout = QtWidgets.QGridLayout(self.layoutWidget)
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.gridLayout.setObjectName("gridLayout")
        self.label_5 = QtWidgets.QLabel(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_5.sizePolicy().hasHeightForWidth())
        self.label_5.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_5.setFont(font)
        self.label_5.setObjectName("label_5")
        self.gridLayout.addWidget(self.label_5, 0, 0, 1, 3)
        self.addRow = QtWidgets.QPushButton(self.layoutWidget)
        self.addRow.setEnabled(True)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.addRow.sizePolicy().hasHeightForWidth())
        self.addRow.setSizePolicy(sizePolicy)
        self.addRow.setObjectName("addRow")
        self.gridLayout.addWidget(self.addRow, 4, 2, 1, 1)
        self.freqBands = QtWidgets.QTableView(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.freqBands.sizePolicy().hasHeightForWidth())
        self.freqBands.setSizePolicy(sizePolicy)
        self.freqBands.setMinimumSize(QtCore.QSize(0, 450))
        self.freqBands.setObjectName("freqBands")
        self.gridLayout.addWidget(self.freqBands, 1, 0, 2, 4)
        self.deleteRow = QtWidgets.QPushButton(self.layoutWidget)
        self.deleteRow.setEnabled(True)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.deleteRow.sizePolicy().hasHeightForWidth())
        self.deleteRow.setSizePolicy(sizePolicy)
        self.deleteRow.setObjectName("deleteRow")
        self.gridLayout.addWidget(self.deleteRow, 4, 3, 1, 1)
        self.deleteAll = QtWidgets.QPushButton(self.layoutWidget)
        self.deleteAll.setObjectName("deleteAll")
        self.gridLayout.addWidget(self.deleteAll, 5, 3, 1, 1)
        self.label_12 = QtWidgets.QLabel(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_12.sizePolicy().hasHeightForWidth())
        self.label_12.setSizePolicy(sizePolicy)
        self.label_12.setObjectName("label_12")
        self.gridLayout.addWidget(self.label_12, 3, 0, 1, 1)
        self.filterBox = QtWidgets.QComboBox(self.layoutWidget)
        self.filterBox.setEditable(False)
        self.filterBox.setObjectName("filterBox")
        self.gridLayout.addWidget(self.filterBox, 3, 1, 1, 1)
        self.exportButton = QtWidgets.QPushButton(self.layoutWidget)
        self.exportButton.setEnabled(True)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.exportButton.sizePolicy().hasHeightForWidth())
        self.exportButton.setSizePolicy(sizePolicy)
        self.exportButton.setObjectName("exportButton")
        self.gridLayout.addWidget(self.exportButton, 4, 1, 1, 1)
        self.importButton = QtWidgets.QPushButton(self.layoutWidget)
        self.importButton.setEnabled(True)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.importButton.sizePolicy().hasHeightForWidth())
        self.importButton.setSizePolicy(sizePolicy)
        self.importButton.setObjectName("importButton")
        self.gridLayout.addWidget(self.importButton, 4, 0, 1, 1)
        self.layoutWidget1 = QtWidgets.QWidget(Preferences)
        self.layoutWidget1.setGeometry(QtCore.QRect(440, 10, 361, 571))
        self.layoutWidget1.setObjectName("layoutWidget1")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.layoutWidget1)
        self.gridLayout_2.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.label_11 = QtWidgets.QLabel(self.layoutWidget1)
        self.label_11.setObjectName("label_11")
        self.gridLayout_2.addWidget(self.label_11, 1, 0, 1, 1)
        self.plus6Line = QtWidgets.QCheckBox(self.layoutWidget1)
        self.plus6Line.setChecked(False)
        self.plus6Line.setObjectName("plus6Line")
        self.gridLayout_2.addWidget(self.plus6Line, 1, 1, 1, 1)
        self.minPoints = QtWidgets.QSpinBox(self.layoutWidget1)
        self.minPoints.setMaximumSize(QtCore.QSize(100, 16777215))
        self.minPoints.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
        self.minPoints.setMinimum(25)
        self.minPoints.setMaximum(450)
        self.minPoints.setProperty("value", 450)
        self.minPoints.setObjectName("minPoints")
        self.gridLayout_2.addWidget(self.minPoints, 6, 1, 1, 1)
        spacerItem = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.gridLayout_2.addItem(spacerItem, 16, 0, 1, 1)
        self.freqLO = QtWidgets.QDoubleSpinBox(self.layoutWidget1)
        self.freqLO.setDecimals(6)
        self.freqLO.setMinimum(0.0)
        self.freqLO.setMaximum(100000.0)
        self.freqLO.setProperty("value", 0.0)
        self.freqLO.setObjectName("freqLO")
        self.gridLayout_2.addWidget(self.freqLO, 10, 1, 1, 1)
        self.label_8 = QtWidgets.QLabel(self.layoutWidget1)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_8.setFont(font)
        self.label_8.setObjectName("label_8")
        self.gridLayout_2.addWidget(self.label_8, 4, 0, 1, 1)
        self.label_6 = QtWidgets.QLabel(self.layoutWidget1)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_6.setFont(font)
        self.label_6.setObjectName("label_6")
        self.gridLayout_2.addWidget(self.label_6, 0, 0, 1, 2)
        self.peakThreshold = QtWidgets.QSpinBox(self.layoutWidget1)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.peakThreshold.sizePolicy().hasHeightForWidth())
        self.peakThreshold.setSizePolicy(sizePolicy)
        self.peakThreshold.setMaximumSize(QtCore.QSize(100, 16777215))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.peakThreshold.setFont(font)
        self.peakThreshold.setPrefix("")
        self.peakThreshold.setMinimum(-120)
        self.peakThreshold.setMaximum(-20)
        self.peakThreshold.setProperty("value", -90)
        self.peakThreshold.setObjectName("peakThreshold")
        self.gridLayout_2.addWidget(self.peakThreshold, 12, 1, 1, 1)
        self.label_4 = QtWidgets.QLabel(self.layoutWidget1)
        self.label_4.setObjectName("label_4")
        self.gridLayout_2.addWidget(self.label_4, 7, 0, 1, 1)
        self.label_3 = QtWidgets.QLabel(self.layoutWidget1)
        self.label_3.setObjectName("label_3")
        self.gridLayout_2.addWidget(self.label_3, 6, 0, 1, 1)
        self.label_16 = QtWidgets.QLabel(self.layoutWidget1)
        self.label_16.setObjectName("label_16")
        self.gridLayout_2.addWidget(self.label_16, 10, 0, 1, 1)
        self.highLO = QtWidgets.QCheckBox(self.layoutWidget1)
        self.highLO.setObjectName("highLO")
        self.gridLayout_2.addWidget(self.highLO, 9, 1, 1, 1)
        self.label_17 = QtWidgets.QLabel(self.layoutWidget1)
        self.label_17.setObjectName("label_17")
        self.gridLayout_2.addWidget(self.label_17, 13, 0, 1, 1)
        self.neg25Line = QtWidgets.QCheckBox(self.layoutWidget1)
        self.neg25Line.setChecked(False)
        self.neg25Line.setObjectName("neg25Line")
        self.gridLayout_2.addWidget(self.neg25Line, 3, 1, 1, 1)
        self.maxFreqBox = QtWidgets.QSpinBox(self.layoutWidget1)
        self.maxFreqBox.setMaximumSize(QtCore.QSize(100, 16777215))
        self.maxFreqBox.setMinimum(350)
        self.maxFreqBox.setMaximum(20000)
        self.maxFreqBox.setSingleStep(100)
        self.maxFreqBox.setProperty("value", 6000)
        self.maxFreqBox.setObjectName("maxFreqBox")
        self.gridLayout_2.addWidget(self.maxFreqBox, 14, 1, 1, 1)
        self.label_10 = QtWidgets.QLabel(self.layoutWidget1)
        self.label_10.setObjectName("label_10")
        self.gridLayout_2.addWidget(self.label_10, 2, 0, 1, 1)
        self.label = QtWidgets.QLabel(self.layoutWidget1)
        self.label.setObjectName("label")
        self.gridLayout_2.addWidget(self.label, 5, 0, 1, 1)
        self.label_14 = QtWidgets.QLabel(self.layoutWidget1)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_14.setFont(font)
        self.label_14.setObjectName("label_14")
        self.gridLayout_2.addWidget(self.label_14, 8, 0, 1, 2)
        self.syncTime = QtWidgets.QCheckBox(self.layoutWidget1)
        self.syncTime.setObjectName("syncTime")
        self.gridLayout_2.addWidget(self.syncTime, 13, 1, 1, 1)
        self.label_7 = QtWidgets.QLabel(self.layoutWidget1)
        self.label_7.setObjectName("label_7")
        self.gridLayout_2.addWidget(self.label_7, 14, 0, 1, 1)
        self.maxPoints = QtWidgets.QSpinBox(self.layoutWidget1)
        self.maxPoints.setMaximumSize(QtCore.QSize(100, 16777215))
        self.maxPoints.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
        self.maxPoints.setMinimum(25)
        self.maxPoints.setMaximum(100000)
        self.maxPoints.setProperty("value", 30000)
        self.maxPoints.setObjectName("maxPoints")
        self.gridLayout_2.addWidget(self.maxPoints, 7, 1, 1, 1)
        self.zeroLine = QtWidgets.QCheckBox(self.layoutWidget1)
        self.zeroLine.setChecked(False)
        self.zeroLine.setObjectName("zeroLine")
        self.gridLayout_2.addWidget(self.zeroLine, 2, 1, 1, 1)
        self.label_9 = QtWidgets.QLabel(self.layoutWidget1)
        self.label_9.setObjectName("label_9")
        self.gridLayout_2.addWidget(self.label_9, 3, 0, 1, 1)
        self.label_15 = QtWidgets.QLabel(self.layoutWidget1)
        self.label_15.setObjectName("label_15")
        self.gridLayout_2.addWidget(self.label_15, 9, 0, 1, 1)
        self.label_2 = QtWidgets.QLabel(self.layoutWidget1)
        self.label_2.setObjectName("label_2")
        self.gridLayout_2.addWidget(self.label_2, 12, 0, 1, 1)
        self.label_13 = QtWidgets.QLabel(self.layoutWidget1)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_13.setFont(font)
        self.label_13.setObjectName("label_13")
        self.gridLayout_2.addWidget(self.label_13, 11, 0, 1, 1)
        self.rbw_x = QtWidgets.QSpinBox(self.layoutWidget1)
        self.rbw_x.setMaximumSize(QtCore.QSize(100, 16777215))
        self.rbw_x.setMinimum(2)
        self.rbw_x.setMaximum(10)
        self.rbw_x.setProperty("value", 3)
        self.rbw_x.setObjectName("rbw_x")
        self.gridLayout_2.addWidget(self.rbw_x, 5, 1, 1, 1)
        self.label_18 = QtWidgets.QLabel(self.layoutWidget1)
        self.label_18.setObjectName("label_18")
        self.gridLayout_2.addWidget(self.label_18, 15, 0, 1, 1)
        self.deviceBox = QtWidgets.QComboBox(self.layoutWidget1)
        self.deviceBox.setObjectName("deviceBox")
        self.gridLayout_2.addWidget(self.deviceBox, 15, 1, 1, 1)

        self.retranslateUi(Preferences)
        QtCore.QMetaObject.connectSlotsByName(Preferences)

    def retranslateUi(self, Preferences):
        _translate = QtCore.QCoreApplication.translate
        Preferences.setWindowTitle(_translate("Preferences", "Preferences"))
        self.label_5.setText(_translate("Preferences", "Preset Bands and Markers"))
        self.addRow.setText(_translate("Preferences", "Add Row"))
        self.deleteRow.setText(_translate("Preferences", "Delete Row"))
        self.deleteAll.setText(_translate("Preferences", "Delete All"))
        self.label_12.setText(_translate("Preferences", "Filter on:"))
        self.exportButton.setText(_translate("Preferences", "Export"))
        self.importButton.setText(_translate("Preferences", "Import"))
        self.label_11.setText(_translate("Preferences", "Absolute maximum"))
        self.plus6Line.setText(_translate("Preferences", "+6dBm"))
        self.freqLO.setSuffix(_translate("Preferences", "MHz"))
        self.label_8.setText(_translate("Preferences", "Scan Points Settings"))
        self.label_6.setText(_translate("Preferences", "Signal Level Reminder lines"))
        self.peakThreshold.setSuffix(_translate("Preferences", "dBm"))
        self.label_4.setText(_translate("Preferences", "Auto maximum points"))
        self.label_3.setText(_translate("Preferences", "Auto minimum points"))
        self.label_16.setText(_translate("Preferences", "LO Frequency"))
        self.highLO.setText(_translate("Preferences", "True"))
        self.label_17.setText(_translate("Preferences", "Date and Time"))
        self.neg25Line.setText(_translate("Preferences", "-25dBm"))
        self.maxFreqBox.setToolTip(_translate("Preferences", "Maximum Frequency allowed.  Note: TinySA-Ultra is not calibrated above 6GHz "))
        self.maxFreqBox.setSuffix(_translate("Preferences", "MHz"))
        self.label_10.setText(_translate("Preferences", "Max with auto attenuator"))
        self.label.setText(_translate("Preferences", "Points / Resolution Bandwidth"))
        self.label_14.setText(_translate("Preferences", "External Mixer / LNB"))
        self.syncTime.setText(_translate("Preferences", "Sync to PC"))
        self.label_7.setText(_translate("Preferences", "Maximum operating frequency"))
        self.zeroLine.setText(_translate("Preferences", " 0dBm"))
        self.label_9.setText(_translate("Preferences", "Max for best accuracy"))
        self.label_15.setText(_translate("Preferences", "LO above displayed Freq"))
        self.label_2.setText(_translate("Preferences", "Max/Min marker threshold"))
        self.label_13.setText(_translate("Preferences", "Miscellaneous"))
        self.label_18.setText(_translate("Preferences", "Detected USB Port to use"))
        self.deviceBox.setToolTip(_translate("Preferences", "Select device to use if more than one is detected"))
