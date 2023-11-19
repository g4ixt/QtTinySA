# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'preferences.ui'
#
# Created by: PyQt5 UI code generator 5.15.7
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Preferences(object):
    def setupUi(self, Preferences):
        Preferences.setObjectName("Preferences")
        Preferences.resize(600, 350)
        self.gridLayout_2 = QtWidgets.QGridLayout(Preferences)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.gridLayout = QtWidgets.QGridLayout()
        self.gridLayout.setObjectName("gridLayout")
        self.autoPoints = QtWidgets.QGroupBox(Preferences)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.autoPoints.sizePolicy().hasHeightForWidth())
        self.autoPoints.setSizePolicy(sizePolicy)
        self.autoPoints.setObjectName("autoPoints")
        self.minPoints = QtWidgets.QSpinBox(self.autoPoints)
        self.minPoints.setGeometry(QtCore.QRect(160, 60, 90, 27))
        self.minPoints.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
        self.minPoints.setMinimum(25)
        self.minPoints.setMaximum(450)
        self.minPoints.setProperty("value", 450)
        self.minPoints.setObjectName("minPoints")
        self.maxPoints = QtWidgets.QSpinBox(self.autoPoints)
        self.maxPoints.setGeometry(QtCore.QRect(160, 90, 90, 27))
        self.maxPoints.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
        self.maxPoints.setMinimum(25)
        self.maxPoints.setMaximum(100000)
        self.maxPoints.setProperty("value", 30000)
        self.maxPoints.setObjectName("maxPoints")
        self.label_3 = QtWidgets.QLabel(self.autoPoints)
        self.label_3.setGeometry(QtCore.QRect(30, 60, 111, 24))
        self.label_3.setObjectName("label_3")
        self.label_4 = QtWidgets.QLabel(self.autoPoints)
        self.label_4.setGeometry(QtCore.QRect(30, 90, 111, 24))
        self.label_4.setObjectName("label_4")
        self.bestPoints = QtWidgets.QCheckBox(self.autoPoints)
        self.bestPoints.setGeometry(QtCore.QRect(10, 30, 161, 24))
        self.bestPoints.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.bestPoints.setObjectName("bestPoints")
        self.gridLayout.addWidget(self.autoPoints, 1, 1, 1, 1)
        self.signalLines = QtWidgets.QGroupBox(Preferences)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.signalLines.sizePolicy().hasHeightForWidth())
        self.signalLines.setSizePolicy(sizePolicy)
        self.signalLines.setObjectName("signalLines")
        self.neg25Line = QtWidgets.QCheckBox(self.signalLines)
        self.neg25Line.setGeometry(QtCore.QRect(10, 30, 221, 24))
        self.neg25Line.setChecked(False)
        self.neg25Line.setObjectName("neg25Line")
        self.zeroLine = QtWidgets.QCheckBox(self.signalLines)
        self.zeroLine.setGeometry(QtCore.QRect(10, 60, 221, 24))
        self.zeroLine.setChecked(False)
        self.zeroLine.setObjectName("zeroLine")
        self.plus6Line = QtWidgets.QCheckBox(self.signalLines)
        self.plus6Line.setGeometry(QtCore.QRect(10, 90, 221, 24))
        self.plus6Line.setChecked(False)
        self.plus6Line.setObjectName("plus6Line")
        self.gridLayout.addWidget(self.signalLines, 2, 1, 1, 1)
        self.FreqBands = QtWidgets.QGroupBox(Preferences)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.FreqBands.sizePolicy().hasHeightForWidth())
        self.FreqBands.setSizePolicy(sizePolicy)
        self.FreqBands.setObjectName("FreqBands")
        self.freqBands = QtWidgets.QTableView(self.FreqBands)
        self.freqBands.setGeometry(QtCore.QRect(10, 30, 271, 211))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.freqBands.sizePolicy().hasHeightForWidth())
        self.freqBands.setSizePolicy(sizePolicy)
        self.freqBands.setObjectName("freqBands")
        self.addRow = QtWidgets.QPushButton(self.FreqBands)
        self.addRow.setEnabled(True)
        self.addRow.setGeometry(QtCore.QRect(190, 250, 90, 26))
        self.addRow.setObjectName("addRow")
        self.deleteRow = QtWidgets.QPushButton(self.FreqBands)
        self.deleteRow.setEnabled(True)
        self.deleteRow.setGeometry(QtCore.QRect(190, 290, 90, 26))
        self.deleteRow.setObjectName("deleteRow")
        self.rowUp = QtWidgets.QToolButton(self.FreqBands)
        self.rowUp.setGeometry(QtCore.QRect(10, 250, 25, 25))
        self.rowUp.setArrowType(QtCore.Qt.UpArrow)
        self.rowUp.setObjectName("rowUp")
        self.rowDown = QtWidgets.QToolButton(self.FreqBands)
        self.rowDown.setGeometry(QtCore.QRect(10, 290, 25, 25))
        self.rowDown.setArrowType(QtCore.Qt.DownArrow)
        self.rowDown.setObjectName("rowDown")
        self.label_5 = QtWidgets.QLabel(self.FreqBands)
        self.label_5.setGeometry(QtCore.QRect(50, 250, 131, 61))
        font = QtGui.QFont()
        font.setItalic(True)
        self.label_5.setFont(font)
        self.label_5.setWordWrap(True)
        self.label_5.setObjectName("label_5")
        self.gridLayout.addWidget(self.FreqBands, 0, 0, 4, 1)
        self.gridLayout_2.addLayout(self.gridLayout, 0, 0, 1, 1)

        self.retranslateUi(Preferences)
        QtCore.QMetaObject.connectSlotsByName(Preferences)

    def retranslateUi(self, Preferences):
        _translate = QtCore.QCoreApplication.translate
        Preferences.setWindowTitle(_translate("Preferences", "Preferences"))
        self.autoPoints.setTitle(_translate("Preferences", "Auto Points"))
        self.label_3.setText(_translate("Preferences", "Auto min points"))
        self.label_4.setText(_translate("Preferences", "Auto max points"))
        self.bestPoints.setText(_translate("Preferences", "Best Power Accuracy"))
        self.signalLines.setTitle(_translate("Preferences", "Signal level reminder lines  "))
        self.neg25Line.setText(_translate("Preferences", "-25dBm  (best accuracy)"))
        self.zeroLine.setText(_translate("Preferences", " 0dBm    (attenuator on Auto)"))
        self.plus6Line.setText(_translate("Preferences", "Absolute maximum of +6dBm"))
        self.FreqBands.setTitle(_translate("Preferences", "Frequency Bands  "))
        self.addRow.setText(_translate("Preferences", "Add"))
        self.deleteRow.setText(_translate("Preferences", "Delete"))
        self.rowUp.setText(_translate("Preferences", "..."))
        self.rowDown.setText(_translate("Preferences", "..."))
        self.label_5.setText(_translate("Preferences", "use arrows to select row for add/delete"))
