#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Created on Tue Jun 8 at 18:10:10 2021
@author: Ian Jefferson G4IXT
RF power meter programme for the AD8318/AD7887 power detector sold by Matkis SV1AFN https://www.sv1afn.com/

This code makes use of a subset of the code from touchstone.py from scikit-rf, an open-source
Python package for RF and Microwave applications.
"""

import time
import logging
import numpy as np
import queue
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtCore import pyqtSlot, pyqtSignal, QRunnable, QObject, QThreadPool
from PyQt5.QtWidgets import QMessageBox, QFileDialog, QDataWidgetMapper
from PyQt5.QtSql import QSqlDatabase, QSqlTableModel
import spidev
import pyqtgraph
import QtPowerMeter  # the GUI
from touchstone_subset import Touchstone  # for importing S2P files
spi = spidev.SpiDev()
threadpool = QThreadPool()

# AD7887 ADC control register setting for external reference, single channel, mode 3
dOut = 0b00100000  # power down when CS high

# Meter scale values
Units = [' pW', ' nW', ' uW', ' mW', ' W']
dB = [-60, -30, 0, 30, 60]

# Frequency radio button values
fBand = [14, 50, 70, 144, 432, 1296, 2320, 3400, 5700]

# calibration slope datasheet limits
fSpec = [900, 1900, 2200]
maxSlope = [-26*1.6384, -27*1.6384, -28*1.6384]  # Max spec mV converted to ADC Code
minSlope = [-23*1.6384, -22*1.6384, -21.5*1.6384]  # Min spec mV converted to ADC Code

logging.basicConfig(format="%(message)s", level=logging.INFO)

###############################################################################
# classes


class database():
    '''calibration and attenuator/coupler data are stored in a SQLite database'''

    def __init__(self):
        self.db = None

    def connect(self):
        self.db = QSqlDatabase.addDatabase('QSQLITE')
        if QtCore.QFile.exists('powerMeter.db'):
            logging.info('Open database')
            self.db.setDatabaseName('powerMeter.db')
            self.db.open()
        else:
            logging.info('Database file missing')
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Critical)
            msg.setText('Database file missing')
            msg.setStandardButtons(QMessageBox.Ok)
            msg.exec_()

    def disconnect(self):
        attenuators.tm.submitAll()
        parameters.tm.submitAll()
        calibration.tm.submitAll()
        del attenuators.tm
        del parameters.tm
        del calibration.tm
        self.db.close()


class WorkerSignals(QObject):
    error = pyqtSignal(str)
    result = pyqtSignal(np.ndarray, float, float)


class Worker(QRunnable):
    '''Worker threads so that measurements can run outside GUI event loop'''

    def __init__(self, fn):
        super(Worker, self).__init__()
        self.fn = fn
        self.signals = WorkerSignals()

    @pyqtSlot()
    def run(self):
        '''Initialise the runner'''

        logging.info(f'{self.fn.__name__} thread running')
        self.fn()


class Measurement():
    '''Read AD8318 RF Power value via AD7887 ADC using the Serial Peripheral Interface (SPI)'''

    def __init__(self):
        self.running = False
        self.sampleTimer = QtCore.QElapsedTimer()
        self.runTimer = QtCore.QElapsedTimer()
        self.sampleCounter = 0
        self.block = 500
        self.signals = WorkerSignals()
        self.signals.result.connect(self.updateGUI)
        self.signals.error.connect(spiError)
        self.fifo = queue.SimpleQueue()

    def startMeasurement(self):
        self.spiTransaction = Worker(self.readSPI)  # workers are auto-deleted when thread stops
        self.powerResults = Worker(self.calcPowers)
        try:
            openSPI()
        except FileNotFoundError:
            popUp('No SPI device found', 'OK')
        else:
            activeButtons(False)
            selectCal()
            sumLosses()
            self.running = True
            self.setTimebase()
            self.sampleCounter = 0
            self.sampleTimer.start()
            threadpool.start(self.spiTransaction)
            threadpool.start(self.powerResults)

    def readSPI(self):  # always threaded
        while self.running:
            dIn = spi.xfer([dOut, dOut])  # dOut = Pi to AD7887, MOSI. dIn = measurement result, MISO.
            if dIn[0] > 13:
                self.signals.error.emit('SPI error')  # anything > 13 is due to noise or spi errors
                return
            dIn = (dIn[0] << 8) + dIn[1]  # shift first byte to be MSB of a 12-bit word and add second byte
            self.fifo.put(dIn)  # put measurement onto FIFO queue

    def calcPowers(self):  # always threaded
        buffer = np.zeros(self.block, dtype=float)  # a buffer is necessary for high sample rate
        while self.running or self.fifo.qsize() > 0:  # empty the fifo queue on stop, or App hangs
            for i in range(self.block):
                buffer[i] = self.fifo.get(block=True, timeout=None)  # pop measurement from FIFO queue
            buffer = np.divide(buffer, calibration.slope)
            buffer = np.add(buffer, calibration.intercept)
            self.runTimer.start()
            self.yAxis = np.roll(self.yAxis, -self.block)  # roll the array left
            self.yAxis[-self.block:] = buffer  # over-write rolled data with new data
            averagePower = np.average(self.yAxis[-self.averages:])
            measuredPdBm = averagePower - attenuators.loss  # subtract total loss of couplers and attenuators
            self.signals.result.emit(self.yAxis, averagePower, measuredPdBm)  # pass to UpdateGUI

    def setTimebase(self):
        self.samples = ui.memorySize.value() * 1000  # memory size in kSamples
        self.averages = ui.averaging.value()
        self.xAxis = np.arange(0, self.samples, 1, dtype=int)  # fill the np array with consecutive integers
        self.yAxis = np.full(self.samples, -75, dtype=float)  # fill the np array with each element = 75

    def updateGUI(self, Axis, avgP, PdBm):
        # update power meter range and label
        if ui.autoRangeButton.isChecked():
            try:
                # determine if the power units are nW, uW, mW, or W
                self.scale = next(index for index, listValue in enumerate(dB) if listValue > PdBm)
                ui.powerUnit.setText(str(Units[self.scale]))
                ui.powerWatts.setSuffix(str(Units[self.scale]))
            except StopIteration:
                logging.info(f'range error = {self.scale}')
                spiError('Range error')
                stopMeter()
                return
        else:
            self.userRange  # set the units from the main steps of the slider

        # convert to display according to meter range selected
        power = 10 ** ((PdBm - dB[self.scale-1]) / 10)
        ui.meterWidget.set_MaxValue(10.0)  # a max of 1 on the widget doesn't work
        if power >= 1:
            ui.meterWidget.set_MaxValue(10.0)
        if power >= 10:
            ui.meterWidget.set_MaxValue(100.0)
        if power >= 100:
            ui.meterWidget.set_MaxValue(1000.0)

        # update boxes on Display tab (uses the average powers)
        self.sampleCounter += self.block
        sampleRate = self.sampleCounter / (self.sampleTimer.nsecsElapsed()/1e9)
        ui.measurementRate.setValue(sampleRate)
        ui.sensorPower.setValue(avgP)
        ui.inputPower.setValue(PdBm)

        # update the analogue gauge widget (uses the average powers)
        ui.meterWidget.update_value(power, mouse_controlled=False)
        ui.powerWatts.setValue(power)

        # update the moving pyqtgraph (uses sampled powers with no averaging)
        powerCurve.setData(self.xAxis, Axis)

    def userRange(self):
        # set the units from the main steps of the slider
        self.scale = ui.rangeSlider.value()
        ui.powerUnit.setText(Units[int(self.scale)])
        ui.powerWatts.setSuffix(Units[int(self.scale)])


class modelView():
    '''set up and process data models bound to the GUI widgets'''

    def __init__(self, tableName, graphName):
        self.table = tableName
        self.graphName = graphName
        self.tm = QSqlTableModel()
        self.dwm = QDataWidgetMapper()
        self.loss = 0
        self.marker = self.graphName.addLine(0, 90, movable=True, pen='g', label="{value:.2f}")
        self.marker.label.setPosition(0.1)
        self.curve = self.graphName.plot([], [], name='', pen='r')

    def createTableModel(self):
        # add exception handling?
        self.tm.setTable(self.table)
        self.dwm.setModel(self.tm)
        self.dwm.setSubmitPolicy(QDataWidgetMapper.ManualSubmit)

    def insertData(self, AssetID, Freq, Loss):  # used by ImportS2P
        record = self.tm.record()
        if AssetID != '':
            record.setValue('AssetID', AssetID)
        if Freq != '':
            record.setValue('FreqMHz', Freq)
        if Loss != '':
            record.setValue('ValuedB', Loss)
        self.tm.insertRecord(-1, record)
        self.updateModel()
        self.dwm.submit()
        app.processEvents()

    def saveChanges(self):
        self.dwm.submit()
        sumLosses()
        selectCal()
        if self.table != 'Calibration':
            parameters.tm.setFilter('AssetID =' + str(ui.assetID.value()))
        if self.table != 'Device':
            self.showCurve()

    def deleteRow(self):
        cI = self.dwm.currentIndex()
        self.dwm.toPrevious()
        self.tm.removeRow(cI)
        self.tm.submit()
        app.processEvents()

    def updateModel(self):  # model must be re-populated when python code changes data
        self.tm.select()
        self.tm.layoutChanged.emit()

    def showCurve(self):
        # plot calibration slope or device loss in GUI
        freqs = []
        y = []
        curveType = 'ValuedB'
        if self.table == 'Calibration':
            curveType = 'Slope'
        self.tm.sort(self.tm.fieldIndex('FreqMHz'), QtCore.Qt.AscendingOrder)
        #  iterate through selected values and display on graph
        for i in range(0, self.tm.rowCount()):
            freqs.append(self.tm.record(i).value('FreqMHz'))
            y.append(self.tm.record(i).value(curveType))
        self.curve.setData(freqs, y)

    def updateSpinBox(self):
        # update GUI boxes to discrete value marker was dragged to
        try:
            for i in range(self.tm.rowCount()):
                if self.tm.record(i).value('FreqMHz') <= self.marker.value():
                    self.dwm.setCurrentIndex(i)
            self.marker.setValue(self.tm.record(self.dwm.currentIndex()).value('FreqMHz'))
        except TypeError:
            return


###############################################################################
# other methods

def spiError(message):
    logging.info('SPI error function called')
    ui.spiNoise.setText(message)


def exit_handler():
    meter.running = False
    while meter.fifo.qsize() > 0:
        time.sleep(0.2)  # allow time for the fifo queue to empty
    app.processEvents()
    config.disconnect()
    spi.close()
    logging.info('Closed cleanly')


def importS2P():
    # Import the (Touchstone) file of attenuator/coupler/cable calibration data
    index = ui.assetID.value()  # if User clicks arrows during import, data would associate with wrong Device

    # pop up a dialogue box for user to select the file
    s2pFile = QFileDialog.getOpenFileName(None, 'Import s-parameter file for selected device', '', '*.s2p')
    sParam = Touchstone(s2pFile[0])  # use skrf.io method to read file - error trapping needed.  Very slow.

    # extract the device parameters (insertion loss or coupling factors)
    sParamData = sParam.get_sparameter_data('db')
    Freq = sParamData['frequency'].tolist()
    Loss = sParamData['S21DB'].tolist()

    # insert the nominal value to the Device data table
    ui.nominaldB.setValue(Loss[int(len(Freq)/2)])
    attenuators.dwm.submit()

    # read the frequency and S21 data from the lists and insert into model
    deleteAllFreq()
    activeButtons(False)
    logging.info(f'Inserting {len(Freq)} records')
    for i in range(len(Freq)):
        parameters.insertData(index, Freq[i]/1e6, round(Loss[i], 3))  # 2 decimals plenty, 3 to minimise rounding error
        parameters.marker.setValue(Freq[i]/1e6)
    prevParam()
    parameters.showCurve()
    activeButtons(True)


def sumLosses():  # this might be better done with a relational query? (to avoid changing the model filter)
    attenuators.tm.setFilter('inUse =' + str(1))  # set model to devices in use
    attenuators.updateModel()
    attenuators.loss = 0

    # future - check for devices being used out of their operating freq band

    for i in range(attenuators.tm.rowCount()):
        freqList = []
        lossList = []
        deviceRecord = attenuators.tm.record(i)
        asid = deviceRecord.value('AssetID')
        parameters.tm.setFilter('AssetID =' + str(asid))  # filter to only parameters of device i
        parameters.tm.sort(1, 0)  # sort by frequency, ascending: required for numpy interpolate

        # copy the parameters into lists.  There must be a better way...
        for j in range(parameters.tm.rowCount()):
            parameterRecord = parameters.tm.record(j)
            freqList.append(parameterRecord.value('FreqMHz'))
            lossList.append(parameterRecord.value('ValuedB'))
        # interpolate device loss at set freq from the known parameters and sum them
        attenuators.loss += np.interp(ui.freqBox.value(), freqList, lossList)
    ui.totalLoss.setValue(-attenuators.loss)
    attenuators.tm.setFilter('')
    parameters.tm.setFilter('')


def selectCal():
    # select nearest calibration frequency to measurement frequency
    difference = 6000
    for i in range(calibration.tm.rowCount()-1):
        calRecord = calibration.tm.record(i)
        if difference > abs(ui.freqBox.value()-calRecord.value('FreqMHz')):
            difference = abs(ui.freqBox.value()-calRecord.value('FreqMHz'))
            calibration.slope = calRecord.value('Slope')
            calibration.intercept = calRecord.value('Intercept')
            ui.calQualLabel.setText(calRecord.value('CalQuality') + " " + str(int(calRecord.value('FreqMHz'))) + "MHz")


def popUp(message, button):
    msg = QMessageBox(parent=(window))
    msg.setIcon(QMessageBox.Warning)
    msg.setText(message)
    msg.addButton(button, QMessageBox.ActionRole)
    msg.exec_()

##############################################################################
# respond to GUI signals


def deleteFreq():  # delete parameter for the frequency shown in the GUI
    parameters.deleteRow()
    parameters.marker.setValue(ui.freqIndex.value())
    parameters.showCurve()


def deleteAllFreq():
    activeButtons(False)
    parameters.tm.sort(parameters.tm.fieldIndex('FreqMHz'), QtCore.Qt.AscendingOrder)
    logging.info(f'Deleting {parameters.tm.rowCount()} records')
    for i in range(parameters.tm.rowCount()):
        try:
            parameters.marker.setValue((parameters.tm.record(i).value('FreqMHz')))
        except TypeError:
            parameters.marker.setValue((parameters.tm.record(0).value('FreqMHz')))
        parameters.tm.removeRow(i)
        parameters.tm.submit()
        app.processEvents()
    parameters.tm.select()
    parameters.showCurve()
    activeButtons(True)


def addFreq():
    parameters.insertData(ui.assetID.value(), 0, 0)
    parameters.dwm.toLast()


def deleteDevice():  # delete all the device parameters first, then delete the device
    ui.tabWidget.setEnabled(False)
    deleteAllFreq()
    attenuators.deleteRow()
    sumLosses()
    ui.tabWidget.setEnabled(True)


def addDevice():
    attenuators.insertData('', '', '')
    attenuators.dwm.toLast()  # devices are autonumbered so new one is the highest number


def deleteCal():
    calibration.deleteRow()
    calibration.showCurve()
    calibration.marker.setValue(ui.calFreq.value())


def addCal():
    calibration.insertData('', 0, '')
    ui.highRef.setValue(-10)  # -10dBm is optimum high value for best dynamic range
    ui.lowRef.setValue(-50)   # -50dBm is optimum low value for best dynamic range
    ui.highCode.setValue(0)
    ui.lowCode.setValue(0)
    ui.slope.setValue(0)
    ui.intercept.setValue(0)
    calibration.dwm.toFirst()  # Cal values are set with freq=0 so new one is lowest


def updateCal(uiCode):  # change meter.dIn
    try:
        openSPI()
    except FileNotFoundError:
        popUp('No SPI device found', 'OK')
    else:
        dIn = spi.xfer([dOut, dOut])  # dOut = Pi to AD7887, MOSI. dIn = measurement result, MISO.
        if dIn[0] > 13:
            spiError('SPI error')  # anything > 13 is due to noise or spi errors
            return
        dIn = (dIn[0] << 8) + dIn[1]  # shift first byte to be MSB of a 12-bit word and add second byte
        uiCode.setValue(dIn)
        spi.close()


def measHCode():
    updateCal(ui.highCode)


def measLCode():
    updateCal(ui.lowCode)


def calibrate():
    # Formula from AD8318 data sheet.  AD8318 transfer function slope is negative, higher RF power = lower ADC code
    if ui.highCode.value() != 0 and ui.lowCode.value() != 0:
        slope = (ui.highCode.value()-ui.lowCode.value())/(ui.highRef.value()-ui.lowRef.value())
        ui.slope.setValue(slope)
        intercept = ui.highRef.value()-(ui.highCode.value()/slope)
        ui.intercept.setValue(intercept)
    else:
        popUp('One or both of the ADC Codes are zero', 'OK')


def openSPI():
    # set up spi bus
    spi.open(0, 0)  # bus 0, device 0.  MISO = GPIO9, MOSI = GPIO10, SCLK = GPIO11
    spi.no_cs = False
    spi.mode = 3  # set clock polarity and phase to 0b11

    # spi max_speed_hz must be integer val accepted by driver : Pi max=31.2e6 but AD7887 max=125ks/s or 2MHz fSCLK
    # valid values are 7.629e3, 15.2e3, 30.5e3, 61e3, 122e3, 244e3, 488e3, 976e3, 1.953e6, [3.9, 7.8, 15.6, 31.2e6]
    spi.max_speed_hz = 1953000


def stopMeter():

    meter.running = False
    while meter.fifo.qsize() > 0:
        time.sleep(0.2)  # allow time for the fifo queue to empty
    ui.meterWidget.update_value(0, mouse_controlled=False)
    ui.powerWatts.setValue(0)
    ui.measurementRate.setValue(0)
    ui.sensorPower.setValue(-70)
    ui.inputPower.setValue(-70)
    ui.spiNoise.setText('')
    activeButtons(True)
    spi.close()


def slidersMoved():
    if meter.running:
        stopMeter()


def freqChanged():
    sumLosses()
    selectCal()
    # deselect band radio buttons
    if ui.hamBands.checkedId() != -11:
        ui.GHzSlider.setEnabled(False)
        ui.freqBox.setEnabled(False)


def bandSelect():
    buttonID = ui.hamBands.checkedId()
    if buttonID == -11:
        ui.GHzSlider.setEnabled(True)
        ui.freqBox.setEnabled(True)
        return
    buttonID = (-buttonID)-2  # exclusive group button index starts at -2 and decreases. Convert to list index.
    ui.GHzSlider.setValue(fBand[buttonID])
    ui.freqBox.setValue(fBand[buttonID])


def rangeSelect():
    if ui.setRangeButton.isChecked():
        ui.rangeSlider.setEnabled(True)
    else:
        ui.rangeSlider.setEnabled(False)


def nextDevice():
    attenuators.dwm.toNext()
    parameters.tm.setFilter('AssetID =' + str(ui.assetID.value()))  # filter parameters on selected device
    parameters.dwm.toFirst()
    parameters.showCurve()
    parameters.marker.setValue(ui.freqIndex.value())


def prevDevice():
    attenuators.dwm.toPrevious()
    parameters.tm.setFilter('AssetID =' + str(ui.assetID.value()))  # filter parameters on selected device
    parameters.dwm.toFirst()
    parameters.showCurve()
    parameters.marker.setValue(ui.freqIndex.value())


def nextParam():
    parameters.dwm.toNext()
    parameters.marker.setValue(ui.freqIndex.value())


def prevParam():
    parameters.dwm.toPrevious()
    parameters.marker.setValue(ui.freqIndex.value())


def nextCal():
    calibration.dwm.toNext()
    calibration.marker.setValue(ui.calFreq.value())


def prevCal():
    calibration.dwm.toPrevious()
    calibration.marker.setValue(ui.calFreq.value())


def activeButtons(tF):
    # prevent User button presses that may affect readings when meter is running
    ui.saveDevice.setEnabled(tF)
    ui.loadS2P.setEnabled(tF)
    ui.saveValues.setEnabled(tF)
    ui.measHigh.setEnabled(tF)
    ui.measLow.setEnabled(tF)
    ui.addDevice.setEnabled(tF)
    ui.deleteDevice.setEnabled(tF)
    ui.addFreq.setEnabled(tF)
    ui.delFreq.setEnabled(tF)
    ui.delAllFreq.setEnabled(tF)
    ui.calibrate.setEnabled(tF)
    ui.saveCal.setEnabled(tF)
    ui.addCal.setEnabled(tF)
    ui.deleteCal.setEnabled(tF)
    ui.inUse.setEnabled(tF)


###############################################################################
# Instantiate classes

config = database()
config.connect()
meter = Measurement()
app = QtWidgets.QApplication([])  # create QApplication for the GUI
window = QtWidgets.QMainWindow()
ui = QtPowerMeter.Ui_MainWindow()
ui.setupUi(window)
attenuators = modelView('Device', ui.deviceGraph)
calibration = modelView('Calibration', ui.slopeFreq)
parameters = modelView('deviceParameters', ui.deviceGraph)
attenuators.marker.setPen('y')
attenuators.marker.setAngle(0)

###############################################################################
# GUI settings

# adjust analog gauge meter
ui.meterWidget.set_MaxValue(10)
ui.meterWidget.set_enable_CenterPoint(enable=False)
ui.meterWidget.set_enable_barGraph(enable=True)
ui.meterWidget.set_enable_value_text(enable=False)
ui.meterWidget.set_enable_filled_Polygon(enable=True)
ui.meterWidget.set_start_scale_angle(90)
ui.meterWidget.set_total_scale_angle_size(270)

# pyqtgraph settings for power vs time display
red = pyqtgraph.mkPen(color='r', width=1.0)
blue = pyqtgraph.mkPen(color='c', width=0.5, style=QtCore.Qt.DashLine)
yellow = pyqtgraph.mkPen(color='y', width=1.0)
ui.graphWidget.setYRange(-60, 10)
ui.graphWidget.setBackground('k')  # black
ui.graphWidget.showGrid(x=True, y=True)
ui.graphWidget.addLine(y=0, movable=False, pen=red, label='max', labelOpts={'position':0.05, 'color':('r')})
ui.graphWidget.addLine(y=-10, movable=False, pen=blue, label='', labelOpts={'position':0.025, 'color':('c')})
ui.graphWidget.addLine(y=-50, movable=False, pen=blue, label='', labelOpts={'position':0.025, 'color':('c')})
ui.graphWidget.setLabel('left', 'Sensor Power', 'dBm')
ui.graphWidget.setLabel('bottom', 'Power Measurement', 'Samples')
powerCurve = ui.graphWidget.plot([], [], name='Sensor', pen=yellow, width=1)

# pyqtgraph settings for device parameters display
ui.deviceGraph.showGrid(x=True, y=True)
ui.deviceGraph.setBackground('k')  # white
ui.deviceGraph.setLabel('left', 'Gain', 'dB')
ui.deviceGraph.setLabel('bottom', 'Frequency', '')

# pyqtgraph settings for calibration display
ui.slopeFreq.addLegend(offset=(20, 10))
ui.slopeFreq.setXRange(0, 3200, padding=0)
ui.slopeFreq.showGrid(x=True, y=True)
ui.slopeFreq.setBackground('k')  # white
ui.slopeFreq.setLabel('bottom', 'Frequency', '')
ui.slopeFreq.setLabel('left', 'Transfer Fn Slope (Codes/dB)', '')
maxLim = ui.slopeFreq.plot(fSpec, maxSlope, name='AD8318 spec limits', pen='y')
minLim = ui.slopeFreq.plot(fSpec, minSlope, pen='y')

###############################################################################
# Connect signals from buttons and sliders

# Display Tab
ui.runButton.clicked.connect(meter.startMeasurement)
ui.stopButton.clicked.connect(stopMeter)
ui.freqBox.valueChanged.connect(freqChanged)
ui.memorySize.valueChanged.connect(slidersMoved)
ui.averaging.valueChanged.connect(slidersMoved)
ui.hamBands.buttonClicked.connect(bandSelect)
ui.rangeSlider.valueChanged.connect(meter.userRange)
ui.autoRangeButton.clicked.connect(rangeSelect)
ui.setRangeButton.clicked.connect(rangeSelect)
parameters.marker.sigPositionChanged.connect(parameters.updateSpinBox)
calibration.marker.sigPositionChanged.connect(calibration.updateSpinBox)

# Devices Tab
ui.nextDevice.clicked.connect(nextDevice)
ui.previousDevice.clicked.connect(prevDevice)
ui.saveDevice.clicked.connect(attenuators.saveChanges)
ui.addDevice.clicked.connect(addDevice)
ui.deleteDevice.clicked.connect(deleteDevice)
ui.nextFreq.clicked.connect(nextParam)
ui.previousFreq.clicked.connect(prevParam)
ui.addFreq.clicked.connect(addFreq)
ui.delFreq.clicked.connect(deleteFreq)
ui.delAllFreq.clicked.connect(deleteAllFreq)
ui.saveValues.clicked.connect(parameters.saveChanges)
ui.loadS2P.clicked.connect(importS2P)

# Calibration Tab
ui.addCal.clicked.connect(addCal)
ui.deleteCal.clicked.connect(deleteCal)
ui.prevCal.clicked.connect(prevCal)
ui.nextCal.clicked.connect(nextCal)
ui.saveCal.clicked.connect(calibration.saveChanges)
ui.measHigh.clicked.connect(measHCode)
ui.measLow.clicked.connect(measLCode)
ui.calibrate.clicked.connect(calibrate)

###############################################################################
# set up the application

attenuators.createTableModel()  # attenuator/coupler etc devices
attenuators.dwm.addMapping(ui.assetID, 0)
attenuators.dwm.addMapping(ui.description, 1)
attenuators.dwm.addMapping(ui.partNum, 2)
attenuators.dwm.addMapping(ui.identifier, 3)
attenuators.dwm.addMapping(ui.maxPower, 4)
attenuators.dwm.addMapping(ui.nominaldB, 5)
attenuators.dwm.addMapping(ui.inUse, 8)

parameters.createTableModel()  # loss vs frequency for each device
parameters.dwm.addMapping(ui.freqIndex, 1)
parameters.dwm.addMapping(ui.loss, 2)
parameters.dwm.addMapping(ui.directivity, 3)

calibration.createTableModel()
calibration.dwm.addMapping(ui.calFreq, 0)
calibration.dwm.addMapping(ui.highRef, 1)
calibration.dwm.addMapping(ui.lowRef, 2)
calibration.dwm.addMapping(ui.highCode, 3)
calibration.dwm.addMapping(ui.lowCode, 4)
calibration.dwm.addMapping(ui.slope, 5)
calibration.dwm.addMapping(ui.intercept, 6)
calibration.dwm.addMapping(ui.refMeter, 7)

sumLosses()
selectCal()
calibration.showCurve()

window.show()

###############################################################################
# run the application until the user closes it

try:
    app.exec()
finally:
    exit_handler()  # close database
