#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jan 19 10:05:11 2025

@author: ian
"""

class clickConnect():
    # Connect signals from buttons and sliders.  Called by 'initialise'.

    ui.scan_button.clicked.connect(tinySA.scan)
    ui.run3D.clicked.connect(tinySA.scan)
    ui.atten_box.valueChanged.connect(attenuate_changed)
    ui.atten_auto.clicked.connect(attenuate_changed)
    ui.spur_box.clicked.connect(tinySA.spur)
    ui.lna_box.clicked.connect(tinySA.lna)
    ui.memBox.valueChanged.connect(memChanged)
    ui.points_auto.stateChanged.connect(pointsChanged)
    ui.points_box.editingFinished.connect(pointsChanged)
    ui.setRange.clicked.connect(tinySA.mouseScaled)
    ui.band_box.currentIndexChanged.connect(band_changed)
    ui.band_box.activated.connect(band_changed)
    ui.rbw_box.currentIndexChanged.connect(rbwChanged)
    ui.rbw_auto.clicked.connect(rbwChanged)

    # frequencies
    ui.start_freq.editingFinished.connect(tinySA.freq_changed)
    ui.stop_freq.editingFinished.connect(tinySA.freq_changed)
    ui.centre_freq.valueChanged.connect(lambda: tinySA.freq_changed(True))  # centre/span mode
    ui.span_freq.valueChanged.connect(lambda: tinySA.freq_changed(True))  # centre/span mode

    # marker dragging
    M1.line.sigPositionChanged.connect(M1.setDelta)
    M2.line.sigPositionChanged.connect(M2.setDelta)
    M3.line.sigPositionChanged.connect(M3.setDelta)
    M4.line.sigPositionChanged.connect(M4.setDelta)
    M1.deltaline.sigPositionChanged.connect(M1.deltaMoved)
    M2.deltaline.sigPositionChanged.connect(M2.deltaMoved)
    M3.deltaline.sigPositionChanged.connect(M3.deltaMoved)
    M4.deltaline.sigPositionChanged.connect(M4.deltaMoved)

    # marker setting within span range
    ui.mkr_start.clicked.connect(markerToStart)
    ui.mkr_centre.clicked.connect(markerToCentre)

    # marker tracking level
    ui.m1track.valueChanged.connect(lambda: M1.setLevel(ui.m1track.value()))
    ui.m2track.valueChanged.connect(lambda: M2.setLevel(ui.m2track.value()))
    ui.m3track.valueChanged.connect(lambda: M3.setLevel(ui.m3track.value()))
    ui.m4track.valueChanged.connect(lambda: M4.setLevel(ui.m4track.value()))

    # marker type changes
    ui.m1_type.activated.connect(M1.mType)
    ui.m2_type.activated.connect(M2.mType)
    ui.m3_type.activated.connect(M3.mType)
    ui.m4_type.activated.connect(M4.mType)

    # frequency band markers
    ui.presetMarker.clicked.connect(freqMarkers)
    ui.presetLabel.clicked.connect(freqMarkerLabel)
    ui.mToBand.clicked.connect(addBandPressed)
    ui.filterBox.currentTextChanged.connect(freqMarkers)

    # trace checkboxes
    ui.trace1.stateChanged.connect(T1.enable)
    ui.trace2.stateChanged.connect(T2.enable)
    ui.trace3.stateChanged.connect(T3.enable)
    ui.trace4.stateChanged.connect(T4.enable)

    # trace type changes
    ui.t1_type.activated.connect(T1.tType)
    ui.t2_type.activated.connect(T2.tType)
    ui.t3_type.activated.connect(T3.tType)
    ui.t4_type.activated.connect(T4.tType)

    ui.sampleRepeat.valueChanged.connect(tinySA.sampleRep)

    # 3D graph controls
    ui.orbitL.clicked.connect(lambda: tinySA.orbit3D(1, True))
    ui.orbitR.clicked.connect(lambda: tinySA.orbit3D(-1, True))
    ui.orbitU.clicked.connect(lambda: tinySA.orbit3D(-1, False))
    ui.orbitD.clicked.connect(lambda: tinySA.orbit3D(1, False))
    ui.timeF.clicked.connect(lambda: tinySA.axes3D(-1, 'X'))
    ui.timeR.clicked.connect(lambda: tinySA.axes3D(1, 'X'))
    ui.freqR.clicked.connect(lambda: tinySA.axes3D(-1, 'Y'))
    ui.freqL.clicked.connect(lambda: tinySA.axes3D(1, 'Y'))
    ui.signalUp.clicked.connect(lambda: tinySA.axes3D(-1, 'Z'))
    ui.signalDown.clicked.connect(lambda: tinySA.axes3D(1, 'Z'))
    ui.gridF.clicked.connect(lambda: tinySA.grid(1))
    ui.gridR.clicked.connect(lambda: tinySA.grid(-1))
    ui.zoom.sliderMoved.connect(tinySA.zoom3D)
    ui.reset3D.clicked.connect(tinySA.reset3D)
    ui.timeSpectrum.clicked.connect(lambda: ui.stackedWidget.setCurrentWidget(ui.View3D))
    ui.analyser.clicked.connect(lambda: ui.stackedWidget.setCurrentWidget(ui.ViewNormal))

    # preferences
    preferences.neg25Line.stateChanged.connect(lambda: T1.hEnable(preferences.neg25Line))
    preferences.zeroLine.stateChanged.connect(lambda: T2.hEnable(preferences.zeroLine))
    preferences.plus6Line.stateChanged.connect(lambda: T3.hEnable(preferences.plus6Line))
    preferences.addRow.clicked.connect(bands.addRow)
    preferences.deleteRow.clicked.connect(lambda: bands.deleteRow(True))
    preferences.deleteAll.clicked.connect(lambda: bands.deleteRow(False))
    preferences.freqBands.clicked.connect(bands.tableClicked)
    preferences.filterBox.currentTextChanged.connect(lambda: bands.filterType(True, preferences.filterBox.currentText()))
    ui.filterBox.currentTextChanged.connect(lambda: bandselect.filterType(False, ui.filterBox.currentText()))
    ui.actionPreferences.triggered.connect(dialogPrefs)  # open preferences dialogue when its menu is clicked
    ui.actionAbout_QtTinySA.triggered.connect(about)
    pwindow.finished.connect(setPreferences)  # update database checkboxes table on dialogue window close
    preferences.exportButton.pressed.connect(exportData)
    preferences.importButton.pressed.connect(importData)
    preferences.deviceBox.activated.connect(testComPort)

    # filebrowse
    ui.actionBrowse_TinySA.triggered.connect(tinySA.dialogBrowse)
    filebrowse.download.clicked.connect(tinySA.fileDownload)
    filebrowse.listWidget.itemClicked.connect(tinySA.fileShow)

    # Trace and Marker settings
    ui.m1trace.valueChanged.connect(lambda: M1.traceLink(ui.m1trace.value()))
    ui.m2trace.valueChanged.connect(lambda: M2.traceLink(ui.m2trace.value()))
    ui.m3trace.valueChanged.connect(lambda: M3.traceLink(ui.m3trace.value()))
    ui.m4trace.valueChanged.connect(lambda: M4.traceLink(ui.m4trace.value()))

    # Quit
    ui.actionQuit.triggered.connect(app.closeAllWindows)

    # Sweep time
    # ui.sweepTime.valueChanged.connect(lambda: tinySA.sweepTime(ui.sweepTime.value()))

    # Waterfall
    ui.waterfallSize.valueChanged.connect(setWaterfall)