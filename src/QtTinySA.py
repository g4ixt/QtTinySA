#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2026 Ian Jefferson G4IXT
# SPDX-License-Identifier: GPL-3.0-or-later

# nuitka-project: --enable-plugin=pyside6
# nuitka-project: --include-qt-plugins=sqldrivers,designer
# nuitka-project: --include-data-file=QtTSAprefs.db=./
# nuitka-project: --include-data-files=./modules/*baseline.txt=modules/
# nuitka-project: --include-data-files=./modules/*.ui=modules/
# nuitka-project: --nofollow-import-to=tkinter,pandas,setuptools,tk,wheel,zipp,pyyaml
# nuitka-project: --nofollow-import-to=packaging,altgraph,mkl,fortran,matlab
# nuitka-project: --mode=standalone
# nuitka-project: --remove-output

"""TinySA GUI programme using Qt, PySide6 and PyQtGraph.

This code provides some of the TinySA Ultra on-screen commands and PC control.
Development is now on Kubuntu 26.04LTS with Python 3.14 and PySide6 using Spyder.
TinySA, TinySA Ultra and the tinysa icon are trademarks of Erik Kaashoek and are used with permission.
TinySA commands are based on Erik's Python examples: http://athome.kaashoek.com/tinySA/python/
Serial communication commands are based on Martin's Python NanoVNA/TinySA Toolset: https://github.com/Ho-Ro"""

import os
import time
import logging

from platform import system
from PySide6 import QtCore
from PySide6 import QtWidgets
from PySide6.QtUiTools import QUiLoader
from PySide6.QtCore import QFile, Slot, QSignalBlocker
from PySide6.QtWidgets import QMessageBox, QDataWidgetMapper, QFileDialog, QApplication
from PySide6.QtWidgets import QTableWidgetItem, QInputDialog, QLineEdit
from PySide6.QtWidgets import QWidget, QSpinBox, QDoubleSpinBox, QComboBox, QCheckBox, QRadioButton
from PySide6.QtSql import QSqlDatabase, QSqlRelation, QSqlRelationalTableModel, QSqlRelationalDelegate, QSqlQuery
from PySide6.QtGui import QPixmap, QIcon

import shutil
import platformdirs
import csv
import numpy as np
import pyqtgraph

from io import BytesIO

from modules.exporters import WWBExporter, WSMExporter
from modules.graphs import SurfaceGraph, PhaseNoiseGraph, SpectrumGraph, PolarGraph
from modules.devices import USBdevice, Worker, WorkerSignals
from modules.utility import resource_path

# Defaults to non local configuration/data dirs - needed for packaging
if system() == "Linux":
    os.environ['XDG_CONFIG_DIRS'] = '/etc:/usr/local/etc'
    os.environ['XDG_DATA_DIRS'] = '/usr/share:/usr/local/share'

# force Qt to use OpenGL rather than DirectX for Windows OS
# QtCore.QCoreApplication.setAttribute(QtCore.Qt.ApplicationAttribute.AA_UseDesktopOpenGL)

logging.basicConfig(format="%(message)s", level=logging.INFO)
threadpool = QtCore.QThreadPool()
basedir = os.path.dirname(__file__)

# classes ##############################################################################

class ApplicationController(QtCore.QObject):
    """Owns the lifetime and coordination of the application.  GUI setup, widget updates, 
       dialogs, timers, and UI-specific callbacks live here.

        ApplicationController
        │
        ├── ui
        ├── dialogs
        ├── hardware
        ├── data
        │   └── correction
        │
        └── analyser
            ├── correction_devices
            ├── read_tables()
            └── upload_correction()
     """

    def __init__(self, app):
        super().__init__()

        self.app = app
        self._DB_VERSION = 211

        self._create_ui()
        self._create_dialogs()
        self._create_device()
        self._set_database()
        self._create_data()
        self._setup_ps_freq_table()
        self._setup_trace_colours_table()
        self._setup_correction_table()
        self._bind_comboboxes()
        self._set_controls()
        self._create_analyser()
        self._set_timers()
        self._setup_gui()

    @property
    def app_name(self):
        return self.app.applicationName()

    def _create_ui(self):
        loader = CustomLoader()
        self.ui = loader.load(
            resource_path("spectrum.ui"), None)
        self.ui.actionQuit_2.triggered.connect(self.shutdown)

    def _create_dialogs(self):
        self.dialogs = Dialogs()

    def _create_device(self):
        self.hardware = USBdevice()

    def _create_analyser(self):
        self.analyser = Analyser(ui=self.ui, hardware=self.hardware, dialogs=self.dialogs, data=self.data, parent=self)
        
    def _create_data(self):
        self.data = Data(self.config)
        
    def _setup_ps_freq_table(self):
        table = self.dialogs.preset_freqs.ui.freqTable
        presets = QSqlRelationalDelegate(table)
        table.setItemDelegate(presets)
        colHeader = table.horizontalHeader()
        colHeader.setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.ResizeToContents)
        # do we need table.select() here?
        
        # connect the preset types window table widget to the data model
        self.dialogs.preset_freqs.ui.typeTable.setModel(self.data.bandstype.tm)
        
        # connect the preset frequencies window table widget to the data model
        self.dialogs.preset_freqs.ui.freqTable.setModel(self.data.bands.tm)
        self.dialogs.preset_freqs.ui.freqTable.hideColumn(0)  # ID
        self.dialogs.preset_freqs.ui.typeTable.hideColumn(0)  # hide primary key so user can't change it

        # populate the preset bands and markers dialogue and ui filter combo boxes
        self.ui.filterBox.setModel(self.data.bandstype.tm)
        self.ui.filterBox.setModelColumn(1)

    def _setup_trace_colours_table(self):
        tracesettings = QSqlRelationalDelegate(self.dialogs.settings.ui.colourTable)
        self.dialogs.settings.ui.colourTable.setItemDelegate(tracesettings)
        traceHeader = self.dialogs.settings.ui.colourTable.horizontalHeader()
        traceHeader.setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.ResizeToContents)
        self.dialogs.settings.ui.colourTable.setModel(self.data.tracecolours.tm)
        self.dialogs.settings.ui.colourTable.hideColumn(0)  # ID
        self.dialogs.settings.ui.colourTable.verticalHeader().setVisible(True)
        
    def _bind_comboboxes(self):
        # the spur combobox is not bound to a model so is populated directly in the controller
    
        # populate the rbw combobox
        self.ui.rbw_box.setModel(self.data.rbwtext.tm)
    
        # populate the trace comboboxes
        self.ui.t1_type.setModel(self.data.tracetext.tm)
        self.ui.t2_type.setModel(self.data.tracetext.tm)
        self.ui.t3_type.setModel(self.data.tracetext.tm)
        self.ui.t4_type.setModel(self.data.tracetext.tm)
    
        # populate the marker comboboxes
        self.ui.m1_type.setModel(self.data.markertext.tm)
        self.ui.m2_type.setModel(self.data.markertext.tm)
        self.ui.m3_type.setModel(self.data.markertext.tm)
        self.ui.m4_type.setModel(self.data.markertext.tm)
    
        # populate the correction text combobox
        self.dialogs.offset.ui.correction_mode.setModel(self.data.correctiontext.tm)
        
        # populate the band select combobox
        self.ui.band_box.setModel(self.data.bandselect.tm)
        self.ui.band_box.setModelColumn(1)
        
    def _setup_correction_table(self):
        # setup correction table
        c_header = self.dialogs.offset.ui.c_table.horizontalHeader()
        c_header.setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.ResizeToContents)
        self.dialogs.offset.ui.c_table.setModel(self.data.correction.tm)
        self.dialogs.offset.ui.c_table.hideColumn(0)

    def _set_controls(self):
        # populate the spur combo box and map the other combo boxes to the database fields
        self.ui.spur_box.addItems(['off', 'on', 'auto'])
        self.ui.spur_box.setCurrentIndex(2)
        
        self.map_table_to_widgets(self.data.checkboxes,'checkboxes')
        self.map_table_to_widgets(self.data.numbers, 'numbers')
        
        # Populate the mapped widgets only after all mappings exist.
        self.data.checkboxes.dwm.setCurrentIndex(0)
        self.data.numbers.dwm.setCurrentIndex(0)
    
    def _set_database(self):
        self.database = Database(db_file="QtTSAprefs.db", connection_name="settings",
                                 target_version=self._DB_VERSION, app_name=self.app_name,
                                 parent_window=self.ui)
        
        self.config = self.database.open_db()
        
        if self.database.upgraded:
            backup_path = os.path.join(platformdirs.user_config_dir(appname=self.app_name, appauthor=False),
                                                           f"frequencies_{self._DB_VERSION}.csv")

            answer = popUp(self.ui, "Restore your previous preset frequencies to the new database?", "OkC", "Question")

            if answer == QMessageBox.StandardButton.Ok:
                impex = Data._create_bands_model(self.config)
                try:
                    impex.deleteRow(False)
                    inserted, failed = self.import_preset(impex, backup_path)
                    logging.info(f"Preset restore complete: {inserted} inserted, {failed} failed.")
                finally:
                    del impex

    def _set_timers(self):
        self.hardware.probe_ports.start(500)
        self.auto_run_timer = QtCore.QTimer()
        self.auto_run_timer.timeout.connect(self._run_on_start)
        if self.dialogs.settings.ui.auto_run.isChecked():
            self.auto_run_timer.start(4000)

    def _set_scan_button(self, action):
        self.ui.scan_button.setText(action)
        self.ui.scan_button.setEnabled(True)

    def map_table_to_widgets(self, model, model_name):
        """Uses the database 'mapping' table, which contains GUI field names and the column 
            number of the table ('numbers' or 'checkboxes') their state/value is stored in."""
        mapping = self.data.mapping
        
        # filter the mapping definition table to show just values for this model_name
        mapping.tm.setFilter(f'model = "{model_name}"')
    
        # setFilter() must be followed by select() to apply the filter.
        if not mapping.tm.select():
            logging.error(f'Failed to select mappings for {model_name}: 'f'{mapping.tm.lastError().text()}')
            return
    
        namespace = {'ui': self.ui, 'dialogs': self.dialogs}
    
        for index in range(mapping.tm.rowCount()):
            record = mapping.tm.record(index)
            gui = record.value('gui')
            column = record.value('column')
            widget = eval(gui, {}, namespace)
            model.dwm.addMapping(widget, int(column))

    def _setup_gui(self):
        '''Configure the various GUIs'''

        # pyqtgraph settings for spectrum display
        self.ui.graphWidget.setYRange(-112, -20)
        self.ui.graphWidget.setDefaultPadding(padding=0.015) # was 0.005
        self.ui.graphWidget.showGrid(x=True, y=True)
        self.ui.graphWidget.setLabel('bottom', '', units='Hz')

        # # pyqtgraph settings for waterfall and histogram display
        # self.ui.waterfall.setDefaultPadding(padding=0.005)
        self.ui.waterfall.setDefaultPadding(padding=0.016)
        self.ui.waterfall.getPlotItem().hideAxis('bottom')
        self.ui.waterfall.setLabel('left', '.', **{'color': '#FFF', 'font-size': '2pt'})
        # self.ui.waterfall.getPlotItem().hideAxis('left')
        self.ui.waterfall.invertY(True)

        self.ui.histogram.setDefaultPadding(padding=0)
        self.ui.histogram.plotItem.invertY(True)
        self.ui.histogram.getPlotItem().hideAxis('bottom')
        self.ui.histogram.getPlotItem().hideAxis('left')

        # Phase Noise widget
        self.dialogs.phasenoise.ui.plotWidget.setYRange(-120, -40)
        self.dialogs.phasenoise.ui.plotWidget.plotItem.showGrid(x=True, y=True, alpha=0.5)
        self.dialogs.phasenoise.ui.plotWidget.plotItem.setLogMode(x=True)
        self.dialogs.phasenoise.ui.plotWidget.setLabel('bottom', 'Offset Frequency', units='Hz')
        self.dialogs.phasenoise.ui.plotWidget.setLabel('left', 'Phase Noise', units='dBc/Hz')
        
        # Pattern widget
        self.dialogs.pattern.ui.measure.clicked.connect(self.analyser.start_polar_plot)
        
        # Band and frequency changes signals
        self.ui.filterBox.currentTextChanged.connect(self.set_band_filter)
        self.dialogs.preset_freqs.ui.deletePsType.clicked.connect(self.delete_preset_type)
        self.dialogs.preset_freqs.ui.freqTable.clicked.connect(self.preset_frequency_clicked)
        self.dialogs.preset_freqs.ui.typeTable.clicked.connect(self.preset_type_clicked)
        self.dialogs.preset_freqs.ui.clearFilter.clicked.connect(self.clear_preset_frequency_filter)
        
        # Correction table signals
        self.dialogs.offset.ui.read_button.clicked.connect(self.analyser.read_tables)
        self.dialogs.offset.ui.upload_button.clicked.connect(self.analyser.upload_correction)
        
        # Waterfall signals
        self.ui.waterfall_size.valueChanged.connect(self.set_wf_height)
        self.ui.wf_2D.stateChanged.connect(self.set_wf_format)

        # hardware signals
        self.hardware.signals.progress.connect(self.time_path_indicator)
        self.hardware.update_info.connect(self.set_device_info)

        # menu actions
        self.ui.actionPresets.triggered.connect(self.preferences_menu_clicked)
        self.ui.actionSettings.triggered.connect(self.settings_clicked)
        self.ui.actionAbout_QtTinySA.triggered.connect(self.about)
        self.ui.actionAbout_Qt.triggered.connect(self.app.aboutQt)
        self.ui.actionSoapySDR.triggered.connect(self.soapySDR_menu_clicked)

        # fixed markers
        self.ui.addFix.clicked.connect(self.add_fixed)

        # settings
        self.dialogs.settings.ui.set_folder.clicked.connect(self.set_folder)
        if self.dialogs.settings.ui.bold_text.isChecked():
            self.app.setStyleSheet("QWidget { font-weight: bold; }")  # enhancement issue 118
        
        # SD card saving
        self.dialogs.file_browse.ui.download.clicked.connect(lambda: self.set_sd_file_save(True))
        self.dialogs.file_browse.ui.saveAll.clicked.connect(lambda: self.set_sd_file_save(False))

        # finally
        self.set_band_filter(self.ui.filterBox.currentText())

    def shutdown(self):
        '''Save gui field vals, marker freqs and checkbox states. Close usb ports, config database and windows'''
        self.hardware.probe_ports.stop()
        self.analyser.mkr_update_timer.stop()
        if self.hardware.is_scanning:
            self.hardware.stop(restart=False)
        if len(self.hardware.ports) != 0:
            self.analyser.marker_save()
            self.hardware.closePort()
        self.data.checkboxes.dwm.submit()
        self.data.numbers.dwm.submit()
        # disconnect(self.config)
        self.database.close()
        self.app.closeAllWindows()
        logging.info('QtTinySA Closed')

    def _run_on_start(self):
        self.auto_run_timer.stop()
        self.ui.scan_button.clicked.emit()

    def delete_preset_type(self):
        bandstype = self.data.bandstype
        bands = self.data.bands
        dialog = self.dialogs.preset_freqs
    
        record = bandstype.tm.record(bandstype.currentRow)
    
        if record.value('ID') == bandstype.ID:
            popUp(dialog, "Cannot delete a preset type that is selected on main screen",
                          'Ok',
                          'Critical')
            return
    
        # bands.set_filter_to(True, record.value('preset'))
        bands.set_filter('preset = "' + record.value('preset') + '"')
        bands.deleteRow(False)
    
        if bands.tm.rowCount() == 0:
            # now no frequency records with the preset type,
            # so delete the preset type and preserve database referential integrity
            bandstype.deleteRow(True)

    def preset_type_clicked(self):
        bandstype = self.data.bandstype
        bands = self.data.bands
        table = self.dialogs.preset_freqs.ui.typeTable
        bandstype.set_current_row(table.currentIndex().row())
        record = bandstype.tm.record(bandstype.currentRow)
        bands.set_filter('preset = "' + record.value('preset') + '"')
        bands.unlimited()
        self.dialogs.preset_freqs.ui.psCount.setValue(bands.tm.rowCount())
        
    def preset_frequency_clicked(self):
        table = self.dialogs.preset_freqs.ui.freqTable
        self.data.bands.set_current_row(table.currentIndex().row())

    def clear_preset_frequency_filter(self):
        bands = self.data.bands
        dialog = self.dialogs.preset_freqs
        dialog.ui.typeTable.clearSelection()
        bands.clear_filter()
        dialog.ui.psCount.setValue(bands.tm.rowCount())

    def set_band_filter(self, boxText):
        bandselect = self.data.bandselect
        bandstype = self.data.bandstype
    
        sql = 'visible = "1" AND preset = "' + boxText + '"'
        bandselect.set_filter(sql)
    
        # find and store the ID and LO of the selected preset type
        bandstype.unlimited()
        for index in range(bandstype.tm.rowCount()):
            record = bandstype.tm.record(index)
            if record.value('preset') == boxText:
                bandstype.ID = record.value('ID')
                bandstype.freq = record.value('LO')
                break
    
        self.analyser.set_mixer_highlighting()  # do we need this here?
       
    def import_data(self, model):
        file_name = QFileDialog.getOpenFileName(caption="Open File", filter="Comma Separated Values (*.csv)")[0]
    
        logging.info(f'importing data from {file_name}')
    
        if file_name != '':
            self.import_preset(model, file_name)

    def import_preset(self, model, file_name):
        records_to_import = []
        skipped = 0
        with open(file_name, "r") as file_input:
            reader = csv.DictReader(file_input)
            for row in reader:
                valid_row = True

                # create a blank record matching the model table
                record = model.tm.record()

                #  step through and process the fields in this row.  Db defaults 'visible' to True if missing.
                for key, value in row.items(): 
                    # if key in ('preset', 'colour', 'value'):  # 'value' -> 'visible' True/False
                    if key in ('preset', 'colour', 'visible', 'value'):
                        value = model.relation_id(key, value)
                        if value is None:
                            # the relation lookup failed so the whole row must be discarded
                            valid_row = False
                            skipped += 1
                            logging.warning(f'{key} at line {reader.line_num} was skipped')
                            break

                    # substitute 'frequency' in the header for our 'startF'
                    if key == 'Frequency':
                        key = 'startF'
                        try:
                            value = str(float(value) / 1e3)
                        except ValueError:
                            valid_row = False
                            logging.warning(f'invalid frequency for line {reader.line_num}')
                            break

                    # An ID column value must be discarded because it's the primary key here
                    if key != 'ID':
                        record.setValue(str(key), value)

                if valid_row:
                    records_to_import.append(record)
    
        inserted, failed = model.import_records(records_to_import)

        if inserted != 0:
            logging.info(f'imported {inserted} frequency presets')
        if failed != 0:
            logging.info(f'import failed for {failed} duplicates or unknown type')
        
        return inserted, failed + skipped
    
    def set_wf_format(self):  # called when wf_2D checkbox state changes
        wf_size = self.ui.waterfall_size.value()
        if self.ui.wf_2D.isChecked():
            self.ui.plot_3D.hide()
            self.ui.waterfall.show()

            # set the display_frame stretch, which is 'units' of total vertical space rows can expand in
            self.ui.display_frame.layout().setRowStretch(0, 9 - wf_size)  # index 0 = row 0 = graphWidget
            self.ui.display_frame.layout().setRowStretch(1, 0)  # index 1 = row 1 = plot_3D
            self.ui.display_frame.layout().setRowStretch(2, wf_size)  # index 2 = row 2 = waterfall
        else:
            self.ui.plot_3D.show()
            self.ui.waterfall.hide()
            self.ui.display_frame.layout().setRowStretch(0, 9 - wf_size)
            self.ui.display_frame.layout().setRowStretch(1, wf_size)
            self.ui.display_frame.layout().setRowStretch(2, 0)
    
    def set_wf_height(self):  # called when wf_size spinbox value changes
        '''changing height sends the widget Resize signal; this is intercepted by the ResizeEventFilter
           in the graphs.py module which then calls on_widget_resized() to set the 3D aspect ratio'''
        wf_size = self.ui.waterfall_size.value()
        if wf_size == 9:
            self.ui.graphWidget.hide()
            self.set_wf_format()
        else:
            self.ui.graphWidget.show()
            self.set_wf_format()
        if wf_size == 0:
            self.ui.plot_3D.hide()
            self.ui.waterfall.hide()
            self.ui.wf_2D.setEnabled(False)
        else:
            self.set_wf_format()
            self.ui.wf_2D.setEnabled(True)

    def settings_clicked(self):
        logging.info('settings clicked')
        self.save_location_valid()
        self.dialogs.settings.ui.show()

    def preferences_menu_clicked(self):
        self.dialogs.preset_freqs.ui.show()
        self.dialogs.preset_freqs.ui.psCount.setValue(self.data.bands.tm.rowCount())
        
    def soapySDR_menu_clicked(self):
        self.dialogs.sdr_settings.ui.show()
    
    def about(self):
        message = ('TinySA Ultra GUI programme using Qt6 PySide6\
                   \nAuthor: Ian Jefferson G4IXT\n\nVersion: {} \nConfig: {}'
                   .format(self.app.applicationVersion(), self.config.databaseName()))
        popUp(self.ui, message, 'Ok', 'Info')

    def add_fixed(self):
        title = "New fixed frequency Marker"
        message = "Enter a name for the fixed Marker"
        fixed_mkr, ok = QInputDialog.getText(None, title, message, QLineEdit.Normal, "")
        if not ok or not fixed_mkr:
            return
        
        mkr_colour = self.data.bands.relation_id('colour', 'orange')
        mkr_freq = int(self.analyser.s0.trace.m0.line.value())
        ps_type = bands.relation_id('preset', 'fixed')
        
        self.data.bands.insertData(name=fixed_mkr, preset=ps_type, startF=f'{mkr_freq}',
                                   stopF=0, visible=1, colour=mkr_colour)

    def set_folder(self):
        folder = QFileDialog.getExistingDirectory(self.dialogs.settings.ui, "Select save folder",
                                                  self.dialogs.settings.ui.save_folder.text())
        if not folder:
            return

        self.dialogs.settings.ui.save_folder.setText(folder)
        self.save_location_valid()
        self.data.numbers.dwm.submit()

    @Slot()
    def set_device_info(self, port_num, description, tooltip, state):
        gui_ctrl = {0: self.ui.dev0, 1: self.ui.dev1, 2: self.ui.dev2, 3: self.ui.dev3}
        gui_ctrl.get(port_num).setText(description)
        gui_ctrl.get(port_num).setToolTip(tooltip)
        gui_ctrl.get(port_num).setChecked(state)

    def set_box_colour(self, pen, box):
        boxes = [self.ui.trace1, self.ui.trace2, self.ui.trace3, self.ui.trace4]
        tint = str("background-color: '" + pen + "';")
        boxes[box].setStyleSheet(tint)

    def save_location_valid(self):
        folder = self.dialogs.settings.ui.save_folder.text()
        if os.path.exists(folder):
            self.dialogs.settings.ui.save_folder.setStyleSheet("")
            return True
        else:
            self.dialogs.settings.ui.save_folder.setStyleSheet(("background-color:red"))
            return False

    def set_sd_file_save(self, single=True):
        device = self.file_devices[self.dialogs.file_browse.ui.device.currentIndex()]
        folder = QFileDialog.getExistingDirectory(caption="Select folder to save SD card files")
        if single:
            file_name = self.dialogs.file_browse.ui.listWidget.currentItem().text()  # the file selected in the list widget
        else:
            file_name = None
        saver = Worker(self.sd_file_save, device, file_name, folder, single)
        threadpool.start(saver)  # workers deleted when thread ends

    def sd_file_save(self, device, file_name, folder, single):
        signals = WorkerSignals()
        signals.progress.connect(self.sd_save_progress)
        signals.progress.emit(0)
        SD = device.listSD()
        for i in range(len(SD.splitlines())):
            if not single:
                file_name = SD.splitlines()[i].split(" ")[0]
            if file_name != '.Trash-1000':
                with open(os.path.join(folder, file_name), "wb") as file:
                    data = device.readSD(file_name)
                    file.write(data)
                signals.progress.emit(int(100 * (i+1)/len(SD.splitlines())))
                if single:
                    signals.progress.emit(100)
                    break
        signals.progress.emit(100)

    def sd_save_progress(self, progress):
        self.dialogs.file_browse.ui.saveProgress.setValue(progress)

    def time_path_indicator(self, position):
        with QSignalBlocker(self.ui.vortex):
            self.ui.vortex.setValue(position)

    def ui_fields_to_dict(self):
        """Loops through fields in sdr_settings widget and extracts their current values to a dictionary,
        so that the dictionary can be passed to the hardware when it's started by analyser."""
        form_data = {}

        # Loop through all widgets inside this parent container
        for widget in self.dialogs.sdr_settings.ui.findChildren(QWidget):
            key = widget.objectName()
            # Read value based on the specific type of widget
            if isinstance(widget, QSpinBox):
                form_data[key] = widget.value()      
            elif isinstance(widget, QDoubleSpinBox):
                form_data[key] = widget.value()  
            elif isinstance(widget, QComboBox):
                form_data[key] = widget.currentText()  # or widget.currentIndex()          
            elif isinstance(widget, QCheckBox):
                form_data[key] = widget.isChecked()
            elif isinstance(widget, QRadioButton):
                form_data[key] = widget.isChecked()
        return form_data


class CustomTableModel(QSqlRelationalTableModel):
    def __init__(self, parent=None, db=None, ro_columns=tuple()):
        super().__init__(parent, db)
        self.read_only = ro_columns

    def flags(self, index):
        if index.column() in self.read_only:
            return QtCore.Qt.ItemFlag.ItemIsEnabled | QtCore.Qt.ItemFlag.ItemIsSelectable
        else:
            return super().flags(index)


class CustomLoader(QUiLoader):
    def createWidget(self, className, parent=None, name=""):
        logging.debug(f'className = {className}')
        file_name = resource_path(name)
        if className == "PlotWidget":
            return pyqtgraph.PlotWidget(parent=parent)
        if className == "GraphicsView":
            return pyqtgraph.GraphicsView(parent=parent)
        return super().createWidget(className, parent, file_name)


class CustomDialogue(QtWidgets.QDialog):
    def __init__(self, ui_name):
        super().__init__()
        ui_file = QFile(ui_name)
        ui_file.open(QFile.ReadOnly)
        loader = CustomLoader()
        self.ui = loader.load(ui_file)
        self.ui.setWindowIcon(QIcon(os.path.join(basedir, 'tinySAsmall.png')))


class Dialogs:
    def __init__(self):
        self.settings = CustomDialogue(resource_path("settings.ui"))
        self.phasenoise = CustomDialogue(resource_path("phasenoise.ui"))
        self.pattern = CustomDialogue(resource_path("pattern.ui"))
        self.fading = CustomDialogue(resource_path("fading.ui"))
        self.preset_freqs = CustomDialogue(resource_path("bands.ui"))
        self.file_browse = CustomDialogue(resource_path("filebrowse.ui"))
        self.offset = CustomDialogue(resource_path("offset.ui"))
        self.sdr_settings = CustomDialogue(resource_path("sdr.ui"))


class Database:
    """Locate, open, validate and close the application's SQLite database."""

    def __init__(self, db_file, connection_name, target_version,
                 app_name, parent_window):
        self.db_file = db_file
        self.connection_name = connection_name
        self.target_version = target_version
        self.app_name = app_name
        self.parent_window = parent_window
        self.db = None
        self.upgraded = False

    def locate_db(self):
        personal_dir = platformdirs.user_config_dir(appname=self.app_name, appauthor=False)
        os.makedirs(personal_dir, exist_ok=True)
        personal_path = os.path.join(personal_dir, self.db_file)

        if os.path.isfile(personal_path):
            logging.info(f"Database {self.db_file} found at {personal_dir}")
            return personal_path

        global_dir = platformdirs.site_config_dir(appname=self.app_name, appauthor=False)
        global_path = os.path.join(global_dir, self.db_file)

        if os.path.isfile(global_path):
            shutil.copy2(global_path, personal_path)
            logging.info(f"Database copied from {global_path} to {personal_path}")
            return personal_path

        bundled_path = resource_path(self.db_file)
        if os.path.isfile(bundled_path):
            shutil.copy2(bundled_path, personal_path)
            logging.info(f"Database copied from {bundled_path} to {personal_path}")
            return personal_path

        directories = {os.path.dirname(os.path.abspath(__file__)), os.getcwd()}
        for directory in directories:
            source = os.path.join(directory, self.db_file)
            if os.path.isfile(source):
                shutil.copy2(source, personal_path)
                logging.info(f"Database copied from {source} to {personal_path}")
                return personal_path

        raise FileNotFoundError(f"Unable to find the database {self.db_file}")

    def open_db(self):
        db_path = self.locate_db()
        self.db = QSqlDatabase.addDatabase("QSQLITE", self.connection_name)
        self.db.setDatabaseName(db_path)

        if not self.db.open():
            error = self.db.lastError().text()
            logging.error(f"Cannot open database {db_path}: {error}")
            self.db = None
            QSqlDatabase.removeDatabase(self.connection_name)
            raise RuntimeError(f"Cannot open database: {error}")

        logging.debug(f"{self.db_file} open: {self.db.isOpen()} Connection = {self.db.connectionName()}")

        query = QSqlQuery(self.db)
        if query.exec("PRAGMA foreign_keys = ON;"):
            logging.debug("Database foreign keys enforced")
        else:
            logging.warning("Failed to enable foreign key enforcement: {query.lastError().text()}")
        query.clear()

        self.upgraded = self.check_version()
        return self.db

    def close(self):
        if self.db is None:
            return

        connection_name = self.db.connectionName()
        self.db.close()
        logging.debug(f"Database {self.db.databaseName()} open: {self.db.isOpen()}")

        # Qt requires all models and queries using this connection to have
        # released it before removeDatabase() is called.
        self.db = None
        QSqlDatabase.removeDatabase(connection_name)

    def fetch_version(self):
        query = QSqlQuery(self.db)
        if not query.exec("PRAGMA user_version;") or not query.next():
            logging.error(f"Cannot read database version: {query.lastError().text()}")
            query.finish()
            return None

        version = query.value(0)
        query.finish()
        return version


    def _release_connection(self):
        """Close and remove the connection before replacing its database file."""
        if self.db is None:
            return

        connection_name = self.db.connectionName()
        database_path = self.db.databaseName()

        self.db.close()
        self.db = None
        QSqlDatabase.removeDatabase(connection_name)

        logging.debug(f"Released database connection {connection_name}: {database_path}")

    # updated method
    def check_version(self):
        existing = self.fetch_version()
        logging.info(f"Database version is {existing}, expected {self.target_version}")

        if existing == self.target_version:
            return False

        message = (f"This version of QtTinySA needs database version {self.target_version}.\n\n"
                   f"Database {self.db.databaseName()}\nversion {existing} may not be compatible.\n\n"
                   f"Clicking OK will replace it with version {self.target_version} and reset some settings.")
        
        answer = popUp(self.parent_window, message, "OkC", "Question")
        if answer != QMessageBox.StandardButton.Ok:
            logging.warning("Database upgrade declined; keeping existing database.")
            return

        old_path = self.db.databaseName()
        backup_path = os.path.join(platformdirs.user_config_dir(appname=self.app_name, appauthor=False),
                                   f"frequencies_{self.target_version}.csv")

        # Export all preset frequencies before replacing the database.
        impex = Data._create_bands_model(self.db)
        if not impex.tm.select():
            error = impex.tm.lastError().text()
            del impex
            raise RuntimeError(f"Cannot read preset frequencies: {error}")

        impex.unlimited()
        impex.exportData(backup_path)
        del impex

        # Release the old connection before renaming its database file.
        self._release_connection()

        archived_path = f"{old_path}.{existing}"
        try:
            os.replace(old_path, archived_path)
        except OSError:
            logging.exception("Could not archive the old database.")
            raise

        # locate_db() finds the 'bundled' database and copies it into the personal configuration directory.
        new_path = self.locate_db()
        self.db = QSqlDatabase.addDatabase("QSQLITE", self.connection_name)
        self.db.setDatabaseName(new_path)

        if not self.db.open():
            error = self.db.lastError().text()
            logging.error(f"Cannot open replacement database: {error}")
            self.db = None
            QSqlDatabase.removeDatabase(self.connection_name)
            raise RuntimeError(f"Cannot open replacement database: {error}")

        query = QSqlQuery(self.db)
        if not query.exec("PRAGMA foreign_keys = ON;"):
            logging.warning(f"Failed to enable foreign key enforcement: {query.lastError().text()}")
        query.finish()

        new_version = self.fetch_version()
        if new_version != self.target_version:
            message = (f"The replacement database has version {new_version}, "
                       f"but version {self.target_version} is required.")
            logging.error(message)
            popUp(self.parent_window, message, "Ok", "Critical")
            raise RuntimeError(message)

        logging.info(f"Old Database replaced with version {new_version}: {new_path}")
        
        return True


class Data:
    '''Creates and exposes the application's data models'''

    def __init__(self, config):
        self.config = config

        self.mapping = self._set_mapping()
        self.bands = self._set_bands()
        self.bandstype = self._set_bandstype()
        self.rbw = self._create_combo_box("rbw")
        self.tracetext = self._set_tracetext()
        self.trace = self._create_combo_box("trace")
        self.markertext = self._set_markertext()
        self.marker = self._create_combo_box("marker")
        self.correction = self._set_correction()
        self.correction_type = self._create_combo_box("correction")
        self.colours = self._set_colours()
        self.presetmarker = self._set_presetmarker()
        self.bandselect = self._set_bandselect()
        self.tracecolours = self._set_tracecolours()
        self.checkboxes = self._set_checkboxes()
        self.rbwtext = self._set_rbwtext()
        self.correctiontext = self._set_correctiontext()
        self.numbers = self._set_numbers()
    
    def _set_mapping(self):
        '''field mapping of the checkboxes and numbers database tables, for storing startup configuration'''
        mapping = ModelView('mapping', self.config, ())
        mapping.tm.select()
        return mapping
    
    @staticmethod
    def _create_bands_model(config):
        """Create the relational frequencies model used by the preset frequencies window."""
        bands = ModelView('frequencies', config, ())
        bands.tm.setSort(3, QtCore.Qt.SortOrder.AscendingOrder)
        bands.tm.setHeaderData(5, QtCore.Qt.Orientation.Horizontal, "visible")
        bands.tm.setEditStrategy(QSqlRelationalTableModel.EditStrategy.OnRowChange)
        bands.tm.setRelation(2, QSqlRelation("freqtype", "ID", "preset"))
        bands.tm.setRelation(5, QSqlRelation("boolean", "ID", "value"))
        bands.tm.setRelation(6, QSqlRelation("SVGColour", "ID", "colour"))
        bands.tm.select()
        return bands

    def _set_bands(self):
        return self._create_bands_model(self.config)
    
    def _set_bandstype(self):
        # the preset Types table in the preset frequencies window
        bandstype = ModelView('freqtype', self.config, ())
        bandstype.tm.select()
        return bandstype
        
    def _set_correction(self):
        # the correction values table in the correction window
        correction = ModelView('correction', self.config, (0, 1, 2, 3))
        correction.tm.select()
        return correction
        
    def _set_colours(self):
        # the preset bands and markers colours because can't get the relationships to work
        colours = ModelView('SVGColour', self.config, ())
        colours.tm.select()
        return colours
        
    def _set_presetmarker(self):
        # the main screen preset markers, which need different filtering to the preset frequencies window
        presetmarker = ModelView('frequencies', self.config, ())
        presetmarker.tm.setRelation(6, QSqlRelation('SVGColour', 'ID', 'colour'))
        presetmarker.tm.setRelation(2, QSqlRelation('freqtype', 'ID', 'preset'))
        presetmarker.tm.setSort(3, QtCore.Qt.SortOrder.AscendingOrder)
        presetmarker.tm.select()
        return presetmarker
        
    def _set_bandselect(self):
        # the ui band selection combo box; needs different filter to the main and preset frequencies window
        bandselect = ModelView('frequencies', self.config, ())
        bandselect.tm.setRelation(2, QSqlRelation('freqtype', 'ID', 'preset'))
        bandselect.tm.setRelation(5, QSqlRelation('boolean', 'ID', 'value'))
        bandselect.tm.setRelation(6, QSqlRelation('SVGColour', 'ID', 'colour'))
        bandselect.tm.setSort(3, QtCore.Qt.SortOrder.AscendingOrder)
        bandselect.tm.select()
        return bandselect
        
    def _set_tracecolours(self):
        # connect the settings window trace colours widget to the data model
        tracecolours = ModelView('trace', self.config, (0, 1))
        tracecolours.tm.setRelation(2, QSqlRelation('SVGColour', 'ID', 'colour'))
        tracecolours.tm.setEditStrategy(QSqlRelationalTableModel.EditStrategy.OnFieldChange)
        tracecolours.tm.select()
        return tracecolours
        
    def _set_checkboxes(self):
        # Map data tables to presets/settings/GUI fields - must be here & in this order
        checkboxes = ModelView('checkboxes', self.config, ())
        checkboxes.createMapper()
        checkboxes.tm.select()
        return checkboxes
        
    def _create_combo_box(self, combo_type):
        combo = ModelView('combo', self.config, ())
        combo.tm.setFilter(f'type = "{combo_type}"')
        combo.tm.select()
        return combo

    def _set_rbwtext(self):
        rbwtext = ModelView('combo', self.config, ())
        rbwtext.tm.setFilter('type = "rbw"')
        rbwtext.tm.select()
        return rbwtext
        
    def _set_correctiontext(self):
        correctiontext = ModelView('combo', self.config, ())
        correctiontext.tm.setFilter('type = "correction"')
        correctiontext.tm.select()
        return correctiontext
        
    def _set_numbers(self):
        # Map data tables to presets/settings/GUI fields - must be here & in this order
        numbers = ModelView('numbers', self.config, ())
        numbers.createMapper()
        numbers.tm.select()
        return numbers
    
    def _set_tracetext(self):
        # # the trace comboboxes
        tracetext = ModelView('combo', self.config, ())
        tracetext.tm.setFilter('type = "trace"')
        tracetext.tm.select()
        return tracetext

    def _set_markertext(self):
        # # the marker comboboxes
        markertext = ModelView('combo', self.config, ())
        markertext.tm.setFilter('type = "marker"')
        markertext.tm.select()
        return markertext
        
      
class Analyser(QtCore.QObject):
    '''Spectrum-analyser operations and analysis logic.  Owns operations that perform
       measurements and gui graph updates by coordination of hardware, dialogs and data models'''
       
    def __init__(self, ui, hardware, dialogs, data, parent=None):
        super().__init__(parent)
        
        self.ui = ui
        self.hardware = hardware
        self.dialogs = dialogs
        self.data = data
        self.limits = LimitLines(self.ui.graphWidget,
                                 self.dialogs.settings.ui.peakThreshold.value(),
                                 self.ui.start_freq.value(),
                                 self.ui.stop_freq.value(),
                                 self.ui.span_freq.value())
        
        self.multiplot = pyqtgraph.GraphicsLayout()  # for plotting marker signal level over time
        self.dialogs.fading.ui.grView.setCentralItem(self.multiplot)

        self.maxF = 12000
        self.memF = BytesIO()
        self.mkr_update_timer = QtCore.QTimer(self)
        self.file_devices = []
        self.correction_devices = []
        self.depth = 50
        self.points = 101

        self.wf_data = np.ndarray(2)

    def setGraphs(self):
        self.phaseNoise = PhaseNoiseGraph(self.dialogs.phasenoise.ui.plotWidget, np.ndarray, np.ndarray, 1)
        self.polar = PolarGraph(self.dialogs.pattern.ui, 4, 40)
        self.timespectrum = SurfaceGraph(self.ui.plot_3D, np.ndarray, np.ndarray)
      
        # instantiate each spectrum, which has three elements: 1 trace; 4 markers; 1 monitor
        self.s0 = SpectrumGraph(self.ui.graphWidget, self.ui.waterfall, self.ui.histogram, self.multiplot, 100)
        self.s1 = SpectrumGraph(self.ui.graphWidget, self.ui.waterfall, self.ui.histogram, self.multiplot, 300)
        self.s2 = SpectrumGraph(self.ui.graphWidget, self.ui.waterfall, self.ui.histogram, self.multiplot, 500)
        self.s3 = SpectrumGraph(self.ui.graphWidget, self.ui.waterfall, self.ui.histogram, self.multiplot, 700)
        self.spectra = (self.s0, self.s1, self.s2, self.s3)

    def setSignals(self):
        self.hardware.signals.result.connect(self.router)
        self.hardware.signals.save.connect(self.save_data)
        self.hardware.signals.error.connect(popUp)
        self.hardware.stopped.connect(self.allStopped)
        self.mkr_update_timer.timeout.connect(self.updateMarker)

    @Slot()
    def router(self, freq, levl, maxl, minl, buffer, port_in_use, ser_num, timestamp, split, sweep_end):
        '''Called by a signal from the measurement threads to route updates to
           the spectrum trace(s) & recorder based on the port and device count.
           tuple 1 = (device, number of devices) tuple 2 = trace(s) to update
           If only 1 SA it updates all 4 traces. If 2 SAs: first=1&2, second=3&4; etc'''
           
        routes = {(0, 1): (self.s0, self.s1, self.s2, self.s3),
                  (0, 2): (self.s0, self.s2),
                  (0, 3): (self.s0, None),
                  (0, 4): (self.s0, None),
                  (1, 2): (self.s1, self.s3),
                  (1, 3): (self.s1, None),
                  (1, 4): (self.s1, None),
                  (2, 3): (self.s2, None),
                  (2, 4): (self.s2, None),
                  (3, 4): (self.s3, None)}

        # only route for enabled ports, which may be a subset of connected ports depending on gui checkboxes
        count = self.hardware.num_enabled
        enabled_devices = [device.enabled for device in self.hardware.devices]
        port_name = [port.device for port in self.hardware.ports]
        enabled_ports = [port for enabled, port in zip(enabled_devices, port_name) if enabled]
        try:
            indx = enabled_ports.index(port_in_use)
        except ValueError:
            # may occur when a device is disabled, before its measurement thread re-starts
            indx = 0

        # create the data route, i.e. spectra to update, from the matching key tuple of the dictionary 'routes'
        route = routes.get((indx, count))
        if route is None:
            logging.info(f'failed to route data from {indx} of {count} on {port_in_use} to spectrum trace')
            self.hardware.stop(restart=False)
        else:
            self.updateGUI(route, freq, levl, maxl, minl, buffer, ser_num, indx, timestamp, split, sweep_end)
            if self.hardware.recorders[indx].recording and sweep_end:
                self.hardware.recorders[indx].record(freq, levl, ser_num)       

    def setGUI(self):
        # connect GUI controls that don't interfere with restoration of data at startup
        self.connect_passive()
        self.setStartFreq()

        # set various defaults
        self.set_preferences()
        band = self.ui.band_box.currentText()
        # self.data.bandselect.set_filter_to(False, self.ui.filterBox.currentText())  # setting filter overwrites band

        self.set_mixer_highlighting()
        index = self.ui.band_box.findText(band, QtCore.Qt.MatchExactly)

        # connect GUI controls that would interfere with restoration of data at startup
        ## may need to modify for multi-devices ##
        self.connect_active()
        
        # restore previous exact frequencies or band default frequencies depending on set preferences
        if self.dialogs.settings.ui.restore_band.isChecked():
            self.ui.band_box.setCurrentIndex(-1)  # toggle otherwise index 0 doesn't update
            self.ui.band_box.setCurrentIndex(index)
        else:
            with QSignalBlocker(self.ui.band_box):  # set the band in the combobox, but prevent freq update
                self.ui.band_box.setCurrentIndex(index)
        
        self.ui.waterfall_size.valueChanged.emit(self.ui.waterfall_size.value()) # call the waterfall size setter
        
        self.s0.enable(self.ui.trace1.isChecked())
        self.s1.enable(self.ui.trace2.isChecked())
        self.s2.enable(self.ui.trace3.isChecked())
        self.s3.enable(self.ui.trace4.isChecked())
        
        # set the marker types and restore the frequencies from the config database
        for mkr_num in range(4):
            self.set_main_marker(mkr_num)
        self.marker_restore()
            
        # hide the playback time slider
        self.ui.vortex.hide()

    def setting_change(self):
        if self.hardware.is_scanning:
            self.hardware.stop(True)
        self.stop_recording()
            
    def split_scan(self, startF, stopF, points, split):
        '''splits the spectrum start/stop variables across multiple devices'''
        if not split or self.hardware.num_enabled == 1:
            for spectrum in self.spectra:
                spectrum.startF = startF
                spectrum.stopF = stopF
                spectrum.points = points  # set points per spectrum = future potential for different vals
            return
        points = int(points/self.hardware.num_enabled)
        span = int((stopF - startF)/self.hardware.num_enabled)
        starts = {1: (startF, startF, startF, startF),
                  2: (startF, startF+span, startF, startF+span),
                  3: (startF, startF+span, startF+2*span, 0),  # what happens to the zero?
                  4: (startF, startF+span, startF+2*span, startF+3*span)}
        for indx, spectrum in enumerate(self.spectra):
            # set the start and stop feqs for each trace, which is used by 
            spectrum.startF = starts.get(self.hardware.num_enabled)[indx]
            spectrum.stopF = starts.get(self.hardware.num_enabled)[indx] + span
            spectrum.points = points

    def scan(self):  # called by the scan/stop button
        ''''take settings from GUI boxes and start usbDevice, which interfaces to the hardware'''
        if self.hardware.is_scanning:
            self.hardware.stop(restart=False)
            return
        self.stop_playback()
        
        if self.hardware.devices is None:
            popUp(self.ui, 'No spectrum analyser devices found', 'Ok', 'Critical')
            return
        
        if self.dialogs.settings.ui.saveSweep.isChecked() and not self.parent().save_location_valid():
            popUp(self.ui, "The save file location in 'settings > preferences' is not valid", 'Ok', 'Critical')
            return
        
        self.mkr_update_timer.stop()  # stop it because updateGUI does it when scanning
        # self.hardware.set_sa_info()

        # set sweep and device-specific control values
        self.setPoints()
        startF = self.ui.start_freq.value() * 1e6  # freq in Hz
        stopF = self.ui.stop_freq.value() * 1e6
        split = self.ui.split_scan.isChecked()
        maxF = self.dialogs.settings.ui.maxFreqBox.value() * 1e6
        interval = self.dialogs.settings.ui.intervalBox.value()
        rbw = self.setRBW()
        attn = self.attn()
        lna = self.lna()
        spur = self.spur()
        self.setGraphFreq(startF, stopF)
        if self.hardware.num_enabled == 0:
            popUp(self.ui, 'No devices enabled', 'Ok', 'Critical')
            return
        
        sdr_settings = self.parent().ui_fields_to_dict()
        self.hardware.controls(rbw, attn, lna, spur, sdr_settings)

        # For LNB / transverter mode, modify startF and stopF to suit LO freq
        if self.data.bandstype.freq != 0:
            startF, stopF = self.freqOffset(startF, stopF)

        self.split_scan(startF, stopF, self.points, split)
        self.set_gui_colours()
        self.set_arrays()

        # start device(s) scanning
        self.hardware.start(self.spectra, rbw, self.depth, maxF, interval, split, sdr_settings, loop=True)
        self.parent()._set_scan_button('Stop Scan')

    def set_gui_colours(self):
        # set each spectrum (trace & marker) and box colours
        for indx, spectrum in enumerate(self.spectra):
            spectrum.count = 0
            pen = self.data.tracecolours.tm.record(indx).value('colour')
            spectrum.set_colour(pen)
            self.parent().set_box_colour(pen, indx)

    def set_arrays(self):
        # set the data arrays for the monitor and waterfall
        self.depth = self.ui.memBox.value()
        for indx, spectrum in enumerate(self.spectra):
            time_points = self.dialogs.fading.ui.timePoints.value()
            spectrum.monitor_data = np.full((int(time_points), 2), None, dtype=float)
            self.wf_data = np.full((self.depth, self.points), None, dtype=float)

    def lna(self):
        if self.ui.lna_box.isChecked():
            self.ui.atten_box.setValue(0)
            self.ui.atten_auto.setEnabled(False)  # attenuator and lna are switched so mutually exclusive
            self.ui.atten_auto.setChecked(False)
            self.ui.atten_box.setEnabled(False)
            return True
        else:
            self.ui.atten_auto.setEnabled(True)
            self.ui.atten_auto.setChecked(True)
            return False

    def attn(self):
        if self.ui.lna_box.isChecked():  # attenuator and lna are mutually exclusive
            return "0"
        attenuation = self.ui.atten_box.value()
        if self.ui.atten_auto.isChecked():
            self.ui.atten_box.setEnabled(False)
            return "auto"
        else:
            self.ui.atten_box.setEnabled(True)
            return attenuation

    def spur(self):
        sType = self.ui.spur_box.currentText()
        return sType

    def setCentreFreq(self):
        startF = self.ui.centre_freq.value()-self.ui.span_freq.value()/2
        stopF = self.ui.centre_freq.value()+self.ui.span_freq.value()/2
        with QSignalBlocker(self.ui.start_freq):
            self.ui.start_freq.setValue(startF)
        with QSignalBlocker(self.ui.stop_freq):
            self.ui.stop_freq.setValue(stopF)
        self.setGraphFreq(startF * 1e6, stopF * 1e6)
        self.setting_change()

    def setStartFreq(self):
        startF = self.ui.start_freq.value()  # freq in MHz
        stopF = self.ui.stop_freq.value()
        if startF > stopF:
            stopF = startF
            with QSignalBlocker(self.ui.stop_freq):
                self.ui.stop_freq.setValue(stopF)
        with QSignalBlocker(self.ui.centre_freq):
            self.ui.centre_freq.setValue(startF + (stopF - startF) / 2)
        with QSignalBlocker(self.ui.span_freq):
            self.ui.span_freq.setValue(stopF - startF)
        self.setGraphFreq(startF * 1e6, stopF * 1e6)
        self.setting_change()

    def setGraphFreq(self, startF, stopF):
        self.ui.graphWidget.setXRange(startF, stopF)
        span = stopF - startF
        if span != 0:
            self.limits.lowF.line.setValue((startF + span/20))
            self.limits.highF.line.setValue((stopF - span/20))

    def setToMarker(self):
        mkr_freq = self.s0.trace.m0.line.value()
        with QSignalBlocker(self.ui.centre_freq):
            self.ui.centre_freq.setValue(mkr_freq / 1e6)
        self.setCentreFreq()

    def freqOffset(self, startF, stopF):
        ''''for mixers or LNBs external to TinySA.  Returns a tuple (startF, stopF)'''
        spanF = stopF - startF
        loF = self.data.bandstype.freq
        logging.debug(f'LO freq = {loF} startF = {startF} stopF = {stopF}')
        if loF > startF:  # high side LO so IF is inverted compared to (usual) low side LO
            scanF = (loF - startF - spanF, loF - startF)
        else:
            scanF = (startF - loF, startF - loF + spanF)
        if min(scanF) < 0:
            self.sweeping = False
            scanF = (88 * 1e6, 108 * 1e6)
            logging.info('LO frequency offset error, check settings')
            popUp(self.ui, "LO frequency offset error, check settings", 'Ok', 'Critical')
        logging.debug(f'freqOffset(): scanF = {scanF}')
        return scanF

    def rbwMask(self, startF, stopF):
        '''calculate a frequency width factor, used to mask readings near each maximum or minimum'''
        if self.ui.rbw_auto.isChecked():
            # auto rbw is ~7 kHz per 1 MHz scan frequency span
            approx_rbw = 7 * (stopF - startF) / 1e6  # kHz
            # find the nearest lower discrete rbw value
            for i in range(0, self.data.rbwtext.tm.rowCount() - 1):
                rbw = float(self.data.rbwtext.tm.record(i).value('value'))  # kHz
                if approx_rbw <= float(self.data.rbwtext.tm.record(i).value('value')):
                    break
            maskFreq = self.dialogs.settings.ui.rbw_x.value() * rbw * 1e3  # Hz
        else:
            # manual rbw setting
            maskFreq = self.dialogs.settings.ui.rbw_x.value() * float(self.ui.rbw_box.currentText()) * 1e3  # Hz
            logging.debug(f'manual rbw masking factor = {maskFreq/1e3}kHz')
        return maskFreq

    def rbwChanged(self):
        if self.ui.rbw_auto.isChecked():  # can't calculate Points because we don't know what the RBW will be
            self.ui.rbw_box.setEnabled(False)
            self.ui.points_auto.setChecked(False)
            self.ui.points_auto.setEnabled(False)
        else:
            self.ui.rbw_box.setEnabled(True)
            self.ui.points_auto.setEnabled(True)
        self.setting_change()
        self.setRBW()

    def setRBW(self):
        if self.ui.rbw_auto.isChecked():
            rbw = 'auto'
        else:
            rbw = float(self.ui.rbw_box.currentText())  # ui values are discrete ones in kHz
        return rbw

    def setPoints(self):
        if self.ui.points_auto.isChecked():
            rbw = float(self.ui.rbw_box.currentText())
            self.points = self.dialogs.settings.ui.rbw_x.value() * int((self.ui.span_freq.value()*1000)/(rbw))  # RBW multiplier * freq kHz
            self.points = np.clip(self.points, self.dialogs.settings.ui.minPoints.value(), self.dialogs.settings.ui.maxPoints.value())  # limit points
            with QSignalBlocker(self.ui.points_box):
                self.ui.points_box.setValue(self.points)
        else:
            self.points = self.ui.points_box.value()
            logging.debug(f'setPoints: points = {self.ui.points_box.value()}')

    def points_changed(self):
        if self.ui.points_auto.isChecked():
            self.ui.points_box.setEnabled(False)
            self.ui.rbw_box.setEnabled(True)
        else:
            self.ui.points_box.setEnabled(True)
        self.setting_change()

    def memChanged(self):
        self.depth = self.ui.memBox.value()
        if self.depth < self.ui.avgBox.value():
            self.ui.avgBox.setValue(self.depth)

    @Slot()
    def allStopped(self, restart):
        self.parent()._set_scan_button('Start scan')
        self.mkr_update_timer.start(100)
        if restart:
            self.scan()

    def updateGUI(self, route, freq, levl, maxl, minl, buffer, ser_num, dev_id, timestamp, split, sweep_end):
        ''''updates all the traces in the route in one call'''

        lo_freq = self.data.bandstype.freq
        if lo_freq !=0 and lo_freq < self.ui.start_freq.value() * 1e6:
            freq = freq + lo_freq
        elif lo_freq !=0 and lo_freq > self.ui.start_freq.value() * 1e6:
            freq = lo_freq - freq
        
        # reverse the arrays if in LNB/Mixer mode when LO is above measured freq
        if lo_freq > self.ui.start_freq.value() * 1e6:
            freq = freq[::-1]
            levl = levl[::-1]
            maxl = maxl[::-1]
            minl = minl[::-1]
            logging.info('invert freq axis')
            self.timespectrum.surface.axisX().setReversed(True)
        else:
            self.timespectrum.surface.axisX().setReversed(False)

        # check for zero span and change the spectrum graph x-axis if so
        try:
            if freq[0] == freq[-1]:
                self.ui.graphWidget.setLabel('bottom', 'Time')
                freq = np.arange(1, len(freq) + 1, dtype=int)
                self.ui.graphWidget.setXRange(freq[0], freq[-1])
            else:
                self.ui.graphWidget.setLabel('bottom', units='Hz')
        except IndexError:
            logging.info('Info: updateGUI ignored an index error')
            return

        # update the waterfall data array
        wf_auto = self.ui.waterfall_auto.isChecked()
        buffer_cols = np.shape(buffer)[1]
        if split:
            # the sweep was split in frequency across devices
            slice_start = 0
            if dev_id > 0:
                for i in range(dev_id):
                    slice_start = slice_start + self.spectra[i].points
            slice_end = slice_start + buffer_cols
            self.wf_data[:, slice_start:slice_end] = buffer
        else:
            self.wf_data = buffer  
        # self.ui.waterfall.setXRange(0, np.size(self.wf_data, axis=1))

        # update the average values (nan check to prevent "mean of empty slice" error)
        if ~np.isnan(buffer[0, :]).any():
            avg = np.nanmean(buffer[0:self.ui.avgBox.value(), :], axis=0)
        else:
            avg = levl
        
        for spectrum in route:
                if sweep_end and ~np.all(np.isnan(self.wf_data)):
                    spectrum.waterfall.setImage(self.wf_data, autoLevels=wf_auto)
                
                # update waterfall display if visible and if there is data
                if sweep_end and self.ui.waterfall_size.value() > 0:
                    if ~np.isnan(self.wf_data[0, :]).any():
                        wf_levels = spectrum.waterfall.getLevels()
                        self.timespectrum.set_dynamic_range(wf_levels)
                        self.timespectrum.updater.updateTimeSpectrum(freq, self.wf_data)
                         
                # update the spectrum trace, according to trace type
                gui_boxes = {0: self.ui.t1_type, 1: self.ui.t2_type, 2: self.ui.t3_type, 3: self.ui.t4_type}
                trace_type = gui_boxes.get(self.spectra.index(spectrum)).currentText()
                data = {'Normal': levl, 'Average': avg, 'Max': maxl, 'Min': minl, 'Freeze': levl}

                if trace_type != 'Freeze':
                    spectrum.updateTrace(freq, data.get(trace_type))
                logging.debug(f'updating {self.spectra.index(spectrum)}')

                if timestamp != 0:
                    spectrum.timestamp.setText(time.ctime(timestamp))
                else:
                    spectrum.timestamp.setText('Device ' + str(dev_id + 1))
    
                # update the markers
                maskFreq = self.rbwMask(freq[0], freq[-1])
                limits = (maskFreq,
                          self.limits.highF.line.value(),
                          self.limits.lowF.line.value(),
                          self.limits.threshold.line.value())
                
                for trace in spectrum.mkr_list:
                    trace.mkr_update(limits, spectrum.trace.is_visible)
    
                if sweep_end:
                    spectrum.count += 1
                    timeNow = time.time()

                    # update the monitor graph. timestamp is zero unless a recorded file is playing
                    if timestamp == 0:
                        spectrum.update_monitor(freq, timeNow)
                    else:
                        spectrum.update_monitor(freq, timestamp)

                    # update phase noise and pattern graphs (which use trace 1 only)
                    if spectrum == self.s0:
                        m0_index = np.argmin(np.abs(freq - (spectrum.trace.m0.line.value())))  # marker 1 index
                        if self.dialogs.phasenoise.ui.isVisible() and not self.ui.rbw_auto.isChecked():
                            self.phaseNoise.update(m0_index, freq, levl, float(self.ui.rbw_box.currentText()))
                        if self.dialogs.pattern.ui.isVisible():
                            self.polar.update_plot(self.dialogs.pattern.ui, m0_index, levl)
    
                # save sweep data to file if enabled and the waterfall data array is full
                if spectrum.count == self.depth:
                    if self.dialogs.settings.ui.saveSweep.isChecked():
                        self.save_data(freq, self.wf_data, ser_num)
                    spectrum.count = 0  # resets the counter only for the traces being updated by this route

    def set_dev_combo(self, ui_name, dev_names):
        '''Populate a device combo box with matching devices and return the matches.'''
        ui_name.device.clear()
        devices = []
        if self.hardware.devices:
            for device in self.hardware.devices:
                if device.name in dev_names:
                    if device.sweeping:
                        popUp(self.ui, f"Cannot read from {device.name} whilst a scan is running", 'Ok', 'Info')
                    else:
                        with QSignalBlocker(ui_name.device):
                            ui_name.device.addItem(device.name + ' serial ' + str(device.sn))
                            devices.append(device)  # keep a reference to the device for file ops
            # device = self.dev_ref[ui_name.device.currentIndex()]
        return devices

    def file_browser(self):
        if self.hardware.devices:
            match_vals = ('tinySA ULTRA ZS405', 'tinySA ULTRA ZS406', 'tinySA ULTRA ZS407')
            self.file_devices = self.set_dev_combo(self.dialogs.file_browse.ui, match_vals)
            if self.file_devices:
                self.dialogs.file_browse.ui.show()
                self.list_files()
            else:
                popUp(self.ui, "No browsable devices connected", 'Ok', 'Info')
        else:
            popUp(self.ui, "No browsable devices connected", 'Ok', 'Info')

    def list_files(self):
        device = self.file_devices[self.dialogs.file_browse.ui.device.currentIndex()]
        SD = device.listSD()
        self.dialogs.file_browse.ui.listWidget.clear()
        ls = []
        for i in range(len(SD.splitlines())):
            file_name = SD.splitlines()[i].split(" ")[0]
            if file_name != '.Trash-1000':
                ls.append(file_name)
        self.dialogs.file_browse.ui.listWidget.insertItems(0, ls)

    def show_file(self):
        device = self.file_devices[self.dialogs.file_browse.ui.device.currentIndex()]
        self.memF.seek(0, 0)  # set the memory buffer pointer to the start
        self.memF.truncate()  # clear down the memory buffer to the pointer
        self.dialogs.file_browse.ui.picture.clear()
        fileName = self.dialogs.file_browse.ui.listWidget.currentItem().text()
        device.clearBuffer()  # clear the tinySA serial buffer
        self.memF.write(device.readSD(fileName))  # read the file from the tinySA memory card and store in memory buffer
        if fileName[-3:] == 'bmp':
            pixmap = QPixmap()
            pixmap.loadFromData(self.memF.getvalue())
            self.dialogs.file_browse.ui.picture.setPixmap(pixmap)

    def correction_window(self):
        match_vals = ('tinySA ULTRA ZS405', 'tinySA ULTRA ZS406', 'tinySA ULTRA ZS407')
        self.correction_devices = self.set_dev_combo(self.dialogs.offset.ui, match_vals)
        self.dialogs.offset.ui.progress.setValue(0)
        self.dialogs.offset.ui.show()

    # def sweepTime(self, seconds):
    #     #  0.003 to 60S
    #     command = f'sweeptime {seconds}\r'
    #     self.fifo.put(command)

    def sweep_as_zoomed(self):
        '''find the current limits of the (frequency axis) viewbox and set the sweep to them'''
        xaxis = (self.ui.graphWidget.getAxis('bottom').range)
        startF = float(xaxis[0]/1e6)
        stopF = float(xaxis[1]/1e6)
        logging.debug(f'sweep_as_zoomed: start = {startF} stop = {stopF}')
        with QSignalBlocker(self.ui.start_freq):
            self.ui.start_freq.setValue(startF)
        with QSignalBlocker(self.ui.stop_freq):
            self.ui.stop_freq.setValue(stopF)
        self.setStartFreq()

    def markerToStart(self):
        startF = self.ui.start_freq.value()
        for spectrum in self.spectra:
            for mkr in spectrum.mkr_list:
                mkr.to_freq(startF * 1e6)

    def markerSpread(self):
        startF = self.ui.start_freq.value()
        stopF = self.ui.stop_freq.value()
        for spectrum in self.spectra:
            for index, mkr in enumerate(spectrum.mkr_list):
                mkr.spread(startF, stopF, index + 1)

    def marker_restore(self):
        fields = {0: 'm1f', 1:'m2f', 2: 'm3f', 3: 'm4f'}
        for spectrum in self.spectra:
            for i in range(4):
                freq = self.data.numbers.tm.record(0).value(fields.get(i))
                spectrum.mkr_list[i].to_freq(freq)

    def marker_save(self):
        # to save the marker frequencies on exit.  Only saves the spectrum 0 marker freqs at present.
        record = self.data.numbers.tm.record(0)
        fields = {0: 'm1f', 1:'m2f', 2: 'm3f', 3: 'm4f'}
        for i in range(4):
            freq = self.spectra[0].mkr_list[i].line.value()
            logging.debug(f'marker_save: freq = {freq}')
            record.setValue(fields.get(i), float(freq))
        self.data.numbers.tm.setRecord(0, record)

    def centreTone(self):  # for phase noise graph
        centreF = self.s0.trace.m0.line.value() * 1e-6
        self.ui.centre_freq.setValue(centreF)

    def updateMarker(self):  # called by timer when not scanning
        startF = self.ui.start_freq.value() * 1e6  # freq in Hz
        stopF = self.ui.stop_freq.value() * 1e6
        maskFreq = self.rbwMask(startF, stopF)
        limits = (maskFreq,
                  self.limits.highF.line.value(),
                  self.limits.lowF.line.value(),
                  self.limits.threshold.line.value())

        for spectrum in self.spectra:
            for marker in spectrum.mkr_list:
                marker.mkr_update(limits, spectrum.trace.is_visible)  # update markers that are inside the mask

    def set_main_marker(self, mkr_num):  # mkr number and type are provided from the GUI field changed events
        m_type = {0: self.ui.m1_type.currentText(),
                  1: self.ui.m2_type.currentText(),
                  2: self.ui.m3_type.currentText(),
                  3: self.ui.m4_type.currentText()}
        m_track = {0: self.ui.m1track.value(),
                   1: self.ui.m2track.value(),
                   2: self.ui.m3track.value(),
                   3: self.ui.m4track.value()}
        for spectrum in self.spectra:
            spectrum.mkr_list[mkr_num].set_type(m_type.get(mkr_num), m_track.get(mkr_num))

    def set_preset_marker(self):
        self.s0.del_ps_mkr(self.ui.graphWidget)
        self.data.presetmarker.unlimited()
        for i in range(0, self.data.presetmarker.tm.rowCount()):
            try:
                startF = self.data.presetmarker.tm.record(i).value('StartF')
                stopF = self.data.presetmarker.tm.record(i).value('StopF')
                colour = self.data.presetmarker.tm.record(i).value('colour')
                name = self.data.presetmarker.tm.record(i).value('name')
                visible = self.data.presetmarker.tm.record(i).value('visible')
                on = self.ui.presetMarker.isChecked()
                rotate = self.ui.presetLabel.isChecked()
                if on and visible and stopF in (0, ''):  # it is not a band marker
                    marker = self.s0.set_ps_mkr(self.ui.graphWidget, startF, colour, name)
                    self.s0.label_ps_mkr(marker, colour, rotate, False)
                if on and visible and stopF not in (0, '', startF):  # it is a band marker
                    band_start = self.s0.set_ps_mkr(self.ui.graphWidget, startF, colour, name)
                    self.s0.label_ps_mkr(band_start, colour, rotate, True)
                    band_end = self.s0.set_ps_mkr(self.ui.graphWidget, stopF, colour, name)
                    self.s0.label_ps_mkr(band_end, colour, rotate, True)
            except ValueError:
                logging.info('preset_marker {name} value error')
                continue

    def start_recording(self):
        folder = self.dialogs.settings.ui.save_folder.text()
        if not os.path.exists(folder):
            popUp(self.ui, "The save file location in 'settings > preferences' is not valid", 'Ok', 'Critical')
            return
            
        self.stop_playback()
        self.ui.record.setEnabled(False)
        count = self.hardware.num_enabled
        if not self.hardware.is_scanning:
            self.ui.scan_button.clicked.emit()
        logging.info(f'start recording from {count} devices')
        for i in range(count):
            points = self.spectra[i].points
            self.hardware.recorders[i].configure_array(points, i, count)
            self.hardware.recorders[i].sn = self.hardware.devices[i].sn
            self.hardware.recorders[i].id = i
            self.hardware.recorders[i].recording = True

    def stop_recording(self):
        folder = self.dialogs.settings.ui.save_folder.text()
        count = self.hardware.num_enabled
        for i in range(count):
            if self.hardware.recorders[i].recording:
                self.hardware.recorders[i].recording = False
                self.hardware.recorders[i].save_recording(folder)
        self.ui.record.setEnabled(True)
        
    def start_playback(self, play):
        self.stop_recording()
        self.hardware.stop()
        if np.isnan(self.hardware.devices[0].data_arr).all():
            popUp(self.ui, "No recorded spectrum data is loaded", 'Ok', 'Critical')
            return
        if self.hardware.is_scanning:
            return
        
        if self.dialogs.settings.ui.saveSweep.isChecked() and not self.parent().save_location_valid():
            popUp(self.ui, "The current save file location is not valid", 'Ok', 'Critical')
            return
        
        interval = self.dialogs.settings.ui.intervalBox.value()
        slider = self.ui.vortex.value()

        if play and slider == 100:
            # the end of time has been reached so reset the time vortex
            slider = 0
            with QSignalBlocker(self.ui.vortex):
                self.ui.vortex.setValue(0)
        
        # set the graph frequency axis to the maximum range of the loaded recordings
        start = self.hardware.devices[0].data_arr[0, 1]
        stop = self.hardware.devices[0].data_arr[0, -1]
        file_count = self.hardware.loaded_files
        for i in range(self.hardware.loaded_files):
            start = int(min(start, self.hardware.devices[i].data_arr[0, 1]))
            stop = int(max(stop, self.hardware.devices[i].data_arr[0, -1]))
        self.setGraphFreq(start, stop)

        self.set_gui_colours()
        
        # set the arrays for the signal level monitor
        self.depth = self.ui.memBox.value()
        time_points = self.dialogs.fading.ui.timePoints.value()
        for spectrum in self.spectra:
            spectrum.monitor_data = np.full((int(time_points), 2), None, dtype=float)

        # set the waterfall array size as the sum of the number of columns in all the loaded files
        points = []
        for device in self.hardware.devices:
            if ~np.isnan(device.data_arr).all():  # array has been loaded
                dev_pnts = np.size(device.data_arr, axis=1) - 1  # number of columns
                points.append(dev_pnts)
        wf_points = sum(points)
        self.wf_data = np.full((self.depth, wf_points), np.nan, dtype=float)  

        # create and start the measurement playback worker threads
        split = self.ui.split_scan.isChecked()
        for i in range(self.hardware.loaded_files):    
            if self.hardware.devices[i].enabled:
                self.spectra[i].points = points[i]
                self.hardware.devices[i].sweeping = True
                player = Worker(self.hardware.devices[i].server, self.depth, interval, slider, play, split)
                threadpool.start(player)

    def stop_playback(self):
        file_count = self.hardware.loaded_files
        for i in range(file_count):
            if self.hardware.devices[i].sweeping:
                self.hardware.devices[i].sweeping = False
    
    def set_speed(self):
        spinbox = self.ui.speed.value()
        speed = spinbox / 100
        for i in range(self.hardware.loaded_files):
            self.hardware.recorders[i].speed = speed
    
    def save_data(self, frequencies, data_arr, ser_num):
        sn_txt = str(ser_num)
        timeStamp = time.strftime('%Y-%m-%d-%H%M%S')
        folder = self.dialogs.settings.ui.save_folder.text()
        file_name = str(timeStamp + '_RBW' + self.ui.rbw_box.currentText() + '_' + sn_txt + '.CSV')
        file_name = os.path.join(folder, file_name)
        saver = Worker(save_sweep, file_name, frequencies, data_arr)
        threadpool.start(saver)  # workers are deleted when thread ends
    
    def load_data(self):
        '''loads .npy files made by the recorder into data arrays for playback'''
        dialog = QFileDialog()
        folder = self.dialogs.settings.ui.save_folder.text()
        dialog.setDirectory(folder)
        file_name = dialog.getOpenFileName(caption="Select file to load", filter="NumPy array (*.npy)")[0]
        if file_name != '':
            loader = Worker(self.hardware.load_player, file_name)
        else:
            return
        self.hardware.probe_ports.stop()  # stop probing the usb ports for analyser hardware
        threadpool.start(loader)
        self.ui.vortex.show()

    def clear_data(self):
        self.stop_playback()
        self.hardware.close_player()
        for spectrum in self.spectra:
            spectrum.waterfall.clear()
        with QSignalBlocker(self.ui.vortex):
            self.ui.vortex.setValue(0)
        self.ui.vortex.hide()
        self.hardware.probe_ports.start()  # start probing the usb ports for analyser hardware

    def read_tables(self):
        ''''read the correction tables from the tinySA and display in a table widget'''
        device = self.correction_devices[self.dialogs.offset.ui.device.currentIndex()]

        if device.sweeping:
            popUp(self.dialogs.offset.ui, "Cannot read from tinySA whilst a scan is running", 'Ok', 'Info')
            return

        self.data.correction.unlimited()
        write_config = QMessageBox.StandardButton.Ok

        if self.data.correction.tm.rowCount() > 0 and self.dialogs.offset.ui.save_box.isChecked():
            message = ('OK to over-write config database table with\rcorrection data from tinySA?')
            write_config = popUp(self.dialogs.offset.ui, message, 'OkC', 'Question')
            if write_config == QMessageBox.StandardButton.Ok:
                self.data.correction.deleteRow(single=False)
                
        self.dialogs.offset.ui.tsa_table.clear()
        self.dialogs.offset.ui.tsa_table.setRowCount(200)
        self.dialogs.offset.ui.tsa_table.setColumnCount(4)
        self.dialogs.offset.ui.tsa_table.setHorizontalHeaderLabels(['mode', 'entry', 'frequency', 'dB'])
        self.dialogs.offset.ui.tsa_table.show()
        self.dialogs.offset.ui.tsa_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.ResizeToContents)

        k = 0
        for i in range(self.data.correctiontext.tm.rowCount()):
            # step through each correction mode, fetch its table from the tinySA
            command = 'correction ' + self.data.correctiontext.tm.record(i).value('value') + '\r'
            data = device.serialQuery(command)
            mode_table = data.splitlines()[1:]  # make a list of the rows, discard the mode header
            mode_rows = [row.split(' ')[1:] for row in mode_table]  # split each row into a list, discard first col
            for j in range(20):
                # step through each row of the current mode and write the fields to the tablewidget 'tsa_table'
                mode = str(mode_rows[j][0])
                entry = str(mode_rows[j][1])
                frequency = str(mode_rows[j][2])
                dB = str(mode_rows[j][3])
                self.dialogs.offset.ui.tsa_table.setItem(k+j, 0, QTableWidgetItem(mode))
                self.dialogs.offset.ui.tsa_table.setItem(k+j, 1, QTableWidgetItem(entry))
                self.dialogs.offset.ui.tsa_table.setItem(k+j, 2, QTableWidgetItem(frequency))
                self.dialogs.offset.ui.tsa_table.setItem(k+j, 3, QTableWidgetItem(dB))

                if self.dialogs.offset.ui.save_box.isChecked() and write_config == QMessageBox.StandardButton.Ok:
                    self.data.correction.insertData(mode=mode, entry=entry, frequency=frequency, dB=dB)
            k += 20

    def upload_correction(self):
        ''''upload the correction table(s) from the config database to the tinySA'''
        device = self.correction_devices[self.dialogs.offset.ui.device.currentIndex()]
        if device.sweeping:
            popUp(self.dialogs.offset.ui, "Cannot read from tinySA whilst a scan is running", 'Ok', 'Info')
            return

        self.data.unlimited()
        self.dialogs.offset.ui.progress.setValue(0)

        update_failed = False
        for i in range(self.data.correction.tm.rowCount()):
            record = self.data.correction.tm.record(i)
            mode = str(record.value('mode')) + ' '
            entry = str(record.value('entry')) + ' '
            frequency = str(record.value('frequency')) + ' '
            dB = str(record.value('dB'))
            command = 'correction ' + mode + entry + frequency + dB + '\r'
            response = device.serialQuery(command)
            logging.debug(f'upload_correction(): {response}')
            if response != 'updated ' + entry + 'to ' + frequency + dB:
                # the error trapping on the tinySA is not comprehensive so this may not work for all scenarios
                update_failed = True
                logging.info(f'Update failure: {command}')
            else:
                self.dialogs.offset.ui.progress.setValue(100 * int(i / (self.data.correction.tm.rowCount() - 1)))
        if update_failed:
            popUp(self.dialogs.offset.ui, "One or more of the updates failed.", 'Ok', 'Critical')

    def start_polar_plot(self):
        self.polar.set_plot(self.dialogs.pattern.ui)

    def connect_active(self):
        '''Connect signals from controls that send messages to tinySA or use trace data.  Called by setGUI().'''
    
        self.ui.atten_box.editingFinished.connect(self.setting_change)
        self.ui.atten_auto.clicked.connect(self.setting_change)
        self.ui.spur_box.currentIndexChanged.connect(self.setting_change)
        self.ui.lna_box.clicked.connect(self.setting_change)
    
        # frequencies
        self.ui.start_freq.editingFinished.connect(self.setStartFreq)
        self.ui.stop_freq.editingFinished.connect(self.setStartFreq)
        self.ui.centre_freq.editingFinished.connect(self.setCentreFreq)  # centre/span mode
        self.ui.span_freq.editingFinished.connect(self.setCentreFreq)  # centre/span mode
        
        self.ui.band_box.currentIndexChanged.connect(self.band_changed)
        self.ui.setRange.clicked.connect(self.sweep_as_zoomed)
        self.ui.setToMkr.clicked.connect(self.setToMarker)
    
        self.ui.rbw_auto.clicked.connect(self.rbwChanged)
        self.ui.rbw_box.currentIndexChanged.connect(self.rbwChanged)
        self.ui.points_auto.stateChanged.connect(self.points_changed)
        self.ui.points_box.editingFinished.connect(self.points_changed)
    
        # self.ui.sampleRepeat.valueChanged.connect(self.sampleRep)
    
        # filebrowse
        self.dialogs.file_browse.ui.listWidget.itemClicked.connect(self.show_file)
    
        # Sweep time
        # self.ui.sweepTime.valueChanged.connect(lambda: self.sweepTime(self.ui.sweepTime.value()))
    
        # level calibration
        self.dialogs.offset.ui.correction_mode.currentTextChanged.connect(self.correction_filter)
        self.dialogs.offset.ui.filter_box.stateChanged.connect(self.correction_filter)
        # self.dialogs.offset.ui.read_button.clicked.connect(correction.read_tables)
        # self.dialogs.offset.ui.upload_button.clicked.connect(correction.upload_correction)
    
    def connect_passive(self):
        '''Connect signals from GUI controls that don't cause messages to go to the tinySA'''
    
        self.ui.memBox.valueChanged.connect(self.memChanged)
        self.ui.scan_button.clicked.connect(self.scan)
    
        # # marker setting within span range
        self.ui.mkr_start.clicked.connect(self.markerToStart)
        self.ui.mkr_centre.clicked.connect(self.markerSpread)
    
        # marker tracking level
        self.ui.m1track.valueChanged.connect(lambda: self.set_main_marker(0))
        self.ui.m2track.valueChanged.connect(lambda: self.set_main_marker(1))
        self.ui.m3track.valueChanged.connect(lambda: self.set_main_marker(2))
        self.ui.m4track.valueChanged.connect(lambda: self.set_main_marker(3))
    
        # marker type changes
        self.ui.m1_type.currentTextChanged.connect(lambda: self.set_main_marker(0))
        self.ui.m2_type.currentTextChanged.connect(lambda: self.set_main_marker(1))
        self.ui.m3_type.currentTextChanged.connect(lambda: self.set_main_marker(2))
        self.ui.m4_type.currentTextChanged.connect(lambda: self.set_main_marker(3))
    
        # # frequency band and fixed markers
        self.ui.presetMarker.clicked.connect(self.set_preset_marker)
        self.ui.presetLabel.clicked.connect(self.set_preset_marker)
        self.ui.filterBox.currentTextChanged.connect(self.set_preset_marker)
    
        # trace checkboxes
        self.ui.trace1.stateChanged.connect(self.s0.enable)
        self.ui.trace2.stateChanged.connect(self.s1.enable)
        self.ui.trace3.stateChanged.connect(self.s2.enable)
        self.ui.trace4.stateChanged.connect(self.s3.enable)
    
        # preset freqs and settings
        self.dialogs.preset_freqs.ui.addPs.clicked.connect(self.data.bands.addRow)
        self.dialogs.preset_freqs.ui.deletePs.clicked.connect(
            lambda: self.data.bands.deleteRow(True))
        
        self.dialogs.preset_freqs.ui.deleteAll.clicked.connect(
            lambda: self.data.bands.deleteRow(False))
        
        self.dialogs.preset_freqs.ui.addPsType.clicked.connect(self.data.bandstype.addRow)
        self.dialogs.preset_freqs.ui.finished.connect(self.set_preferences)  # update db checkboxes table on window close
        self.dialogs.preset_freqs.ui.exportPs.pressed.connect(
            lambda: self.data.bands.exportData(''))
        
        self.dialogs.preset_freqs.ui.importPs.pressed.connect(
            lambda: self.parent().import_data(self.data.bands))
    
        self.ui.actionCorrection.triggered.connect(self.correction_window)
    
        # Measurement menu
        self.ui.actionPhNoise.triggered.connect(self.dialogs.phasenoise.ui.show)
        self.ui.actionFading.triggered.connect(self.dialogs.fading.ui.show)
        self.ui.actionPattern.triggered.connect(self.dialogs.pattern.ui.show)
    
        # phase noise
        self.dialogs.phasenoise.ui.centre.clicked.connect(self.centreTone)
    
        # File menu
        self.ui.actionBrowse.triggered.connect(self.file_browser)
        self.dialogs.file_browse.ui.device.currentIndexChanged.connect(self.list_files)
        self.ui.actionRecordings.triggered.connect(self.load_data)
    
        # correction
        self.dialogs.offset.ui.export_button.clicked.connect(lambda: self.data.correction.exportData(''))
        self.dialogs.offset.ui.import_button.clicked.connect(lambda: self.data.correction.importData(''))
       
        # recording and playback
        self.ui.record.clicked.connect(self.start_recording)
        self.ui.play.clicked.connect(lambda:self.start_playback(True))
        self.ui.stop.clicked.connect(self.stop_playback)
        self.ui.stop.clicked.connect(self.stop_recording)
        self.ui.speed.valueChanged.connect(self.set_speed)
        self.ui.eject.clicked.connect(self.clear_data)
        self.ui.vortex.valueChanged.connect(lambda:self.start_playback(False))
        
        # device enable
        self.ui.dev0.stateChanged.connect(lambda: self.hardware.toggle_dev_state(0, self.ui.dev0.isChecked()))
        self.ui.dev1.stateChanged.connect(lambda: self.hardware.toggle_dev_state(1, self.ui.dev1.isChecked()))
        self.ui.dev2.stateChanged.connect(lambda: self.hardware.toggle_dev_state(2, self.ui.dev2.isChecked()))
        self.ui.dev3.stateChanged.connect(lambda: self.hardware.toggle_dev_state(3, self.ui.dev3.isChecked()))

    def band_changed(self):
        index = self.ui.band_box.currentIndex()
        startF = self.data.bandselect.tm.record(index).value('StartF')
        stopF = self.data.bandselect.tm.record(index).value('StopF')
        if stopF not in (0, '', startF):
            with QSignalBlocker(self.ui.start_freq):
                self.ui.start_freq.setValue(startF / 1e6)
            with QSignalBlocker(self.ui.stop_freq):
                self.ui.stop_freq.setValue(stopF / 1e6)
            self.setStartFreq()
        else:
            centreF = startF / 1e6
            span = int(centreF / 10)  # default span to a tenth of the centre freq
            for i in range(self.data.bandselect.tm.rowCount()):
                type_start = self.data.bandselect.tm.record(i).value('StartF')
                type_stop = self.data.bandselect.tm.record(i).value('StopF')
                if type_stop not in (0, '', startF):
                    span = (type_stop - type_start) / 1e6
                    break
            with QSignalBlocker(self.ui.centre_freq):
                self.ui.centre_freq.setValue(centreF)
            with QSignalBlocker(self.ui.span_freq):
                self.ui.span_freq.setValue(span)
            self.setCentreFreq()
        self.data.numbers.dwm.submit()
        self.set_mixer_highlighting()
    
    def set_mixer_highlighting(self):
        if self.data.bandstype.freq == 0:
            self.ui.mixerMode.setVisible(False)
            self.ui.start_freq.setStyleSheet('background-color:None')
            self.ui.stop_freq.setStyleSheet('background-color:None')
            self.ui.centre_freq.setStyleSheet('background-color:None')
            self.ui.start_freq.setMaximum(self.maxF)
            self.ui.centre_freq.setMaximum(self.maxF)
            self.ui.stop_freq.setMaximum(self.maxF)
        else:
            self.ui.mixerMode.setVisible(True)
            self.ui.start_freq.setStyleSheet('background-color:lightGreen')
            self.ui.stop_freq.setStyleSheet('background-color:lightGreen')
            self.ui.centre_freq.setStyleSheet('background-color:lightGreen')
            self.ui.start_freq.setMaximum(100000)
            self.ui.centre_freq.setMaximum(100000)
            self.ui.stop_freq.setMaximum(100000)

    def set_preferences(self):  # called at startup and when the preferences window is closed
        self.data.checkboxes.dwm.submit()
        self.data.bands.tm.submitAll()
        self.limits.threshold.line.setValue(self.dialogs.settings.ui.peakThreshold.value())
        self.limits.best.visible(self.dialogs.settings.ui.neg25Line.isChecked())
        self.limits.maximum.visible(self.dialogs.settings.ui.zeroLine.isChecked())
        self.limits.damage.visible(self.dialogs.settings.ui.plus6Line.isChecked())

        if self.ui.presetMarker.isChecked():
            self.set_preset_marker()

    def correction_filter(self):
        if self.dialogs.offset.ui.filter_box.isChecked():
            sql = 'mode = "' + self.dialogs.offset.ui.correction_mode.currentText() + '"'
            self.data.correction.tm.setFilter(sql)
        else:
            self.data.correction.tm.setFilter('')


class LimitLines:
    def __init__(self, graph, threshold, start_freq, stop_freq, span_freq):
        self.best = Limit(graph, 'gold', None, -25, movable=False)
        self.maximum = Limit(graph, 'red', None, 0, movable=False)
        self.damage = Limit(graph, 'red', None, 6, movable=False)
        self.threshold = Limit(graph, 'cyan', None, threshold, movable=True)
        self.lowF = Limit(graph,'cyan', (start_freq + span_freq / 20) * 1e6, None, movable=True)
        self.highF = Limit(graph,'cyan',(stop_freq - span_freq / 20) * 1e6, None, movable=True)
        self.reference = Limit(graph, 'yellow', None, -110, movable=True)

        self.best.create(True, '|>', 0.99)
        self.maximum.create(True, '|>', 0.99)
        self.damage.create(False, '|>', 0.99)
        self.threshold.create(True, '<|', 0.99)
        self.lowF.create(True, '|>', 0.01)
        self.highF.create(True, '<|', 0.01)
        self.reference.create(True, '<|>', 0.99)
        
class Limit:
    def __init__(self, graph, pen, x, y, movable):  # x = None, horizontal.  y = None, vertical
        self.graph = graph
        self.pen = pen
        self.x = x
        self.y = y
        self.movable = movable

    def create(self, dash=False, mark='', posn=0.99):
        label = ''
        if self.y:
            label = '{value:.1f}'
        self.line = self.graph.addLine(self.x, self.y, movable=self.movable, pen=self.pen, label=label,
                                              labelOpts={'position': 0.98, 'color': (self.pen), 'movable': True})
        self.line.addMarker(mark, posn, 10)
        if dash:
            self.line.setPen(self.pen, width=0.5, style=QtCore.Qt.PenStyle.DashLine)

    def visible(self, show=True):
        if show:
            self.line.show()
        else:
            self.line.hide()


class ModelView():
    '''Represents and manages an individual table/model.  Owns generic database/table
       operations operations only, without knowledge of the UI 
       structure, other models, dialogs, hardware, or controller.'''
    
    def __init__(self, table_name, db_name, ro_columns):
        self.currentRow = 0
        self.ID = 0
        self.freq = 0
        self.createTableModel(table_name, db_name, ro_columns)
    
    def createTableModel(self, table_name, db_name, limit_edit):
        self.tm = CustomTableModel(db=db_name, ro_columns=limit_edit)
        self.tm.setTable(table_name)
    
    def set_current_row(self, row):
        self.currentRow = row
        logging.debug(f'set_current_row: row {self.currentRow} clicked')

    def set_filter(self, sql):
        self.tm.setFilter(sql)

    def clear_filter(self):
        self.tm.setFilter('')
        self.unlimited()

    def createMapper(self):
        self.dwm = QDataWidgetMapper()
        self.dwm.setModel(self.tm)
        self.dwm.setSubmitPolicy(QDataWidgetMapper.SubmitPolicy.AutoSubmit)

    def addRow(self):  # adds a blank row to the table widget above current row
        logging.debug(f'addRow(): currentRow = {self.currentRow}')
        if self.currentRow == 0:
            self.tm.insertRow(0)
        else:
            self.tm.insertRow(self.currentRow)
        self.tm.layoutChanged.emit()  # don't invoke select() because row not yet populated or saved

    def saveChanges(self):
        self.dwm.submit()

    def deleteRow(self, single=True):  # deletes rows in the table widget
        if single:
            logging.debug(f'deleteRow: current row = {self.currentRow}')
            self.tm.removeRow(self.currentRow)
        else:
            for i in reversed(range(0, self.tm.rowCount())):  # Reversing avoids a shifting-index problem
                self.tm.removeRow(i)
        self.tm.select()
        self.tm.layoutChanged.emit()

    def insertData(self, **data):
        record = self.tm.record()
        logging.debug(f'insertData: record = {record}')
        for key, value in data.items():
            logging.debug(f'insertData: key = {key} value={value}')
            record.setValue(str(key), value)
        self.tm.insertRecord(-1, record)  # -1 means after existing records
        self.tm.select()
        self.tm.layoutChanged.emit()
    
    def import_records(self, records):
        inserted = 0
        failed = 0
        for record in records:
            write_rec = self.tm.insertRecord(-1, record)
            if write_rec:
                inserted += 1
        self.tm.select()
        self.tm.layoutChanged.emit()
        failed = len(records) - inserted
        return inserted, failed

    def relation_id(self, field, display_value):
        """Resolve a relation's display value to its key using QSqlRelationalTableModel inbuilt functions."""
        column = self.tm.fieldIndex(field)
        relation = self.tm.relation(column)
        relation_model = self.tm.relationModel(column)
    
        if relation_model is None:
            logging.warning(f"No relation configured for field '{field}'")
            return None
    
        related_value = relation_model.fieldIndex(relation.displayColumn())
        related_key = relation_model.fieldIndex(relation.indexColumn())
    
        for row in range(relation_model.rowCount()):
            record = relation_model.record(row)
            if record.value(related_value) == display_value:
                return record.value(related_key)
    
        logging.warning(f"Unknown '{field}' value of '{related_value}'")
        return None

    def writeCSV(self, fileName):
        header = []
        for i in range(1, self.tm.columnCount()):
            header.append(self.tm.record().fieldName(i))
        with open(fileName, "w") as fileOutput:
            output = csv.writer(fileOutput)
            output.writerow(header)
            for rowNumber in range(self.tm.rowCount()):
                fields = [self.tm.data(self.tm.index(rowNumber, columnNumber))
                          for columnNumber in range(1, self.tm.columnCount())]
                output.writerow(fields)

    def exportData(self, filename=''):
        if filename == '':
            filename = QFileDialog.getSaveFileName(caption="Save As", filter="Comma Separated Values (*.csv)")[0]
        logging.info(f'exporting data to {filename}')
        if filename != '':
            self.writeCSV(filename)

    def unlimited(self):  # remove 256 row limit for QSql Query
        while self.tm.canFetchMore():
            self.tm.fetchMore()

    def update_row(self, row, **data):
        record = self.tm.record(row)
        for key, value in data.items():
            logging.debug(f'update_row: key = {key} value={value}')
            record.setValue(str(key), value)
        self.tm.setRecord(row, record)
        # self.updateModel()
        self.tm.select()
        self.tm.layoutChanged.emit()

 
###############################################################################
# other methods

def clickEvent():
    logging.info('clickEvent')
  
    
def save_sweep(file_name, frequencies, readings):
    # insert the measurement freqs at the top of the readings array
    array = np.insert(readings, 0, frequencies, axis=0)
    
    # transpose columns and rows
    dBm = np.transpose(np.round(array, decimals=2))
    
    np.savetxt(file_name, dBm, delimiter=',', fmt='%.2f')


def fetchVersion(db):
    query = QSqlQuery(db)
    query.exec("PRAGMA user_version;")  # execute PRAGMA command to fetch the user-defined version number
    query.next()  # advances to the result row, and query.value(0) retrieves the user version.
    version = query.value(0)
    query.clear()
    return version

def popUp(window, message, button, icon):
    if window is None:
        window = QApplication.activeWindow()
    icons = {'Warn': QMessageBox.Icon.Warning, 'Info': QMessageBox.Icon.Information,
             'Critical': QMessageBox.Icon.Critical, 'Question': QMessageBox.Icon.Question}
    buttons = {'Ok': QMessageBox.StandardButton.Ok, 'Cancel': QMessageBox.StandardButton.Cancel,
               'OkC': QMessageBox.StandardButton.Ok | QMessageBox.StandardButton.Cancel}
    msg = QMessageBox(parent=(window))
    msg.setIcon(icons.get(icon))
    msg.setText(message)
    msg.setStandardButtons(buttons.get(button))
    return msg.exec()
    
def main():

    app = QApplication.instance()
    if not app:
        app = QApplication([])
    app.setApplicationName('QtTinySA')
    app.setApplicationVersion(' v2.1.y')
    logging.info(f'{app.applicationName()} {app.applicationVersion()}')

    # pyqtgraph custom exporters
    WWBExporter.register()
    WSMExporter.register()

    controller = ApplicationController(app)

    controller.analyser.setGraphs()
    controller.analyser.setGUI()
    controller.analyser.setSignals()

    controller.ui.show()
    controller.ui.setWindowTitle(app.applicationName() + app.applicationVersion())
    controller.ui.setWindowIcon(QIcon(os.path.join(basedir, 'tinySAsmall.png')))  

    # run the application until the user closes it
    try:
        app.exec()
    finally:
        # exit_handler()  # close cleanly
        controller.shutdown()
        app.quit()

if __name__ == "__main__":
    main()    