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
    """Owns the lifetime and coordination of the application.

        ApplicationController
        │
        ├── ui
        ├── dialogs
        ├── hardware
        ├── models
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
        self._DB_VERSION = 210

        self._create_ui()
        self._create_dialogs()
        self._create_device()
        self._set_database()
        self._create_models()
        self._setup_ps_freq_table()
        self._setup_trace_colours_table()
        self._setup_correction_table()
        self._bind_comboboxes()
        self._set_controls()
        self._create_analyser()
        self._set_timers()

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
        self.analyser = Analyser(ui=self.ui, hardware=self.hardware, dialogs=self.dialogs, models=self.models, parent=self)
        
    def _create_models(self):
        self.models = Models(self.config)
        
    def _setup_ps_freq_table(self):
        table = self.dialogs.preset_freqs.ui.freqTable
        presets = QSqlRelationalDelegate(table)
        table.setItemDelegate(presets)
        colHeader = table.horizontalHeader()
        colHeader.setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.ResizeToContents)
        # do we need table.select() here?
        
        # connect the preset types window table widget to the data model
        logging.info(f'_setup_ps_freq_table: bandstype = {self.models.bandstype}')
        self.dialogs.preset_freqs.ui.typeTable.setModel(self.models.bandstype.tm)
        
        # connect the preset frequencies window table widget to the data model
        #self.dialogs.preset_freqs.ui.freqTable.setModel(bands.tm)
        self.dialogs.preset_freqs.ui.freqTable.setModel(self.models.bands.tm)
        self.dialogs.preset_freqs.ui.freqTable.hideColumn(0)  # ID
        self.dialogs.preset_freqs.ui.typeTable.hideColumn(0)  # hide primary key so user can't change it

        # populate the preset bands and markers dialogue and ui filter combo boxes
        # self.ui.filterBox.setModel(self.models.bandstype.tm)
        self.ui.filterBox.setModel(self.models.bandstype.tm)
        self.ui.filterBox.setModelColumn(1)

    def _setup_trace_colours_table(self):
        tracesettings = QSqlRelationalDelegate(self.dialogs.settings.ui.colourTable)
        self.dialogs.settings.ui.colourTable.setItemDelegate(tracesettings)
        traceHeader = self.dialogs.settings.ui.colourTable.horizontalHeader()
        traceHeader.setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.ResizeToContents)
        self.dialogs.settings.ui.colourTable.setModel(self.models.tracecolours.tm)
        self.dialogs.settings.ui.colourTable.hideColumn(0)  # ID
        self.dialogs.settings.ui.colourTable.verticalHeader().setVisible(True)
        
    def _bind_comboboxes(self):
        # the spur combobox is not bound to a model so is populated directly in the controller
    
        # populate the rbw combobox
        self.ui.rbw_box.setModel(self.models.rbwtext.tm)
    
        # populate the trace comboboxes
        self.ui.t1_type.setModel(self.models.tracetext.tm)
        self.ui.t2_type.setModel(self.models.tracetext.tm)
        self.ui.t3_type.setModel(self.models.tracetext.tm)
        self.ui.t4_type.setModel(self.models.tracetext.tm)
    
        # populate the marker comboboxes
        self.ui.m1_type.setModel(self.models.markertext.tm)
        self.ui.m2_type.setModel(self.models.markertext.tm)
        self.ui.m3_type.setModel(self.models.markertext.tm)
        self.ui.m4_type.setModel(self.models.markertext.tm)
    
        # populate the correction text combobox
        self.dialogs.offset.ui.correction_mode.setModel(self.models.correctiontext.tm)
        
        # populate the band select combobox
        self.ui.band_box.setModel(self.models.bandselect.tm)
        self.ui.band_box.setModelColumn(1)
        
    def _setup_correction_table(self):
        # setup correction table
        c_header = self.dialogs.offset.ui.c_table.horizontalHeader()
        c_header.setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.ResizeToContents)
        self.dialogs.offset.ui.c_table.setModel(self.models.correction.tm)
        self.dialogs.offset.ui.c_table.hideColumn(0)

    def _set_controls(self):
        # populate the spur combo box and map the other combo boxes to the database fields
        self.ui.spur_box.addItems(['off', 'on', 'auto'])
        self.ui.spur_box.setCurrentIndex(2)
        
        self.map_table_to_widgets(self.models.checkboxes,'checkboxes')
        self.map_table_to_widgets(self.models.numbers, 'numbers')
        
    def _set_database(self):
        # Database and models for configuration settings
        self.config = connect("QtTSAprefs.db", "settings", self._DB_VERSION, self.app_name, self.ui)

    def _set_timers(self):
        self.hardware.probe_ports.start(500)
        self.auto_run_timer = QtCore.QTimer()
        self.auto_run_timer.timeout.connect(self._run_on_start)
        if self.dialogs.settings.ui.auto_run.isChecked():
            self.auto_run_timer.start(4000)

    def map_table_to_widgets(self, model, model_name):
        ''''map widget fields to the appropriate database table fields, using the mapping table'''
        mapping = self.models.mapping
        
        # filter the mapping table to show just values for this modelName
        mapping.tm.setFilter('model = "' + model_name + '"')
        namespace = {'ui': self.ui, 'dialogs': self.dialogs}

        for index in range(mapping.tm.rowCount()):
            # the mapping table 'gui' column determines which ui field is mapped, as '...ui.field'
            gui = mapping.tm.record(index).value('gui')
            column = mapping.tm.record(index).value('column')
            widget = eval(gui, {}, namespace)
            model.dwm.addMapping(widget, int(column))

    def setup_gui(self):
        '''Configure the various GUIs'''  # what calls it?

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

        # widget settings for Phase Noise
        self.dialogs.phasenoise.ui.plotWidget.setYRange(-120, -40)
        self.dialogs.phasenoise.ui.plotWidget.plotItem.showGrid(x=True, y=True, alpha=0.5)
        self.dialogs.phasenoise.ui.plotWidget.plotItem.setLogMode(x=True)
        self.dialogs.phasenoise.ui.plotWidget.setLabel('bottom', 'Offset Frequency', units='Hz')
        self.dialogs.phasenoise.ui.plotWidget.setLabel('left', 'Phase Noise', units='dBc/Hz')
        
        # Band and frequency changes signals
        self.ui.filterBox.currentTextChanged.connect(self.set_band_filter)
        self.dialogs.preset_freqs.ui.deletePsType.clicked.connect(self.delete_preset_type)
        self.dialogs.preset_freqs.ui.freqTable.clicked.connect(self.preset_frequency_clicked)
        self.dialogs.preset_freqs.ui.typeTable.clicked.connect(self.preset_type_clicked)
        self.dialogs.preset_freqs.ui.clearFilter.clicked.connect(self.clear_preset_frequency_filter)
        
        # Correction table signals
        self.dialogs.offset.ui.read_button.clicked.connect(self.analyser.correction.read_tables)
        self.dialogs.offset.ui.upload_button.clicked.connect(self.analyser.correction.upload_correction)
        
        # Waterfall signals
        self.ui.waterfall_size.valueChanged.connect(self.set_wf_height)
        self.ui.wf_2D.stateChanged.connect(self.set_wf_format)

        # menu actions
        self.ui.actionPresets.triggered.connect(self.preferences_menu_clicked)
        self.ui.actionSettings.triggered.connect(settings_clicked)
        self.ui.actionAbout_Qt.triggered.connect(self.about)
        self.ui.actionAbout_Qt.triggered.connect(self.app.aboutQt)

        # fixed markers
        self.ui.addFix.clicked.connect(self.add_fixed)

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
        self.models.checkboxes.dwm.submit()
        self.models.numbers.dwm.submit()
        disconnect(self.config)
        self.app.closeAllWindows()
        logging.info('QtTinySA Closed')

    def _run_on_start():
        self.auto_run_timer.stop()
        self.ui.scan_button.clicked.emit()

    def delete_preset_type(self):
        bandstype = self.models.bandstype
        bands = self.models.bands
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
        bandstype = self.models.bandstype
        bands = self.models.bands
        table = self.dialogs.preset_freqs.ui.typeTable
        bandstype.set_current_row(table.currentIndex().row())
        record = bandstype.tm.record(bandstype.currentRow)
        bands.set_filter_to(True, record.value('preset'))
        bands.unlimited()
        self.dialogs.preset_freqs.ui.psCount.setValue(bands.tm.rowCount())
        
    def preset_frequency_clicked(self):
        table = self.dialogs.preset_freqs.ui.freqTable
        self.models.bands.set_current_row(table.currentIndex().row())

    def clear_preset_frequency_filter(self):
        bands = self.models.bands
        dialog = self.dialogs.preset_freqs
        dialog.ui.typeTable.clearSelection()
        bands.clear_filter()
        dialog.ui.psCount.setValue(bands.tm.rowCount())

    def set_band_filter(self, boxText):
        bandselect = self.models.bandselect
        bandstype = self.models.bandstype
    
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
    
        self.set_mixer_highlighting()  # do we need this here?
       
    def import_data(self, model):
        file_name = QFileDialog.getOpenFileName(caption="Open File", filter="Comma Separated Values (*.csv)")[0]
    
        logging.info(f'importing data from {file_name}')
    
        if file_name != '':
            self.import_preset(model, file_name)

    def import_preset(self, model, file_name):
        records = []
        with open(file_name, "r") as file_input:
            reader = csv.DictReader(file_input)
            for row in reader:
                logging.debug(f'import_csv(): row = {row}')
                record = model.tm.record()
                for key, value in row.items():
                    if key == 'preset':
                        value = self.models.bandstype.fetch_ID('preset', value)
                    if key == 'colour':
                        value = self.models.colours.fetch_ID('colour', value)
                    if key == 'value':
                        value = int(eval(value))
                    if key == 'Frequency':
                        key = 'startF'
                        value = str(float(value) / 1e3)
                    if key != 'ID':
                        record.setValue(str(key), value)
                if record.value('value') not in (0, 1):
                    record.setValue('value', 1)
                if record.value('preset') == '':
                    preset = self.dialogs.preset_freqs.ui.filterBox.currentText()
                    record.setValue('preset', self.models.bandstype.fetch_ID('preset', preset))
    
                records.append(record)
    
        inserted, skipped = model.import_records(records)
    
        message = ('Inserted ' + str(inserted) + ' rows, skipped ' + str(skipped) + ' duplicates')
        popUp(self.ui, message, 'Ok', 'Info')

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
        self.analyser.save_location_valid()
        self.dialogs.settings.ui.show()

    def preferences_menu_clicked(self):  # called by clicking on the setup > preferences menu
        self.dialogs.presetFreqs.ui.show()
        self.dialogs.presetFreqs.ui.psCount.setValue(bands.tm.rowCount())
    
    def about(self):
        message = ('TinySA Ultra GUI programme using Qt6 PySide6\
                   \nAuthor: Ian Jefferson G4IXT\n\nVersion: {} \nConfig: {}'
                   .format(app.applicationVersion(), self.config.databaseName()))
        popUp(self.ui, message, 'Ok', 'Info')

    def addFixed():
        title = "New fixed frequency Marker"
        message = "Enter a name for the fixed Marker"
        fixedMkr, ok = QInputDialog.getText(None, title, message, QLineEdit.Normal, "")
        controller.models.bands.insertData(name=fixedMkr, preset=12,
                                           startF=f'{int(self.analyser.s0.trace.m0.line.value())}',
                                           stopF=0, visible=1,
                                           colour=self.models.colours.fetch_ID('colour', 'orange'))

    def set_folder(ui_name):
        folder = QFileDialog.getExistingDirectory()
        ui_name.save_folder.setText(folder)
        self.analyser.save_location_valid()

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


class Models:
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
        mapping = ModelView('mapping', self.config, ())
        mapping.tm.select()
        return mapping

    def _set_bands(self):
        # the preset frequencies relational table in the presetFreqs window
        bands = ModelView('frequencies', self.config, ())
        bands = ModelView('frequencies', self.config, ())
        bands.tm.setSort(3, QtCore.Qt.SortOrder.AscendingOrder)
        bands.tm.setHeaderData(5, QtCore.Qt.Orientation.Horizontal, "visible")
        bands.tm.setEditStrategy(QSqlRelationalTableModel.EditStrategy.OnRowChange)
        bands.tm.setRelation(2, QSqlRelation("freqtype", "ID", "preset"))  # set "type" column to a freq type combo box
        bands.tm.setRelation(5, QSqlRelation("boolean", "ID", "value"))  # set "view" column to a True/False combo box
        bands.tm.setRelation(6, QSqlRelation("SVGColour", "ID","colour"))  # set "marker" column to a colours combo box
        bands.tm.select()
        return bands
    
    def _set_bandstype(self):
        # the preset Types table in the preset frequencies window
        bandstype = ModelView('freqtype', self.config, ())
        logging.info(f'Models: bandstype = {bandstype}')
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
        # QtTSA.band_box.setModel(bandselect.tm)
        # QtTSA.band_box.setModelColumn(1)
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
        checkboxes.dwm.setCurrentIndex(0)  # 0 = (last used) default settings
        return checkboxes
        
    def _set_rbwtext(self):
        rbwtext = ModelView('combo', self.config, ())
        return rbwtext
        
    def _set_correctiontext(self):
        correctiontext = ModelView('combo', self.config, ())
        return correctiontext
        
    def _set_numbers(self):
        # Map data tables to presets/settings/GUI fields - must be here & in this order
        numbers = ModelView('numbers', self.config, ())
        numbers.createMapper()
        numbers.tm.select()
        numbers.dwm.setCurrentIndex(0)
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
        
    def _create_combo_box(self, combo_type):
        combo = ModelView('combo', self.config, ())
        combo.tm.setFilter(f'type = "{combo_type}"')
        combo.tm.select()
        return combo
  
      
class Analyser(QtCore.QObject):
    '''owns operations that perform measurements and gui graph updates
       by coordination of hardware, dialogs and data models'''
       
    def __init__(self, ui, hardware, dialogs, models, parent=None):
        super().__init__(parent)
        
        self.ui = ui
        self.hardware = hardware
        self.dialogs = dialogs
        self.models = models
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
        self.hardware.signals.progress.connect(self.time_path_indicator)
        self.hardware.stopped.connect(self.allStopped)
        self.hardware.update_info.connect(self.set_device_info)
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
        # self.models.bandselect.set_filter_to(False, self.ui.filterBox.currentText())  # setting filter overwrites band

        self.set_mixer_highlighting()
        index = self.ui.band_box.findText(band, QtCore.Qt.MatchExactly)
        
        if self.dialogs.settings.ui.bold_text.isChecked():
            app.setStyleSheet("QWidget { font-weight: bold; }")  # enhancement issue 118

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

    @Slot()
    def set_device_info(self, port_num, description, tooltip, state):
        gui_ctrl = {0: self.ui.dev0, 1: self.ui.dev1, 2: self.ui.dev2, 3: self.ui.dev3}
        gui_ctrl.get(port_num).setText(description)
        gui_ctrl.get(port_num).setToolTip(tooltip)
        gui_ctrl.get(port_num).setChecked(state)

    def setting_change(self):
        if self.hardware.is_scanning:
            self.hardware.stop(True)
        self.stop_recording()

    def set_box_colour(self, pen, box):
        boxes = [self.ui.trace1, self.ui.trace2, self.ui.trace3, self.ui.trace4]
        tint = str("background-color: '" + pen + "';")
        boxes[box].setStyleSheet(tint)
            
    def split_scan(self, startF, stopF, points, split):
        # splits the spectrum start/stop variables across multiple devices
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
            popUp(QtTSA, 'No spectrum analyser devices found', 'Ok', 'Critical')
            return
        if self.dialogs.settings.ui.saveSweep.isChecked() and not self.save_location_valid():
            popUp(QtTSA, "The current save file location is not valid", 'Ok', 'Critical')
            settings_clicked()
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
            popUp(QtTSA, 'No devices enabled', 'Ok', 'Critical')
            return
        self.hardware.controls(rbw, attn, lna, spur)

        # For LNB / transverter mode, modify startF and stopF to suit LO freq
        if self.models.bandstype.freq != 0:
            startF, stopF = self.freqOffset(startF, stopF)

        self.split_scan(startF, stopF, self.points, split)
        self.set_gui_colours()
        self.set_arrays()

        # start device(s) scanning
        self.hardware.start(self.spectra, rbw, self.depth, maxF, interval, split, loop=True)
        self.scan_button('Stop Scan')

    def set_gui_colours(self):
        # set each spectrum (trace & marker) and box colours
        for indx, spectrum in enumerate(self.spectra):
            spectrum.count = 0
            pen = self.models.tracecolours.tm.record(indx).value('colour')
            spectrum.set_colour(pen)
            self.set_box_colour(pen, indx)

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
        loF = self.models.bandstype.freq
        logging.debug(f'LO freq = {loF} startF = {startF} stopF = {stopF}')
        if loF > startF:  # high side LO so IF is inverted compared to (usual) low side LO
            scanF = (loF - startF - spanF, loF - startF)
        else:
            scanF = (startF - loF, startF - loF + spanF)
        if min(scanF) < 0:
            self.sweeping = False
            scanF = (88 * 1e6, 108 * 1e6)
            logging.info('LO frequency offset error, check settings')
            popUp(QtTSA, "LO frequency offset error, check settings", 'Ok', 'Critical')
        logging.debug(f'freqOffset(): scanF = {scanF}')
        return scanF

    def rbwMask(self, startF, stopF):
        '''calculate a frequency width factor, used to mask readings near each maximum or minimum'''
        if self.ui.rbw_auto.isChecked():
            # auto rbw is ~7 kHz per 1 MHz scan frequency span
            approx_rbw = 7 * (stopF - startF) / 1e6  # kHz
            # find the nearest lower discrete rbw value
            for i in range(0, self.models.rbwtext.tm.rowCount() - 1):
                rbw = float(self.models.rbwtext.tm.record(i).value('value'))  # kHz
                if approx_rbw <= float(self.models.rbwtext.tm.record(i).value('value')):
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

    def points_changed():
        if self.ui.points_auto.isChecked():
            self.ui.points_box.setEnabled(False)
            self.ui.rbw_box.setEnabled(True)
        else:
            self.points_box.setEnabled(True)
        self.setting_change()

    def memChanged(self):
        self.depth = self.ui.memBox.value()
        if self.depth < self.ui.avgBox.value():
            self.ui.avgBox.setValue(self.depth)

    @Slot()
    def allStopped(self, restart):
        self.scan_button('Start scan')
        self.mkr_update_timer.start(100)
        if restart:
            self.scan()

    def updateGUI(self, route, freq, levl, maxl, minl, buffer, ser_num, dev_id, timestamp, split, sweep_end):
        ''''updates all the traces in the route in one call'''

        if self.models.bandstype.freq !=0 and bandstype.freq < self.ui.start_freq.value() * 1e6:
            freq = freq + self.models.bandstype.freq
        elif self.models.bandstype.freq !=0 and self.models.bandstype.freq > self.ui.start_freq.value() * 1e6:
            freq = self.models.bandstype.freq - freq
        
        # reverse the arrays if in LNB/Mixer mode when LO is above measured freq
        if self.models.bandstype.freq > self.ui.start_freq.value() * 1e6:
            freq = freq[::-1]
            levl = levl[::-1]
            maxl = maxl[::-1]
            minl = minl[::-1]
            logging.info('invert freq axis')
            # self.ui.waterfall.invertX(True)
            self.timespectrum.surface.axisX().setReversed(True)
        else:
            # self.ui.waterfall.invertX(False)
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
                    spectrum.timestamp.setText('')
    
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
                        self.save_data(freq, self.wf_data, ser_num, 0)
                    spectrum.count = 0  # resets the counter only for the traces being updated by this route

    def scan_button(self, action):
        self.ui.scan_button.setText(action)
        self.ui.scan_button.setEnabled(True)

    def set_dev_combo(self, ui_name, dev_names):
        '''Populate a device combo box with matching devices and return the matches.'''
        ui_name.device.clear()
        devices = []
        if self.hardware.devices:
            for device in self.hardware.devices:
                if device.name in dev_names:
                    if device.sweeping:
                        popUp(self.ui, "Cannot browse whilst a scan is running", 'Ok', 'Info')
                    else:
                        with QSignalBlocker(ui_name.device):
                            ui_name.device.addItem(device.name + ' serial ' + str(device.sn))
                            devices.append(device)  # keep a reference to the device for file ops
            # device = self.dev_ref[ui_name.device.currentIndex()]
        return devices

    # def file_browser(self):
    #     if self.hardware.devices:
    #         self.set_dev_combo(filebrowse.ui, ('tinySA ULTRA ZS405', 'tinySA ULTRA ZS406', 'tinySA ULTRA ZS407'))
    #         filebrowse.ui.show()
    #         self.list_files()

    def file_browser(self):
        if self.hardware.devices:
            match_vals = ('tinySA ULTRA ZS405', 'tinySA ULTRA ZS406', 'tinySA ULTRA ZS407')
            self.file_devices = self.set_dev_combo(self.dialogs.file_browse.ui, match_vals)
            self.dialogs.file_browse.ui.show()
            self.list_files()

    def list_files(self):
        device = self.file.devices[self.dialogs.file_browse.ui.device.currentIndex()]
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

    # def correction_window(self):
    #     self.set_dev_combo(offset.ui, ('tinySA ULTRA ZS405', 'tinySA ULTRA ZS406', 'tinySA ULTRA ZS407'))
    #     self.dialogs.offset.ui.progress.setValue(0)
    #     self.dialogs.offset.ui.show()

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
                freq = self.models.numbers.tm.record(0).value(fields.get(i))
                spectrum.mkr_list[i].to_freq(freq)

    def marker_save(self):
        # to save the marker frequencies on exit.  Only saves the spectrum 0 marker freqs at present.
        record = numbers.tm.record(0)
        fields = {0: 'm1f', 1:'m2f', 2: 'm3f', 3: 'm4f'}
        for i in range(4):
            freq = self.spectra[0].mkr_list[i].line.value()
            logging.debug(f'marker_save: freq = {freq}')
            record.setValue(fields.get(i), float(freq))
        numbers.tm.setRecord(0, record)

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
        self.models.presetmarker.unlimited()
        for i in range(0, self.models.presetmarker.tm.rowCount()):
            try:
                startF = self.models.presetmarker.tm.record(i).value('StartF')
                stopF = self.models.presetmarker.tm.record(i).value('StopF')
                colour = self.models.presetmarker.tm.record(i).value('colour')
                name = self.models.presetmarker.tm.record(i).value('name')
                visible = self.models.presetmarker.tm.record(i).value('visible')
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
            popUp(QtTSA, "The current save file location is not valid", 'Ok', 'Critical')
            settings_clicked()
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
        # if np.isnan(self.hardware.recorders[0].data_arr).all():
        if np.isnan(self.hardware.devices[0].data_arr).all():
            popUp(QtTSA, "No recorded spectrum data is loaded", 'Ok', 'Critical')
            return
        if self.hardware.is_scanning:
            return
        
        if self.dialogs.settings.ui.saveSweep.isChecked() and not self.save_location_valid():
            popUp(QtTSA, "The current save file location is not valid", 'Ok', 'Critical')
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
    
    def save_data(self, frequencies, data_arr, ser_num, dev_num):
        sn_txt = str(ser_num)
        timeStamp = time.strftime('%Y-%m-%d-%H%M%S')
        folder = self.dialogs.settings.ui.save_folder.text()
        file_name = str(timeStamp + '_RBW' + self.ui.rbw_box.currentText() + '_' + sn_txt + '.CSV')
        file_name = os.path.join(folder, file_name)
        saver = Worker(save_sweep, folder, file_name, frequencies, data_arr, ser_num)
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

    def set_sd_file_save(self, single=True):
        device = self.file_devices[self.dialogs.file_browse.ui.device.currentIndex()]
        folder = QFileDialog.getExistingDirectory(caption="Select folder to save SD card files")
        if single:
            file_name = self.dialogs.file_browse.ui.listWidget.currentItem().text()  # the file selected in the list widget
        else:
            file_name = None
        saver = Worker(sd_file_save, device, file_name, folder, single)
        threadpool.start(saver)  # workers deleted when thread ends

    def read_tables(self):
        ''''read the correction tables from the tinySA and display in a table widget'''
        device = self.correction_devices[self.dialogs.offset.ui.device.currentIndex()]

        if device.sweeping:
            popUp(offset, "Cannot read from tinySA whilst a scan is running", 'Ok', 'Info')
            return

        self.models.correction.unlimited()
        write_config = QMessageBox.StandardButton.Ok

        if self.models.correcton.tm.rowCount() > 0 and self.dialogs.offset.ui.save_box.isChecked():
            message = ('OK to over-write config database table with\rcorrection data from tinySA?')
            write_config = popUp(offset, message, 'OkC', 'Question')
            if write_config == QMessageBox.StandardButton.Ok:
                self.models.correction.deleteRow(single=False)
                
        self.dialogs.offset.ui.tsa_table.clear()
        self.dialogs.offset.ui.tsa_table.setRowCount(200)
        self.dialogs.offset.ui.tsa_table.setColumnCount(4)
        self.dialogs.offset.ui.tsa_table.setHorizontalHeaderLabels(['mode', 'entry', 'frequency', 'dB'])
        self.dialogs.offset.ui.tsa_table.show()
        self.dialogs.offset.ui.tsa_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.ResizeToContents)

        k = 0
        for i in range(self.models.correctiontext.tm.rowCount()):
            # step through each correction mode, fetch its table from the tinySA
            command = 'correction ' + self.models.correctiontext.tm.record(i).value('value') + '\r'
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
                    self.models.correction.insertData(mode=mode, entry=entry, frequency=frequency, dB=dB)
            k += 20

    def upload_correction(self):
        ''''upload the correction table(s) from the config database to the tinySA'''
        device = self.correction_devices[self.dialogs.offset.ui.device.currentIndex()]
        if device.sweeping:
            popUp(offset, "Cannot read from tinySA whilst a scan is running", 'Ok', 'Info')
            return

        self.models.unlimited()
        self.dialogs.offset.ui.progress.setValue(0)

        update_failed = False
        for i in range(self.models.correction.tm.rowCount()):
            record = self.models.correction.tm.record(i)
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
                self.dialogs.offset.ui.progress.setValue(100 * int(i / (self.models.correction.tm.rowCount() - 1)))
        if update_failed:
            popUp(offset, "One or more of the updates failed.", 'Ok', 'Critical')

    def time_path_indicator(self, position):
        with QSignalBlocker(self.ui.vortex):
            self.ui.vortex.setValue(position)

    def start_polar_plot():
        self.polar.set_plot(pattern.ui)

    def save_location_valid(self):
        folder = self.dialogs.settings.ui.save_folder.text()
        if os.path.exists(folder):
            self.dialogs.settings.ui.save_folder.setStyleSheet("")
            return True
        else:
            self.dialogs.settings.ui.save_folder.setStyleSheet(("background-color:red"))
            return False

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
        self.dialogs.file_browse.ui.download.clicked.connect(lambda: self.set_sd_file_save(True))
        self.dialogs.file_browse.ui.saveAll.clicked.connect(lambda: self.set_sd_file_save(False))
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
        # self.ui.run3D.clicked.connect(self.scan)
    
        # Quit
        # self.ui.actionQuit.triggered.connect(app.closeAllWindows)
        # self.ui.actionQuit_2.triggered.connect(exit_handler)
    
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
        self.dialogs.preset_freqs.ui.addPs.clicked.connect(self.models.bands.addRow)
        self.dialogs.preset_freqs.ui.deletePs.clicked.connect(
            lambda: self.models.bands.deleteRow(True))
        
        self.dialogs.preset_freqs.ui.deleteAll.clicked.connect(
            lambda: self.models.bands.deleteRow(False))
        
        # self.dialogs.preset_freqs.ui.freqTable.clicked.connect(
        #     lambda: self.models.bands.tableClicked(self.dialogs.preset_freqs.ui.freqTable))
        
        # self.dialogs.preset_freqs.ui.typeTable.clicked.connect(
        #     lambda: self.models.bandstype.tableClicked(self.dialogs.preset_freqs.ui.typeTable))
        
        self.dialogs.preset_freqs.ui.addPsType.clicked.connect(self.models.bandstype.addRow)
        # self.dialogs.preset_freqs.ui.deletePsType.clicked.connect(self.delete_preset_type)
        # self.dialogs.preset_freqs.ui.clearFilter.clicked.connect(self.models.bands.showAll)
        self.dialogs.preset_freqs.ui.finished.connect(self.set_preferences)  # update db checkboxes table on window close
        self.dialogs.preset_freqs.ui.exportPs.pressed.connect(
            lambda: self.models.bands.exportData(''))
        
        self.dialogs.preset_freqs.ui.importPs.pressed.connect(
            lambda: self.import_data(self.models.bands))
    
        # self.ui.filterBox.currentTextChanged.connect(
        #     lambda: self.models.bandselect.set_filter_to(False, self.ui.filterBox.currentText()))
        
        # self.ui.actionPresets.triggered.connect(dialogPrefs)  # open preferences dialogue when its menu is clicked
        # self.ui.actionSettings.triggered.connect(settings_clicked)
        self.ui.actionCorrection.triggered.connect(self.correction_window)
    
        # Help
        # self.ui.actionAbout_Qt.triggered.connect(about)
        # self.ui.actionAbout_Qt.triggered.connect(app.aboutQt)
    
        # # Waterfall
        # self.ui.waterfall_size.valueChanged.connect(set_wf_height)
        # self.ui.wf_2D.stateChanged.connect(set_wf_format)
    
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
    
        # polar pattern
        self.dialogs.pattern.ui.measure.clicked.connect(self.start_polar_plot)
    
        # correction
        self.dialogs.offset.ui.export_button.clicked.connect(lambda: self.models.correction.exportData(''))
        self.dialogs.offset.ui.import_button.clicked.connect(lambda: self.models.correction.importData(''))
      
        # settings
        self.dialogs.settings.ui.set_folder.clicked.connect(lambda:set_folder(self.dialogs.settings.ui))
    
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
        startF = self.models.bandselect.tm.record(index).value('StartF')
        stopF = self.models.bandselect.tm.record(index).value('StopF')
        if stopF not in (0, '', startF):
            with QSignalBlocker(self.ui.start_freq):
                self.ui.start_freq.setValue(startF / 1e6)
            with QSignalBlocker(self.ui.stop_freq):
                self.ui.stop_freq.setValue(stopF / 1e6)
            self.setStartFreq()
        else:
            centreF = startF / 1e6
            span = int(centreF / 10)  # default span to a tenth of the centre freq
            for i in range(self.models.bandselect.tm.rowCount()):
                type_start = self.models.bandselect.tm.record(i).value('StartF')
                type_stop = self.models.bandselect.tm.record(i).value('StopF')
                if type_stop not in (0, '', startF):
                    span = (type_stop - type_start) / 1e6
                    break
            with QSignalBlocker(self.ui.centre_freq):
                self.ui.centre_freq.setValue(centreF)
            with QSignalBlocker(self.ui.span_freq):
                self.ui.span_freq.setValue(span)
            self.setCentreFreq()
        self.models.numbers.dwm.submit()
        self.set_mixer_highlighting()
    
    def set_mixer_highlighting(self):
        if self.models.bandstype.freq == 0:
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
        self.models.checkboxes.dwm.submit()
        self.models.bands.tm.submitAll()
        self.limits.threshold.line.setValue(self.dialogs.settings.ui.peakThreshold.value())
        self.limits.best.visible(self.dialogs.settings.ui.neg25Line.isChecked())
        self.limits.maximum.visible(self.dialogs.settings.ui.zeroLine.isChecked())
        self.limits.damage.visible(self.dialogs.settings.ui.plus6Line.isChecked())

        if self.ui.presetMarker.isChecked():
            self.set_preset_marker()

    def correction_filter():
        if self.dialogs.offset.ui.filter_box.isChecked():
            sql = 'mode = "' + self.dialogs.offset.ui.correction_mode.currentText() + '"'
            self.models.correction.tm.setFilter(sql)
        else:
            self.models.correction.tm.setFilter('')

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
    '''owns generic database/table operations'''
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
        logging.info(f'set_current_row: row {self.currentRow} clicked')
        
        self.tm = CustomTableModel(db=db_name, ro_columns=limit_edit)
        self.tm.setTable(table_name)

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
            for i in range(0, self.tm.rowCount()):
                self.tm.removeRow(i)
        self.tm.select()
        self.tm.layoutChanged.emit()

    # def deletePsType(self):
    #     record = self.tm.record(self.currentRow)
    #     if record.value('ID') == bandstype.ID:
    #         popUp(presetFreqs, "Cannot delete a preset type that is selected on main screen", 'Ok', 'Critical')
    #         return
    #     bands.set_filter_to(True, record.value('preset'))
    #     bands.deleteRow(False)
    #     if bands.tm.rowCount() == 0:
    #         # now no freq records with the preset type, so can delete & keep db referential integrity
    #         self.deleteRow(True)

    # def tableClicked(self, table):
    #     self.currentRow = table.currentIndex().row()  # the row index from the QModelIndexObject
    #     logging.debug(f'row {self.currentRow} clicked')
    #     if table == presetFreqs.ui.typeTable:
    #         record = self.tm.record(self.currentRow)
    #         bands.set_filter_to(True, record.value('preset'))
    #         bands.unlimited()
    #         presetFreqs.ui.psCount.setValue(bands.tm.rowCount())

    def insertData(self, **data):
        record = self.tm.record()
        logging.debug(f'insertData: record = {record}')
        for key, value in data.items():
            logging.debug(f'insertData: key = {key} value={value}')
            record.setValue(str(key), value)
        self.tm.insertRecord(-1, record)  # -1 means after existing records
        self.tm.select()
        self.tm.layoutChanged.emit()
        # self.dwm.submit()

    # def set_filter_to(self, isPrefsDialog, boxText):
    #     sql = 'preset = "' + boxText + '"'
    #     if isPrefsDialog:
    #         self.tm.setFilter(sql)
    #     else:
    #         sql = 'visible = "1" AND preset = "' + boxText + '"'

    #         self.tm.setFilter(sql)
    #         # QtTSA.band_box.activated.emit(0)

    #         # find and store the ID of the preset type selected in the combobox
    #         bandstype.unlimited()
    #         for index in range(0, bandstype.tm.rowCount()):
    #             record = bandstype.tm.record(index)
    #             if record.value('preset') == boxText:
    #                 bandstype.ID = record.value('ID')
    #                 bandstype.freq = record.value('LO')
    #                 # isMixerMode()
    #                 break

    # def readCSV(self, fileName):
    #     # Build a set of existing (startF, preset) pairs to prevent duplicates
    #     existing = set()
    #     for i in range(self.tm.rowCount()):
    #         rec = self.tm.record(i)
    #         existing.add((str(rec.value('name')), str(rec.value('startF'))))

    #     with open(fileName, "r") as fileInput:
    #         reader = csv.DictReader(fileInput)
    #         inserted = 0
    #         skipped = 0
    #         for row in reader:
    #             logging.debug(f'readCSV(): row = {row}')
    #             record = self.tm.record()
    #             for key, value in row.items():
    #                 if key == 'preset':
    #                     value = bandstype.fetch_ID('preset', value)
    #                 if key == 'colour':
    #                     value = colours.fetch_ID('colour', value)
    #                 if key == 'value':
    #                     value = int(eval(value))
    #                 if key == 'Frequency':  # to match RF mic CSV files
    #                     key = 'startF'
    #                     value = str(float(value) / 1e3)
    #                 if key != 'ID': # ID is the table primary key and is auto-populated
    #                     record.setValue(str(key), value)
    #             if record.value('value') not in (0, 1): # because it's not present in RF mic CSV files
    #                 record.setValue('value', 1)
    #             if record.value('preset') == '': # preset missing so use current preferences filterbox text
    #                 record.setValue('preset', bandstype.fetch_ID('preset', presetFreqs.ui.filterBox.currentText()))

    #             # Duplicate check: skip if this startF + preset already exists
    #             key_tuple = (str(record.value('name')), str(record.value('startF')))
            
    #             if key_tuple in existing:
    #                 logging.info(f'readCSV(): skipping duplicate entry name={record.value("name")}')
    #                 skipped += 1
    #                 continue

    #             existing.add(key_tuple)
    #             self.tm.insertRecord(-1, record)
    #             inserted += 1

    #         self.tm.select()
    #         self.tm.layoutChanged.emit()
    #         if skipped:
    #             logging.info(f'readCSV(): inserted {inserted} rows, skipped {skipped} duplicates')

    #     message = 'Inserted ' + str(inserted) + ' rows, skipped ' + str(skipped) + ' duplicates'
    #     popUp(QtTSA, message, 'Ok', 'Info')
    #     # self.dwm.submit()
    
    def import_records(self, records):
        existing = set()
        for i in range(self.tm.rowCount()):
            rec = self.tm.record(i)
            existing.add((str(rec.value('name')), str(rec.value('startF'))))
        inserted = 0
        skipped = 0
        for record in records:
            key_tuple = (str(record.value('name')), str(record.value('startF'))        )
            if key_tuple in existing:
                logging.info(f'import_records(): skipping duplicate entry name={record.value("name")}')
                skipped += 1
                continue
            existing.add(key_tuple)
            self.tm.insertRecord(-1, record)
            inserted += 1
        self.tm.select()
        self.tm.layoutChanged.emit()
        return inserted, skipped

    def fetch_ID(self, field, lookup_value):
        ''''find the relation table ID from an aliased field name'''
        for i in range(0, self.tm.rowCount()):
            record = self.tm.record(i).value(field)
            if record == lookup_value:
                ID = self.tm.record(i).value('ID')
                return ID
        return 1

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
          
    # def importData(self, filename=''):
    #     if filename == '':
    #         filename = QFileDialog.getOpenFileName(caption="Open File", filter="Comma Separated Values (*.csv)")[0]
    #     logging.info(f'importing data from {filename}')
    #     if filename != '':
    #         self.readCSV(filename)

    # def map_table_to_widget(self, model_name, mapping, ui, dialogs):
    #     ''''maps the widget fields to the appropriate database table fields, using the mapping table'''

    #     # filter the mapping table to show just values for this modelName
    #     mapping.tm.setFilter('model = "' + model_name + '"')
    #     namespace = {'ui': ui, 'dialogs': dialogs}

    #     for index in range(mapping.tm.rowCount()):
    #         # the mapping table 'gui' column determines which ui field is mapped, as '...ui.field'
    #         gui = mapping.tm.record(index).value('gui')
    #         column = mapping.tm.record(index).value('column')
    #         widget = eval(gui, {}, namespace)
    #         self.dwm.addMapping(widget, int(column))

    def unlimited(self):  # remove 256 row limit for QSql Query
        while self.tm.canFetchMore():
            self.tm.fetchMore()

    # def showAll(self):
    #     presetFreqs.ui.typeTable.clearSelection()
    #     self.tm.setFilter('')
    #     self.unlimited()
    #     presetFreqs.ui.psCount.setValue(bands.tm.rowCount())

    def update_row(self, row, **data):
        record = self.tm.record(row)
        for key, value in data.items():
            logging.debug(f'update_row: key = {key} value={value}')
            record.setValue(str(key), value)
        self.tm.setRecord(row, record)
        # self.updateModel()
        self.tm.select()
        self.tm.layoutChanged.emit()

    # def read_tables(self):
    #     ''''read the correction tables from the tinySA and display in a table widget'''
    #     device = self.correction_devices[self.dialogs.offset.ui.device.currentIndex()]
    #     if device.sweeping:
    #         popUp(offset, "Cannot read from tinySA whilst a scan is running", 'Ok', 'Info')
    #         return
    #     self.unlimited()
    #     write_config = QMessageBox.StandardButton.Ok
    #     if self.tm.rowCount() > 0 and offset.ui.save_box.isChecked():
    #         message = ('OK to over-write config database table with\rcorrection data from tinySA?')
    #         write_config = popUp(offset, message, 'OkC', 'Question')
    #         if write_config == QMessageBox.StandardButton.Ok:
    #             correction.deleteRow(single=False)
    #     offset.ui.tsa_table.clear()
    #     offset.ui.tsa_table.setRowCount(200)
    #     offset.ui.tsa_table.setColumnCount(4)
    #     offset.ui.tsa_table.setHorizontalHeaderLabels(['mode', 'entry', 'frequency', 'dB'])
    #     offset.ui.tsa_table.show()
    #     offset.ui.tsa_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.ResizeToContents)

    #     k = 0
    #     for i in range(correctiontext.tm.rowCount()):
    #         # step through each correction mode, fetch its table from the tinySA
    #         command = 'correction ' + correctiontext.tm.record(i).value('value') + '\r'
    #         data = device.serialQuery(command)
    #         mode_table = data.splitlines()[1:]  # make a list of the rows, discard the mode header
    #         mode_rows = [row.split(' ')[1:] for row in mode_table]  # split each row into a list, discard first col
    #         for j in range(20):
    #             # step through each row of the current mode and write the fields to the tablewidget 'tsa_table'
    #             mode = str(mode_rows[j][0])
    #             entry = str(mode_rows[j][1])
    #             frequency = str(mode_rows[j][2])
    #             dB = str(mode_rows[j][3])
    #             offset.ui.tsa_table.setItem(k+j, 0, QTableWidgetItem(mode))
    #             offset.ui.tsa_table.setItem(k+j, 1, QTableWidgetItem(entry))
    #             offset.ui.tsa_table.setItem(k+j, 2, QTableWidgetItem(frequency))
    #             offset.ui.tsa_table.setItem(k+j, 3, QTableWidgetItem(dB))

    #             if offset.ui.save_box.isChecked() and write_config == QMessageBox.StandardButton.Ok:
    #                 self.insertData(mode=mode, entry=entry, frequency=frequency, dB=dB)
    #         k += 20

    # def upload_correction(self):
    #     ''''upload the correction table(s) from the config database to the tinySA'''
    #     device = self.correction_devices[self.dialogs.offset.ui.device.currentIndex()]
    #     if device.sweeping:
    #         popUp(offset, "Cannot read from tinySA whilst a scan is running", 'Ok', 'Info')
    #         return
    #     self.unlimited()
    #     offset.ui.progress.setValue(0)
    #     update_failed = False
    #     for i in range(self.tm.rowCount()):
    #         record = self.tm.record(i)
    #         mode = str(record.value('mode')) + ' '
    #         entry = str(record.value('entry')) + ' '
    #         frequency = str(record.value('frequency')) + ' '
    #         dB = str(record.value('dB'))
    #         command = 'correction ' + mode + entry + frequency + dB + '\r'
    #         response = device.serialQuery(command)
    #         logging.debug(f'upload_correction(): {response}')
    #         if response != 'updated ' + entry + 'to ' + frequency + dB:
    #             # the error trapping on the tinySA is not comprehensive so this may not work for all scenarios
    #             update_failed = True
    #             logging.info(f'Update failure: {command}')
    #         else:
    #             offset.ui.progress.setValue(100 * int(i / (self.tm.rowCount() - 1)))
    #     if update_failed:
    #         popUp(offset, "One or more of the updates failed.", 'Ok', 'Critical')
 
###############################################################################
# respond to GUI signals

# def band_changed():
#     index = QtTSA.band_box.currentIndex()
#     startF = bandselect.tm.record(index).value('StartF')
#     stopF = bandselect.tm.record(index).value('StopF')
#     if stopF not in (0, '', startF):
#         with QSignalBlocker(QtTSA.start_freq):
#             QtTSA.start_freq.setValue(startF / 1e6)
#         with QSignalBlocker(QtTSA.stop_freq):
#             QtTSA.stop_freq.setValue(stopF / 1e6)
#         tinySA.setStartFreq()
#     else:
#         centreF = startF / 1e6
#         span = int(centreF / 10)  # default span to a tenth of the centre freq
#         for i in range(bandselect.tm.rowCount()):
#             type_start = bandselect.tm.record(i).value('StartF')
#             type_stop = bandselect.tm.record(i).value('StopF')
#             if type_stop not in (0, '', startF):
#                 span = (type_stop - type_start) / 1e6
#                 break
#         with QSignalBlocker(QtTSA.centre_freq):
#             QtTSA.centre_freq.setValue(centreF)
#         with QSignalBlocker(QtTSA.span_freq):
#             QtTSA.span_freq.setValue(span)
#         tinySA.setCentreFreq()
#     numbers.dwm.submit()

# def addFixed():
#     title = "New fixed frequency Marker"
#     message = "Enter a name for the fixed Marker"
#     fixedMkr, ok = QInputDialog.getText(None, title, message, QLineEdit.Normal, "")
#     controller.models.bands.insertData(name=fixedMkr, preset=12, startF=f'{int(tinySA.s0.trace.m0.line.value())}',
#                      stopF=0, visible=1, colour=colours.fetch_ID('colour', 'orange'))  # preset type 12 = fixed Marker


# def pointsChanged():
#     if QtTSA.points_auto.isChecked():
#         QtTSA.points_box.setEnabled(False)
#         QtTSA.rbw_box.setEnabled(True)
#     else:
#         QtTSA.points_box.setEnabled(True)
#     tinySA.setting_change()


# def setPreferences():  # called at startup and when the preferences window is closed
#     checkboxes.dwm.submit()
#     bands.tm.submitAll()
#     threshold.line.setValue(settings.ui.peakThreshold.value())
#     best.visible(settings.ui.neg25Line.isChecked())
#     maximum.visible(settings.ui.zeroLine.isChecked())
#     damage.visible(settings.ui.plus6Line.isChecked())

#     if QtTSA.presetMarker.isChecked():
#         tinySA.set_preset_marker()


# def dialogPrefs():  # called by clicking on the setup > preferences menu
#     presetFreqs.ui.show()
#     presetFreqs.ui.psCount.setValue(bands.tm.rowCount())


# def about():
#     message = ('TinySA Ultra GUI programme using Qt6 PySide6\
#                \nAuthor: Ian Jefferson G4IXT\n\nVersion: {} \nConfig: {}'
#                .format(app.applicationVersion(), config.databaseName()))
#     popUp(QtTSA, message, 'Ok', 'Info')


def clickEvent():
    logging.info('clickEvent')


# def correction_filter():
#     if offset.ui.filter_box.isChecked():
#         sql = 'mode = "' + offset.ui.correction_mode.currentText() + '"'
#         correction.tm.setFilter(sql)
#     else:
#         correction.tm.setFilter('')


##############################################################################
# other methods

# def set_folder(ui_name):
#     folder = QFileDialog.getExistingDirectory()
#     ui_name.save_folder.setText(folder)
#     save_location_valid()
  
def save_sweep(folder, file_name, frequencies, readings, ser_num):
    array = np.insert(readings, 0, frequencies, axis=0)  # insert the measurement freqs at the top of the readings array
    dBm = np.transpose(np.round(array, decimals=2))  # transpose columns and rows
    np.savetxt(file_name, dBm, delimiter=',', fmt='%.2f')

# def set_sd_file_save(single=True):
#     device = tinySA.dev_ref[filebrowse.ui.device.currentIndex()]
#     folder = QFileDialog.getExistingDirectory(caption="Select folder to save SD card files")
#     if single:
#         file_name = filebrowse.ui.listWidget.currentItem().text()  # the file selected in the list widget
#     else:
#         file_name = None
#     saver = Worker(sd_file_save, device, file_name, folder, single)
#     threadpool.start(saver)  # workers deleted when thread ends
    
def sd_file_save(device, file_name, folder, single):
    signals = WorkerSignals()
    signals.progress.connect(sd_save_progress)
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

def sd_save_progress(progress):
    filebrowse.ui.saveProgress.setValue(progress)

def locate_db(dbName, app_name):
    # 1. check if a personal database file exists already
    # personalDir = platformdirs.user_config_dir(appname=app.applicationName(), appauthor=False)
    personalDir = platformdirs.user_config_dir(appname=app_name, appauthor=False)
    if not os.path.exists(personalDir):
        os.mkdir(personalDir)
    if os.path.isfile(os.path.join(personalDir, dbName)):
        logging.info(f'Database {dbName} found at {personalDir}')
        return personalDir

    # 2. if not, then check if a global database file exists
    globalDir = platformdirs.site_config_dir(appname=app_name, appauthor=False)
    if os.path.isfile(os.path.join(globalDir, dbName)):
        shutil.copy(os.path.join(globalDir, dbName), personalDir)
        logging.info(f'Database {dbName} copied from {globalDir} to {personalDir}')
        return personalDir

    # 3. if not, check if database file exists in the app directory
    file_path = resource_path(dbName)
    if os.path.isfile(file_path):
        shutil.copy(file_path, personalDir)
        logging.info(f'{dbName} copied from {file_path} to {personalDir}')
        return personalDir

    # 4. If not, then look in current working folder & where the python file is stored/linked from
    workingDirs = [os.path.dirname(__file__), os.path.dirname(os.path.realpath(__file__)), os.getcwd()]
    for directory in workingDirs:
        if os.path.isfile(os.path.join(directory, dbName)):
            shutil.copy(os.path.join(directory, dbName), personalDir)
            logging.info(f'{dbName} copied from {directory} to {personalDir}')
            return personalDir
    raise FileNotFoundError("Unable to find the database {self.dbName}")


def connect(dbFile, con, target, app_name, parent_window):
    db = QSqlDatabase.addDatabase('QSQLITE', connectionName=con)
    dbPath = locate_db(dbFile, app_name)
    if QtCore.QFile.exists(os.path.join(dbPath, dbFile)):
        db.setDatabaseName(os.path.join(dbPath, dbFile))
        db.open()
        logging.debug(f'{dbFile} open: {db.isOpen()}  Connection = "{db.connectionName()}"')
        logging.debug(f'tables available = {db.tables()}')
        checkVersion(db, target, dbFile, app_name, parent_window)  # check the actual database version = target version
    else:
        logging.info('Database file {dbPath}{dbFile} is missing')
        popUp(QtTSA, 'Database file is missing', 'Ok', 'Critical')
        return
    return db


def disconnect(db):
    db.close()
    logging.debug(f'Database {db.databaseName()} open: {db.isOpen()}')
    QSqlDatabase.removeDatabase(db.databaseName())


def checkVersion(db, target, dbFile, app_name, parent_window):
    existing = fetchVersion(db)
    logging.info(f'Database version is {existing}, expected {target}')
    if existing != target:
        message = "This version of QtTinySA needs database version " + str(target) + ".\n\n" + \
                "Database " + db.databaseName() + "\nversion " + str(existing) + \
                " may not be compatible.\n" + \
                "\nClicking OK will replace it with version " + str(target) + \
                " and will reset some settings.ui."
        replace = popUp(parent_window, message, 'OkC', 'Question')
        if replace == QMessageBox.StandardButton.Ok:
            impex = ModelView('frequencies', db, ())
            impex.tm.select()
            impex.unlimited()
            personalDir = platformdirs.user_config_dir(appname=app_name, appauthor=False)
            fileName = personalDir + "/frequencies_" + str(target) + ".csv"
            impex.exportData(fileName)
            logging.info(f'Renaming file {db.databaseName()} to {db.databaseName()}.{str(existing)}')
            disconnect(db)
            os.rename(db.databaseName(), db.databaseName() + '.' + str(existing))

            locate_db(dbFile)  # this ought to return the same path as when it was run earlier in connect()
            db.open()  # the database connection has not changed, only the file, so can re-open it with the new file
            found = fetchVersion(db)
            logging.info(f'Found new database version {found}')
            if found != target:
                message = "Found new database version " + str(found) + "\nbut expected version " + str(target)
                restore = popUp(QtTSA, message, 'Ok', 'Info')
            message = "Restore your previous preset frequencies to the new database?"
            restore = popUp(QtTSA, message, 'OkC', 'Question')
            if restore == QMessageBox.StandardButton.Ok:
                impex.tm.select()
                impex.unlimited()
                impex.deleteRow(False)
                logging.info(f'Deleting records from frequencies table of database version {found}')
                impex.tm.submit()
                impex.importData(fileName)

def fetchVersion(db):
    query = QSqlQuery(db)
    query.exec("PRAGMA user_version;")  # execute PRAGMA command to fetch the user-defined version number
    query.next()  # advances to the result row, and query.value(0) retrieves the user version.
    version = query.value(0)
    query.clear()
    return version

# def exit_handler():
#     '''Save gui field vals, marker freqs and checkbox states. Close usb ports, config database and windows'''
#     usbCheck.stop()
#     tinySA.mkr_update_timer.stop()
#     if usbInstr.is_scanning:
#         usbInstr.stop(restart=False)
#     if len(usbInstr.ports) != 0:
#         tinySA.marker_save()
#         usbInstr.closePort()
#     checkboxes.dwm.submit()
#     numbers.dwm.submit()
#     disconnect(config)
#     app.closeAllWindows()
#     logging.info('QtTinySA Closed')

def popUp(window, message, button, icon):
    if window is None:
        window = QtTSA
    icons = {'Warn': QMessageBox.Icon.Warning, 'Info': QMessageBox.Icon.Information,
             'Critical': QMessageBox.Icon.Critical, 'Question': QMessageBox.Icon.Question}
    buttons = {'Ok': QMessageBox.StandardButton.Ok, 'Cancel': QMessageBox.StandardButton.Cancel,
               'OkC': QMessageBox.StandardButton.Ok | QMessageBox.StandardButton.Cancel}
    msg = QMessageBox(parent=(window))
    msg.setIcon(icons.get(icon))
    msg.setText(message)
    msg.setStandardButtons(buttons.get(button))
    return msg.exec()

# def isMixerMode():
#     if bandstype.freq == 0:
#         QtTSA.mixerMode.setVisible(False)
#         QtTSA.start_freq.setStyleSheet('background-color:None')
#         QtTSA.stop_freq.setStyleSheet('background-color:None')
#         QtTSA.centre_freq.setStyleSheet('background-color:None')
#         QtTSA.start_freq.setMaximum(tinySA.maxF)
#         QtTSA.centre_freq.setMaximum(tinySA.maxF)
#         QtTSA.stop_freq.setMaximum(tinySA.maxF)
#     else:
#         QtTSA.mixerMode.setVisible(True)
#         QtTSA.start_freq.setStyleSheet('background-color:lightGreen')
#         QtTSA.stop_freq.setStyleSheet('background-color:lightGreen')
#         QtTSA.centre_freq.setStyleSheet('background-color:lightGreen')
#         QtTSA.start_freq.setMaximum(100000)
#         QtTSA.centre_freq.setMaximum(100000)
#         QtTSA.stop_freq.setMaximum(100000)
    
# def set_wf_format():  # called when wf_2D checkbox state changes
#     wf_size = QtTSA.waterfall_size.value()
#     if QtTSA.wf_2D.isChecked():
#         QtTSA.plot_3D.hide()
#         QtTSA.waterfall.show()
#         # set the display_frame stretch, which is 'units' of total vertical space rows can expand in
#         QtTSA.display_frame.layout().setRowStretch(0, 9 - wf_size)  # index 0 = row 0 = graphWidget
#         QtTSA.display_frame.layout().setRowStretch(1, 0)  # index 1 = row 1 = plot_3D
#         QtTSA.display_frame.layout().setRowStretch(2, wf_size)  # index 2 = row 2 = waterfall
#     else:
#         QtTSA.plot_3D.show()
#         QtTSA.waterfall.hide()
#         QtTSA.display_frame.layout().setRowStretch(0, 9 - wf_size)
#         QtTSA.display_frame.layout().setRowStretch(1, wf_size)
#         QtTSA.display_frame.layout().setRowStretch(2, 0)

# def set_wf_height():  # called when wf_size spinbox value changes
#     '''changing height sends the widget Resize signal; this is intercepted by the ResizeEventFilter
#        in the graphs.py module which then calls on_widget_resized() to set the 3D aspect ratio'''
#     wf_size = QtTSA.waterfall_size.value()
#     if wf_size == 9:
#         QtTSA.graphWidget.hide()
#         set_wf_format()
#     else:
#         QtTSA.graphWidget.show()
#         set_wf_format()
#     if wf_size == 0:
#         QtTSA.plot_3D.hide()
#         QtTSA.waterfall.hide()
#         QtTSA.wf_2D.setEnabled(False)
#     else:
#         set_wf_format()
#         QtTSA.wf_2D.setEnabled(True)
    
# def startPolarPlot():
#     tinySA.polar.set_plot(pattern.ui)

# def settings_clicked():
#     save_location_valid()
#     settings.ui.show()
    
# def save_location_valid():
#     folder = settings.ui.save_folder.text()
#     if os.path.exists(folder):
#         settings.ui.save_folder.setStyleSheet("")
#         return True
#     else:
#         settings.ui.save_folder.setStyleSheet(("background-color:red"))
#         return False

    # All names assigned below were originally module-level globals that dozens of other
    # functions/classes throughout this file (and devices.py) reference directly. Declaring
    # them global here means main() still writes to the module namespace, exactly as the
    # original unguarded module-level code did -- everything else keeps working unmodified.
    # global app, auto_run_timer, bands, bandselect, bandstype, best, c_header
    # global checkboxes, colHeader, colours, config, correction, correctiontext, damage, fading
    # global filebrowse, highF, loader, lowF, maps, markertext, maximum
    # global numbers, offset, pattern, phasenoise, presetFreqs, presetmarker, presets, rbwtext
    # global reference, settings, threshold, tinySA, traceHeader, tracecolours, tracesettings, tracetext
    # global usbCheck

    # create QApplication for the GUI
    # app = QApplication.instance()
    # if not app:
    #     app = QApplication([])
    # app.setApplicationName('QtTinySA')
    # app.setApplicationVersion(' v2.1.x')

    # # pyqtgraph custom exporters
    # WWBExporter.register()
    # WSMExporter.register()

    # loader = CustomLoader()
    # QtTSA = loader.load(resource_path("spectrum.ui"), None)

    # # dialogs = Dialogs()

    # usbInstr = USBdevice()
    # tinySA = Analyser(ui=QtTSA, hardware=usbInstr, dialogs=dialogs)

    # presetFreqs = CustomDialogue(resource_path('bands.ui'))
    # settings = CustomDialogue(resource_path('settings.ui'))
    # filebrowse = CustomDialogue(resource_path('filebrowse.ui'))
    # phasenoise = CustomDialogue(resource_path('phasenoise.ui'))
    # fading = CustomDialogue(resource_path('fading.ui'))
    # pattern = CustomDialogue(resource_path('pattern.ui'))
    # offset = CustomDialogue(resource_path('offset.ui'))

def main():

    app = QApplication.instance()
    if not app:
        app = QApplication([])
    app.setApplicationName('QtTinySA')
    app.setApplicationVersion(' v2.1.x')

    # pyqtgraph custom exporters
    WWBExporter.register()
    WSMExporter.register()

    controller = ApplicationController(app)

    # Markers
    # multiplot = pyqtgraph.GraphicsLayout()  # for plotting marker signal level over time
    # fading.ui.grView.setCentralItem(multiplot)

    ###############################################################################
    # # GUI settings

    # # pyqtgraph settings for spectrum display
    # QtTSA.graphWidget.setYRange(-112, -20)
    # QtTSA.graphWidget.setDefaultPadding(padding=0.015) # was 0.005
    # QtTSA.graphWidget.showGrid(x=True, y=True)
    # QtTSA.graphWidget.setLabel('bottom', '', units='Hz')

    # # # pyqtgraph settings for waterfall and histogram display
    # # QtTSA.waterfall.setDefaultPadding(padding=0.005)
    # QtTSA.waterfall.setDefaultPadding(padding=0.016)
    # QtTSA.waterfall.getPlotItem().hideAxis('bottom')
    # QtTSA.waterfall.setLabel('left', '.', **{'color': '#FFF', 'font-size': '2pt'})
    # # QtTSA.waterfall.getPlotItem().hideAxis('left')
    # QtTSA.waterfall.invertY(True)

    # QtTSA.histogram.setDefaultPadding(padding=0)
    # QtTSA.histogram.plotItem.invertY(True)
    # QtTSA.histogram.getPlotItem().hideAxis('bottom')
    # QtTSA.histogram.getPlotItem().hideAxis('left')

    # # widget settings for Phase Noise
    # controller.dialogs.phasenoise.ui.plotWidget.setYRange(-120, -40)
    # controller.dialogs.phasenoise.ui.plotWidget.plotItem.showGrid(x=True, y=True, alpha=0.5)
    # controller.dialogs.phasenoise.ui.plotWidget.plotItem.setLogMode(x=True)
    # controller.dialogs.phasenoise.ui.plotWidget.setLabel('bottom', 'Offset Frequency', units='Hz')
    # controller.dialogs.phasenoise.ui.plotWidget.setLabel('left', 'Phase Noise', units='dBc/Hz')


    ###############################################################################
    # set up the application
    logging.info(f'{app.applicationName()}{app.applicationVersion()}')

    # # Database and models for configuration settings
    # config = connect("QtTSAprefs.db", "settings", 210, app.applicationName(), QtTSA)  # third parameter = db version

    # # field mapping of the checkboxes and numbers database tables, for storing startup configuration
    # mapping = ModelView('mapping', config, ())
    # mapping.tm.select()

    # # populate the preset frequencies relational table in the presetFreqs window
    # bands = ModelView('frequencies', config, ())
    # bands.tm.setSort(3, QtCore.Qt.SortOrder.AscendingOrder)
    # bands.tm.setHeaderData(5, QtCore.Qt.Orientation.Horizontal, "visible")
    # bands.tm.setEditStrategy(QSqlRelationalTableModel.EditStrategy.OnRowChange)
    # bands.tm.setRelation(2, QSqlRelation("freqtype", "ID", "preset"))  # set "type" column to a freq type combo box
    # bands.tm.setRelation(5, QSqlRelation("boolean", "ID", "value"))  # set "view" column to a True/False combo box
    # bands.tm.setRelation(6, QSqlRelation("SVGColour", "ID","colour"))  # set "marker" column to a colours combo box
    # presets = QSqlRelationalDelegate(presetFreqs.ui.freqTable)
    # presetFreqs.ui.freqTable.setItemDelegate(presets)
    # colHeader = presetFreqs.ui.freqTable.horizontalHeader()
    # colHeader.setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.ResizeToContents)
    # bands.tm.select()

    # # populate the preset Types table in the preset frequencies window
    # bandstype = ModelView('freqtype', config, ())
    # bandstype.tm.select()
    # presetFreqs.ui.typeTable.setModel(bandstype.tm)
    # presetFreqs.ui.typeTable.hideColumn(0)  # hide primary key so user can't change it

    # # populate the correction values table in the correction window
    # correction = ModelView('correction', config, (0, 1, 2, 3))
    # c_header = offset.ui.c_table.horizontalHeader()
    # c_header.setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.ResizeToContents)
    # correction.tm.select()
    # offset.ui.c_table.setModel(correction.tm)
    # offset.ui.c_table.hideColumn(0)

    # # to lookup the preset bands and markers colours because can't get the relationships to work
    # colours = ModelView('SVGColour', config, ())
    # colours.tm.select()

    # # for the main screen preset markers, which need different filtering to the preset frequencies window
    # presetmarker = ModelView('frequencies', config, ())
    # presetmarker.tm.setRelation(6, QSqlRelation('SVGColour', 'ID', 'colour'))
    # presetmarker.tm.setRelation(2, QSqlRelation('freqtype', 'ID', 'preset'))
    # presetmarker.tm.setSort(3, QtCore.Qt.SortOrder.AscendingOrder)
    # presetmarker.tm.select()

    # # populate the ui band selection combo box; needs different filter to the main and preset frequencies window
    # bandselect = ModelView('frequencies', config, ())
    # bandselect.tm.setRelation(2, QSqlRelation('freqtype', 'ID', 'preset'))
    # bandselect.tm.setRelation(5, QSqlRelation('boolean', 'ID', 'value'))
    # bandselect.tm.setRelation(6, QSqlRelation('SVGColour', 'ID', 'colour'))
    # bandselect.tm.setSort(3, QtCore.Qt.SortOrder.AscendingOrder)
    # QtTSA.band_box.setModel(bandselect.tm)
    # QtTSA.band_box.setModelColumn(1)
    # bandselect.tm.select()

    # # populate the preset bands and markers dialogue and ui filter combo boxes
    # QtTSA.filterBox.setModel(bandstype.tm)
    # QtTSA.filterBox.setModelColumn(1)

    # # connect the preset frequencies window table widget to the data model
    # presetFreqs.ui.freqTable.setModel(bands.tm)
    # presetFreqs.ui.freqTable.hideColumn(0)  # ID

    # # connect the settings window trace colours widget to the data model
    # tracecolours = ModelView('trace', config, (0, 1))
    # tracecolours.tm.setRelation(2, QSqlRelation('SVGColour', 'ID', 'colour'))
    # tracecolours.tm.setEditStrategy(QSqlRelationalTableModel.EditStrategy.OnFieldChange)
    # tracesettings = QSqlRelationalDelegate(settings.ui.colourTable)
    # settings.ui.colourTable.setItemDelegate(tracesettings)
    # traceHeader = settings.ui.colourTable.horizontalHeader()
    # traceHeader.setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.ResizeToContents)
    # settings.ui.colourTable.setModel(tracecolours.tm)
    # settings.ui.colourTable.hideColumn(0)  # ID
    # settings.ui.colourTable.verticalHeader().setVisible(True)
    # tracecolours.tm.select()

    # settingstext = ModelView('settings', config, ())

    # # Map data tables to presets/settings/GUI fields - must be here & in this order
    # checkboxes = ModelView('checkboxes', config, ())
    # checkboxes.createMapper()
    # checkboxes.map_table_to_widget('checkboxes', mapping, controller.ui, controller.dialogs)  # uses mapping table from database
    # checkboxes.tm.select()
    # checkboxes.dwm.setCurrentIndex(0)  # 0 = (last used) default settings

    # # populate the spur combo box
    # QtTSA.spur_box.addItems(['off', 'on', 'auto'])
    # QtTSA.spur_box.setCurrentIndex(2)

    # # populate the rbw combobox
    # rbwtext = ModelView('combo', config, ())
    # rbwtext.tm.setFilter('type = "rbw"')
    # QtTSA.rbw_box.setModel(rbwtext.tm)
    # rbwtext.tm.select()

    # # populate the trace comboboxes
    # tracetext = ModelView('combo', config, ())
    # tracetext.tm.setFilter('type = "trace"')
    # QtTSA.t1_type.setModel(tracetext.tm)
    # QtTSA.t2_type.setModel(tracetext.tm)
    # QtTSA.t3_type.setModel(tracetext.tm)
    # QtTSA.t4_type.setModel(tracetext.tm)
    # tracetext.tm.select()

    # # populate the marker comboboxes
    # markertext = ModelView('combo', config, ())
    # markertext.tm.setFilter('type = "marker"')
    # QtTSA.m1_type.setModel(markertext.tm)
    # QtTSA.m2_type.setModel(markertext.tm)
    # QtTSA.m3_type.setModel(markertext.tm)
    # QtTSA.m4_type.setModel(markertext.tm)
    # markertext.tm.select()

    # # populate the correction comboboxes
    # correctiontext = ModelView('combo', config, ())
    # correctiontext.tm.setFilter('type = "correction"')
    # offset.ui.correction_mode.setModel(correctiontext.tm)
    # correctiontext.tm.select()

    # The models for saving number, marker and trace settings

    # # Map data tables to presets/settings/GUI fields - must be here & in this order
    # numbers = ModelView('numbers', config, ())
    # numbers.createMapper()
    # numbers.map_table_to_widget('numbers', mapping, controller.ui, controller.dialogs)  # mapping table from database
    # numbers.tm.select()
    # numbers.dwm.setCurrentIndex(0)

    controller.analyser.setGraphs()
    controller.analyser.setGUI()
    controller.analyser.setSignals()

    # usbCheck = QtCore.QTimer()
    # usbCheck.timeout.connect(usbInstr.probe)
    # usbCheck.start(500)

    controller.ui.show()
    controller.ui.setWindowTitle(app.applicationName() + app.applicationVersion())
    controller.ui.setWindowIcon(QIcon(os.path.join(basedir, 'tinySAsmall.png')))  


    ###############################################################################
    # run the application until the user closes it

    # if settings.ui.auto_run.isChecked():
    #     auto_run_timer = QtCore.QTimer()
    #     auto_run_timer.timeout.connect(run_on_start)
    #     auto_run_timer.start(4000)

    try:
        app.exec()
    finally:
        # exit_handler()  # close cleanly
        controller.shutdown()
        app.quit()

if __name__ == "__main__":
    main()    