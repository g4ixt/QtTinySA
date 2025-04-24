import csv
import itertools

import numpy as np

from PyQt5.QtWidgets import QMessageBox

from pyqtgraph import ErrorBarItem, PlotItem
from pyqtgraph.exporters import CSVExporter
from pyqtgraph.parametertree import Parameter


class QtTinySAExportException(Exception):
    """Custom exception for custom pyqtgraph exporters."""


class WWBExporter(CSVExporter):
    Name = "CSV for WWB (Shure)"

    def __init__(self, item):
        CSVExporter.__init__(self, item)
        self.params = Parameter.create(name='params', type='group', children=[
            {'name': 'trace', 'title': 'Export trace', 'type': 'list',
             'value': 1, 'limits': [1, 2, 3, 4]}
        ])
        self.index_counter = itertools.count(start=0)
        self.header = []
        self.data = []

    def _exportPlotDataItem(self, plotDataItem) -> None:
        if hasattr(plotDataItem, 'getOriginalDataset'):
            # try to access unmapped, unprocessed data
            cd = plotDataItem.getOriginalDataset()
        else:
            # fall back to earlier access method
            cd = plotDataItem.getData()
        if cd[0] is None:
            # no data found, break out...
            return None
        self.data.append(cd)
        return None

    def export(self, fileName=None):
        if not isinstance(self.item, PlotItem):
            raise TypeError("Must have a PlotItem selected for CSV export.")
        if fileName is None:
            self.fileSaveDialog(filter=["*.csv", "*.tsv"])
            return
        for item in self.item.items:
            if isinstance(item, ErrorBarItem):
                self._exportErrorBarItem(item)
            elif hasattr(item, 'implements') and item.implements('plotData'):
                self._exportPlotDataItem(item)
        try:
            # we want to flatten the nested arrays of data into columns
            columns = [column for dataset in self.data for column in dataset]
            if len(columns) == 0:
                raise QtTinySAExportException(
                    'No column data to export / empty graph!')
            if len(columns) <= self.params['trace'] * 2:
                raise QtTinySAExportException(
                    "Missing column data for selected trace - can't export!")
            row_x = 0
            row_y = 1
            if self.params['trace'] == 1:
                row_x = 0
                row_y = 1
            elif self.params['trace'] == 2:
                row_x = 2
                row_y = 3
            elif self.params['trace'] == 3:
                row_x = 4
                row_y = 5
            elif self.params['trace'] == 4:
                row_x = 6
                row_y = 7
            with open(fileName, 'w', newline='', encoding='utf-8') as csvfile:
                writer = csv.writer(csvfile, delimiter=',',
                                    quoting=csv.QUOTE_MINIMAL)
                for row in itertools.zip_longest(*columns, fillvalue=""):
                    if isinstance(row[row_x], str):
                        x = row[row_x]
                    else:
                        x = np.format_float_positional(row[row_x] / 1000000,
                                                       precision=3,
                                                       unique=True,
                                                       min_digits=3,
                                                       fractional=True)
                    if isinstance(row[row_y], str):
                        y = row[row_y]
                    else:
                        y = np.format_float_positional(row[row_y],
                                                       precision=4,
                                                       unique=True,
                                                       min_digits=4,
                                                       fractional=True)
                    row_to_write = [x, y]
                    writer.writerow(row_to_write)
        except QtTinySAExportException as e:
            error_dialog = QMessageBox()
            error_dialog.setIcon(QMessageBox.Icon.Critical)
            error_dialog.setText(str(e))
            error_dialog.setWindowTitle('Error')
            error_dialog.exec()
        self.header.clear()
        self.data.clear()


class WSMExporter(CSVExporter):
    Name = "CSV for WSM (Sennheiser)"

    def __init__(self, item):
        CSVExporter.__init__(self, item)
        self.params = Parameter.create(name='params', type='group', children=[
            {'name': 'trace', 'title': 'Export trace', 'type': 'list',
             'value': 1, 'limits': [1, 2, 3, 4]}
        ])
        self.index_counter = itertools.count(start=0)
        self.header = []
        self.data = []

    def _exportPlotDataItem(self, plotDataItem) -> None:
        if hasattr(plotDataItem, 'getOriginalDataset'):
            # try to access unmapped, unprocessed data
            cd = plotDataItem.getOriginalDataset()
        else:
            # fall back to earlier access method
            cd = plotDataItem.getData()
        if cd[0] is None:
            # no data found, break out...
            return None
        self.data.append(cd)
        return None

    def export(self, fileName=None):
        if not isinstance(self.item, PlotItem):
            raise TypeError("Must have a PlotItem selected for CSV export.")
        if fileName is None:
            self.fileSaveDialog(filter=["*.csv", "*.tsv"])
            return
        for item in self.item.items:
            if isinstance(item, ErrorBarItem):
                self._exportErrorBarItem(item)
            elif hasattr(item, 'implements') and item.implements('plotData'):
                self._exportPlotDataItem(item)
        try:
            # we want to flatten the nested arrays of data into columns
            columns = [column for dataset in self.data for column in dataset]
            if len(columns) == 0:
                raise QtTinySAExportException(
                    "No column data to export / empty graph!")
            if len(columns) <= self.params['trace'] * 2:
                raise QtTinySAExportException(
                    "Missing column data for selected trace - can't export!")
            row_x = 0
            row_y = 1
            if self.params['trace'] == 1:
                row_x = 0
                row_y = 1
            elif self.params['trace'] == 2:
                row_x = 2
                row_y = 3
            elif self.params['trace'] == 3:
                row_x = 4
                row_y = 5
            elif self.params['trace'] == 4:
                row_x = 6
                row_y = 7
            self.header = [["Receiver", ''],
                        ["Date/Time", ''],
                        ["RFUnit", "dBm"],
                        ["Owner", ''],
                        ["ScanCity", ''],
                        ["ScanComment", ''],
                        ["ScanCountry", ''],
                        ["ScanDescription", ''],
                        ["ScanInteriorExterior", ''],
                        ["ScanLatitude", ''],
                        ["ScanLongitude", ''],
                        ["ScanName", ''],
                        ["ScanPostalCode", ''],
                        ["Frequency Range [kHz]",
                            f"{int(columns[row_x][0] / 1000)}",
                            f"{int(columns[row_x][-1] / 1000)}",
                            f"{len(columns[row_x])}"],
                        ["Frequency", "RF level (%)", "RF level"]]
            with open(fileName, 'w', newline='', encoding='utf-8') as csvfile:
                writer = csv.writer(csvfile, delimiter=';', quoting=csv.QUOTE_MINIMAL)
                writer.writerows(self.header)
                for row in itertools.zip_longest(*columns, fillvalue=""):
                    if isinstance(row[row_x], str):
                        x = row[row_x]
                    else:
                        x = int(row[row_x] / 1000)
                    if isinstance(row[row_y], str):
                        y = row[row_y]
                    else:
                        y = np.format_float_positional(row[row_y], precision=5)
                    row_to_write = [x, '', y]
                    writer.writerow(row_to_write)
        except QtTinySAExportException as e:
            error_dialog = QMessageBox()
            error_dialog.setIcon(QMessageBox.Icon.Critical)
            error_dialog.setText(str(e))
            error_dialog.setWindowTitle('Error')
            error_dialog.exec()
        self.header.clear()
        self.data.clear()
