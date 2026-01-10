import csv
import itertools

import numpy as np

from PySide6.QtWidgets import QMessageBox

from pyqtgraph import ErrorBarItem, PlotItem
from pyqtgraph.exporters import CSVExporter
from pyqtgraph.parametertree import Parameter


class QtTinySAExportException(Exception):
    """Custom exception for custom pyqtgraph exporters."""


class WWBExporter(CSVExporter):
    """Shure Wireless Workbench CSV exporter class."""

    Name = "CSV for WWB (Shure)"

    def __init__(self, item):
        """Class initialisation."""
        # init superclass
        CSVExporter.__init__(self, item)
        # define parameters:
        # parameter 'trace' lets you select which trace (1-4) should be exported
        # as WWB can only read one trace over frequencies at a time
        self.params = Parameter.create(name='params', type='group', children=[
            {'name': 'trace', 'title': 'Export trace', 'type': 'list',
             'value': 1, 'limits': [1, 2, 3, 4]}
        ])
        # class variables:
        self.index_counter = itertools.count(start=0)
        # class variable for CSV header row(s)
        self.header = []
        # class variable holding the data points
        self.data = []

    def _exportPlotDataItem(self, plotDataItem) -> None:
        """Export selected trace data points to class data variable."""
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
        """Export handler to prepare and export the data to a CSV file."""
        if not isinstance(self.item, PlotItem):
            raise TypeError("Must have a PlotItem selected for CSV export.")
        # show file dialog to select file save location
        if fileName is None:
            self.fileSaveDialog(filter=["*.csv", "*.tsv"])
            return
        # handle error bar items and plot data
        for item in self.item.items:
            if isinstance(item, ErrorBarItem):
                self._exportErrorBarItem(item)
            elif hasattr(item, 'implements') and item.implements('plotData'):
                self._exportPlotDataItem(item)
        try:
            # we want to flatten the nested arrays of data into columns
            columns = [column for dataset in self.data for column in dataset]
            # error handling
            if len(columns) == 0:
                raise QtTinySAExportException(
                    'No column data to export / empty graph!')
            if len(columns) <= self.params['trace'] * 2:
                raise QtTinySAExportException(
                    "Missing column data for selected trace - can't export!")
            # select x and y based on choosen trace
            # could be done more elegant, but this works ;-)
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
            # write CSV file
            with open(fileName, 'w', newline='', encoding='utf-8') as csvfile:
                # set CSV flavor
                writer = csv.writer(csvfile, delimiter=',',
                                    quoting=csv.QUOTE_MINIMAL)
                # iterate thru data and write row by row
                for row in itertools.zip_longest(*columns, fillvalue=""):
                    if isinstance(row[row_x], str):
                        # data is string -> do nothing
                        x = row[row_x]
                    else:
                        # data is number (?) -> convert to float and set precision
                        # convert frequencies to MHz with precision of 0.000
                        x = np.format_float_positional(int(row[row_x]) / 1000000,
                                                       precision=3,
                                                       unique=True,
                                                       min_digits=3,
                                                       fractional=True)
                    if isinstance(row[row_y], str):
                        # data is string -> do nothing
                        y = row[row_y]
                    else:
                        # data is number (?) -> convert to float and set precision
                        # convert level dB values to floating with 0.0000
                        y = np.format_float_positional(int(row[row_y]),
                                                       precision=4,
                                                       unique=True,
                                                       min_digits=4,
                                                       fractional=True)
                    # prepare row to write
                    row_to_write = [x, y]
                    # write row to file
                    writer.writerow(row_to_write)
        except QtTinySAExportException as e:
            # show error dialog box in window with error description
            error_dialog = QMessageBox()
            error_dialog.setIcon(QMessageBox.Icon.Critical)
            error_dialog.setText(str(e))
            error_dialog.setWindowTitle('Error')
            error_dialog.exec()
        # garbage collection, cleaning up!
        self.header.clear()
        self.data.clear()


class WSMExporter(CSVExporter):
    """Sennheiser WSM CSV exporter class."""
    Name = "CSV for WSM (Sennheiser)"

    def __init__(self, item):
        """Class initialisation."""
        # init superclass
        CSVExporter.__init__(self, item)
        # define parameters:
        # parameter 'trace' lets you select which trace (1-4) should be exported
        # as WWB can only read one trace over frequencies at a time
        self.params = Parameter.create(name='params', type='group', children=[
            {'name': 'trace', 'title': 'Export trace', 'type': 'list',
             'value': 1, 'limits': [1, 2, 3, 4]}
        ])
        # class variables:
        self.index_counter = itertools.count(start=0)
        # class variable for CSV header row(s)
        self.header = []
        # class variable holding the data points
        self.data = []

    def _exportPlotDataItem(self, plotDataItem) -> None:
        """Export selected trace data points to class data variable."""
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
        """Export handler to prepare and export the data to a CSV file."""
        if not isinstance(self.item, PlotItem):
            raise TypeError("Must have a PlotItem selected for CSV export.")
        # show file dialog to select file save location
        if fileName is None:
            self.fileSaveDialog(filter=["*.csv", "*.tsv"])
            return
        # handle error bar items and plot data
        for item in self.item.items:
            if isinstance(item, ErrorBarItem):
                self._exportErrorBarItem(item)
            elif hasattr(item, 'implements') and item.implements('plotData'):
                self._exportPlotDataItem(item)
        try:
            # we want to flatten the nested arrays of data into columns
            columns = [column for dataset in self.data for column in dataset]
            # error handling
            if len(columns) == 0:
                raise QtTinySAExportException(
                    "No column data to export / empty graph!")
            if len(columns) <= self.params['trace'] * 2:
                raise QtTinySAExportException(
                    "Missing column data for selected trace - can't export!")
            # select x and y based on choosen trace
            # could be done more elegant, but this works ;-)
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
            # write CSV file
            with open(fileName, 'w', newline='', encoding='utf-8') as csvfile:
                # set CSV flavor to use ; instead of , (required by WSM)
                writer = csv.writer(csvfile, delimiter=';',
                                    quoting=csv.QUOTE_MINIMAL)
                writer.writerows(self.header)
                # iterate thru data and write row by row
                for row in itertools.zip_longest(*columns, fillvalue=""):
                    if isinstance(row[row_x], str):
                        # data is string -> do nothing
                        x = row[row_x]
                    else:
                        # convert frequency to kHz
                        x = int(int(row[row_x]) / 1000)
                    if isinstance(row[row_y], str):
                        # data is string -> do nothing
                        y = row[row_y]
                    else:
                        # convert level to float with precision 0.00000
                        y = np.format_float_positional(float(row[row_y]),
                                                       precision=5)
                    # prepare row to write
                    row_to_write = [x, '', y]
                    # write row to file
                    writer.writerow(row_to_write)
        except QtTinySAExportException as e:
            # show error dialog box in window with error description
            error_dialog = QMessageBox()
            error_dialog.setIcon(QMessageBox.Icon.Critical)
            error_dialog.setText(str(e))
            error_dialog.setWindowTitle('Error')
            error_dialog.exec()
        # garbage collection, cleaning up!
        self.header.clear()
        self.data.clear()
