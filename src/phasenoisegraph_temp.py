#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jan  9 09:54:07 2026

@author: ian
"""

# class PhaseNoiseGraph(QObject):

#     def __init__(self, ui_widget, frequencies, readings, rbw):
#         super().__init__()

#         ui_widget.addLegend()
#         self.lsb_noise = ui_widget.plot([], [], name='lsb', width=1, padding=0, pen='y')
#         self.usb_noise = ui_widget.plot([], [], name='usb', width=1, padding=0, pen='w')
#         self.base_noise = ui_widget.plot([], [], name='tinySA phase noise', width=1, padding=0, pen='g')
#         self.box = pyqtgraph.TextItem(text='', color='k', border='y', fill='y', anchor=(-8, -0.4))  # anchor y=vertical
#         self.box.setParentItem(ui_widget.plotItem)
#         self.box.setVisible(False)

#         self.updater = PhaseNoiseUpdater(frequencies, readings, rbw)
        # From https://github.com/Hagtronics/tinySA-Ultra-Phase-Noise


# class PhaseNoiseUpdater(QObject):

#     def __init__(self, ui_widget):
#         super().__init__()

#     def updatePhaseNoise(self, frequencies, levels, rbw):
#         SHAPE_FACTOR = {0.2: 3.4, 1: -0.6, 3: -5.3, 10: 0, 30: 0, 100: 0, 300: 0, 600: 0, 850: 0}
#         '''M1 is used in max tracking mode to find the frequency and level reference of the signal to be measured
#            T1 data is used for the LSB measurement and T2 data is used for USB'''
#         # frequencies, levels = self.fetchData()
#         # if frequencies is None or levels is None:
#         #     return
#         # rbw = float(QtTSA.rbw_box.currentText())  # kHz

#         # find freq array index of peak of signal
#         tone = np.argmin(np.abs(frequencies - (M1.line.value())))
#         # count freq points in sideband, masking values nearer to carrier than 2*rbw kHz
#         sideband_points = np.count_nonzero(frequencies[tone:] < frequencies[tone] + (2 * rbw * 1e3))

#         eqnbw = SHAPE_FACTOR.get(rbw)

#         # Calculate Noise Power 1Hz bandwidth normalising factor for the RBW
#         factor = 10 * np.log10(rbw * 1e3)

#         if lsb:
#             delta = levels[:tone-sideband_points+1] - levels[tone]  # relative level of lower sideband points to the marked carrier
#             freqOffset = (frequencies[tone] - frequencies[:tone-sideband_points+1])
#         else:
#             delta = levels[tone+sideband_points:] - levels[tone]  # relative level of upper sideband points to the marked carrier
#             freqOffset = (frequencies[tone+sb_points:] - frequencies[tone])
#         dBcHz = delta + eqnbw - factor

#         self.noise.setData(freqOffset, dBcHz)

#         # show signal frequency from Marker 1 in label box
#         T1.box.setVisible(True)
#         decimal = M1.setPrecision(frequencies, frequencies[0])
#         unit, multiple = M1.setUnit(frequencies[0])  # set units
#         self.box.setText(f'{M1.line.value()/multiple:.{decimal}f}{unit}')

#         # draw tinySA typical baseline phase noise on the graph
#         if lsb:  # only need to do it once (per sweep)
#             if frequencies[0] < 100e6:
#                 baseline = np.interp(freqOffset, PN_FREQS, PN_AT_30MHZ)
#             else:
#                 baseline = np.interp(freqOffset, PN_AT_1152MHZ[:, 0], PN_AT_1152MHZ[:, 1])
#             self.noise_spec.setData(freqOffset, baseline)
