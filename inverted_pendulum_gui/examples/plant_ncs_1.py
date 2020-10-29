#!/usr/bin/env python2
# -*- coding: utf-8 -*-
##################################################
# GNU Radio Python Flow Graph
# Title: Plant Ncs 1
# GNU Radio version: 3.7.14.0
##################################################

if __name__ == '__main__':
    import ctypes
    import sys
    if sys.platform.startswith('linux'):
        try:
            x11 = ctypes.cdll.LoadLibrary('libX11.so')
            x11.XInitThreads()
        except:
            print "Warning: failed to XInitThreads()"

import os
import sys
sys.path.append(os.environ.get('GRC_HIER_PATH', os.path.expanduser('~/.grc_gnuradio')))

from PyQt4 import Qt
from gnuradio import blocks
from gnuradio import eng_notation
from gnuradio import gr
from gnuradio import uhd
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from gnuradio.qtgui import Range, RangeWidget
from ieee802_15_4_oqpsk_phy import ieee802_15_4_oqpsk_phy  # grc-generated hier_block
from optparse import OptionParser
import ieee802_15_4
import time
from gnuradio import qtgui


class plant_ncs_1(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "Plant Ncs 1")
        Qt.QWidget.__init__(self)
        self.setWindowTitle("Plant Ncs 1")
        qtgui.util.check_set_qss()
        try:
            self.setWindowIcon(Qt.QIcon.fromTheme('gnuradio-grc'))
        except:
            pass
        self.top_scroll_layout = Qt.QVBoxLayout()
        self.setLayout(self.top_scroll_layout)
        self.top_scroll = Qt.QScrollArea()
        self.top_scroll.setFrameStyle(Qt.QFrame.NoFrame)
        self.top_scroll_layout.addWidget(self.top_scroll)
        self.top_scroll.setWidgetResizable(True)
        self.top_widget = Qt.QWidget()
        self.top_scroll.setWidget(self.top_widget)
        self.top_layout = Qt.QVBoxLayout(self.top_widget)
        self.top_grid_layout = Qt.QGridLayout()
        self.top_layout.addLayout(self.top_grid_layout)

        self.settings = Qt.QSettings("GNU Radio", "plant_ncs_1")
        self.restoreGeometry(self.settings.value("geometry").toByteArray())


        ##################################################
        # Variables
        ##################################################
        self.tx_gain = tx_gain = 0.75
        self.samp_rate = samp_rate = 32000
        self.rx_gain = rx_gain = 0.75
        self.freq_plant_tx = freq_plant_tx = 2470000000
        self.freq_plant_rx = freq_plant_rx = 2480000000

        ##################################################
        # Blocks
        ##################################################
        self._tx_gain_range = Range(0, 1, 0.01, 0.75, 200)
        self._tx_gain_win = RangeWidget(self._tx_gain_range, self.set_tx_gain, "tx_gain", "counter_slider", float)
        self.top_grid_layout.addWidget(self._tx_gain_win)
        self._rx_gain_range = Range(0, 1, 0.01, 0.75, 200)
        self._rx_gain_win = RangeWidget(self._rx_gain_range, self.set_rx_gain, "rx_gain", "counter_slider", float)
        self.top_grid_layout.addWidget(self._rx_gain_win)
        self.uhd_usrp_source_0 = uhd.usrp_source(
        	",".join(('serial=3188134', "")),
        	uhd.stream_args(
        		cpu_format="fc32",
        		channels=range(1),
        	),
        )
        self.uhd_usrp_source_0.set_samp_rate(4000000)
        self.uhd_usrp_source_0.set_center_freq(freq_plant_rx, 0)
        self.uhd_usrp_source_0.set_normalized_gain(rx_gain, 0)
        self.uhd_usrp_source_0.set_auto_dc_offset(False, 0)
        self.uhd_usrp_source_0.set_auto_iq_balance(False, 0)
        self.uhd_usrp_sink_0 = uhd.usrp_sink(
        	",".join(('serial=3188134', "")),
        	uhd.stream_args(
        		cpu_format="fc32",
        		channels=range(1),
        	),
        	'pdu_length',
        )
        self.uhd_usrp_sink_0.set_samp_rate(4000000)
        self.uhd_usrp_sink_0.set_center_freq(freq_plant_tx, 0)
        self.uhd_usrp_sink_0.set_normalized_gain(tx_gain, 0)
        self.ieee802_15_4_rime_stack_0_0 = ieee802_15_4.rime_stack(([129]), ([131]), ([132]), ([23,42]))
        self.ieee802_15_4_oqpsk_phy_0_0 = ieee802_15_4_oqpsk_phy()
        self.ieee802_15_4_mac_plant_0 = ieee802_15_4.mac_plant(False,1,0x8841,0,0x1aaa,0x01)
        self.blocks_socket_pdu_0_0_1 = blocks.socket_pdu("UDP_SERVER", '127.0.0.1', '5100', 10000, False)
        self.blocks_socket_pdu_0_0_0_0 = blocks.socket_pdu("UDP_CLIENT", '127.0.0.1', '5150', 10000, False)



        ##################################################
        # Connections
        ##################################################
        self.msg_connect((self.blocks_socket_pdu_0_0_1, 'pdus'), (self.ieee802_15_4_rime_stack_0_0, 'bcin'))
        self.msg_connect((self.ieee802_15_4_mac_plant_0, 'pdu out'), (self.ieee802_15_4_oqpsk_phy_0_0, 'txin'))
        self.msg_connect((self.ieee802_15_4_mac_plant_0, 'app out'), (self.ieee802_15_4_rime_stack_0_0, 'fromMAC'))
        self.msg_connect((self.ieee802_15_4_oqpsk_phy_0_0, 'rxout'), (self.ieee802_15_4_mac_plant_0, 'pdu in'))
        self.msg_connect((self.ieee802_15_4_rime_stack_0_0, 'bcout'), (self.blocks_socket_pdu_0_0_0_0, 'pdus'))
        self.msg_connect((self.ieee802_15_4_rime_stack_0_0, 'toMAC'), (self.ieee802_15_4_mac_plant_0, 'app in'))
        self.connect((self.ieee802_15_4_oqpsk_phy_0_0, 0), (self.uhd_usrp_sink_0, 0))
        self.connect((self.uhd_usrp_source_0, 0), (self.ieee802_15_4_oqpsk_phy_0_0, 0))

    def closeEvent(self, event):
        self.settings = Qt.QSettings("GNU Radio", "plant_ncs_1")
        self.settings.setValue("geometry", self.saveGeometry())
        event.accept()

    def get_tx_gain(self):
        return self.tx_gain

    def set_tx_gain(self, tx_gain):
        self.tx_gain = tx_gain
        self.uhd_usrp_sink_0.set_normalized_gain(self.tx_gain, 0)


    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate

    def get_rx_gain(self):
        return self.rx_gain

    def set_rx_gain(self, rx_gain):
        self.rx_gain = rx_gain
        self.uhd_usrp_source_0.set_normalized_gain(self.rx_gain, 0)


    def get_freq_plant_tx(self):
        return self.freq_plant_tx

    def set_freq_plant_tx(self, freq_plant_tx):
        self.freq_plant_tx = freq_plant_tx
        self.uhd_usrp_sink_0.set_center_freq(self.freq_plant_tx, 0)

    def get_freq_plant_rx(self):
        return self.freq_plant_rx

    def set_freq_plant_rx(self, freq_plant_rx):
        self.freq_plant_rx = freq_plant_rx
        self.uhd_usrp_source_0.set_center_freq(self.freq_plant_rx, 0)


def main(top_block_cls=plant_ncs_1, options=None):

    from distutils.version import StrictVersion
    if StrictVersion(Qt.qVersion()) >= StrictVersion("4.5.0"):
        style = gr.prefs().get_string('qtgui', 'style', 'raster')
        Qt.QApplication.setGraphicsSystem(style)
    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls()
    tb.start()
    tb.show()

    def quitting():
        tb.stop()
        tb.wait()
    qapp.connect(qapp, Qt.SIGNAL("aboutToQuit()"), quitting)
    qapp.exec_()


if __name__ == '__main__':
    main()
