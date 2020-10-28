# Copyright (C) 2020 Jiayu Zhong
#               2020 Hasan Yagiz Özkan <yagiz.oezkan@tum.de>
#               2020 Onur Ayan <onur.ayan@tum.de>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import numpy as np
import os
import re
import pandas as pd
import sys
import config
import time
import json
import shutil
from PySide2.QtCore import QThread, SIGNAL, QTimer
from pathlib import Path
from matplotlib.backends.qt_compat import QtCore, QtWidgets

if int(QtCore.qVersion()[0]) == 5:
    from matplotlib.backends.backend_qt5agg import (
        FigureCanvas, NavigationToolbar2QT as NavigationToolbar)
else:
    from matplotlib.backends.backend_qt4agg import (
        FigureCanvas, NavigationToolbar2QT as NavigationToolbar)
from matplotlib.figure import Figure
from client import Client
from model import InvertedPendulum
from datetime import datetime
if config.SHOW_GUI:
    from widget import InvertedPendulumWidget


write_time = None
RESULTS_FOLDER_PATH = "Logs/"

#live plot, logging, Plant ID


#main thread creat widgets and server and connect together

class ApplicationWindow(QtWidgets.QMainWindow):
    def __init__(self):
        os.nice(20)
        if config.SHOW_GUI:
            super().__init__()
        np.random.seed(int(time.time()))

        #set the draw_flag in widget to switch between ani and state
        if config.SHOW_GUI:
            self._main = QtWidgets.QWidget()
            self.setCentralWidget(self._main)
            self.layout = QtWidgets.QGridLayout(self._main)
            self.setWindowState(QtCore.Qt.WindowMaximized)
        self.s_port = list(range(config.PLANT_SEND_PORT_START, config.PLANT_SEND_PORT_STOP))
        self.l_port = list(range(config.PLANT_LISTEN_PORT_START, config.PLANT_LISTEN_PORT_STOP))
        self.folder_path = RESULTS_FOLDER_PATH
        self.create_log_folders()
        self.plantCnt = 0
        self.plantRunningFlags = 0

        for gridx in range(config.NUMBER_OF_LOOPS):
            for gridy in range(1):
                self.add_new_instace(gridx, gridy)
                time.sleep(0.01)

    def add_new_instace(self, gridx, gridy):
        self.plantCnt += 1
        self.plantRunningFlags = self.plantRunningFlags | (1 << self.plantCnt)
        if config.SHOW_GUI:
            widget = InvertedPendulumWidget(parent=self, plant_id=self.plantCnt)
            self.layout.addWidget(widget, gridx, gridy)
        client = Client(l_port=self.l_port.pop(0), s_port=self.s_port.pop(0), plant_id=self.plantCnt)
        model = InvertedPendulum(self.plantCnt)
        if config.SHOW_GUI:
            model.Data.data.connect(widget.update_state)
        client.Data.control.connect(model.update_control, type=QtCore.Qt.DirectConnection)
        model.Data.state.connect(client.send_state, type=QtCore.Qt.DirectConnection)
        model.Data.stop.connect(self.exit_model)

        try:
            if config.SHOW_GUI:
                self.widgets.append(widget)
            self.clients.append(client)
            self.models.append(model)
        except:
            if config.SHOW_GUI:
                self.widgets =[]
                self.widgets.append(widget)
            self.clients =[]
            self.clients.append(client)
            self.models =[]
            self.models.append(model)

    def start_sim(self):
        if config.SHOW_GUI:
            for widget in self.widgets:
                widget.start()
        for model in self.models:
            model.start()
        for client in self.clients:
            client.start()

    def exit_model(self, p_id):
        if self.plantRunningFlags & (1 << p_id):
            m = self.models[p_id - 1]
            c = self.clients[p_id - 1]
            c.send_exit()
            m.quit()
            c.quit()
            self.plantRunningFlags = self.plantRunningFlags ^ (1 << p_id)
            print("end p_id = ", p_id, " flags = ", self.plantRunningFlags)
            if self.plantRunningFlags == 0:
                for p_num in range(1, config.NUMBER_OF_LOOPS + 1):
                    cli = self.clients[p_num - 1]
                    mdl = self.models[p_num - 1]
                    with open(self.folder_path + "/Loop_{}/send_time.json".format(p_num), 'w', encoding='utf-8') as f:
                        json.dump({'send time': cli.send_time[0].tolist()}, f, ensure_ascii=False, indent=4)
                    with open(self.folder_path + "/Loop_{}/receive_time.json".format(p_num), 'w', encoding='utf-8') as f:
                        json.dump({'receive time': cli.receive_time[0].tolist()}, f, ensure_ascii=False, indent=4)
                    with open(self.folder_path + "/Loop_{}/rtt.json".format(p_num), 'w', encoding='utf-8') as f:
                        json.dump({'round trip time': (cli.receive_time[0] - cli.send_time[0]).tolist()}, f, ensure_ascii=False, indent=4)

                    with open(self.folder_path + "/Loop_{}/aoi.json".format(p_num), 'w', encoding='utf-8') as f:
                        json.dump({'age of information': mdl.aoi_list[0].tolist()}, f, ensure_ascii=False, indent=4)
                    with open(self.folder_path + "/Loop_{}/update_time.json".format(p_num), 'w', encoding='utf-8') as f:
                        json.dump({'update time': mdl.update_time[0].tolist()}, f, ensure_ascii=False, indent=4)
                    with open(self.folder_path + "/Loop_{}/plant_state.json".format(p_num), 'w', encoding='utf-8') as f:
                        json.dump({'cart position': mdl.plant_values[0].tolist(), 'cart velocity': mdl.plant_values[1].tolist(),
                                   'pendulum angle': mdl.plant_values[2].tolist(), 'angular speed': mdl.plant_values[3].tolist()},
                                  f, ensure_ascii=False, indent=4)
                    with open(self.folder_path + "/Loop_{}/control_state.json".format(p_num), 'w', encoding='utf-8') as f:
                        json.dump({'control value': mdl.control_values[0].tolist()}, f, ensure_ascii=False, indent=4)

                    p_num += 1
                QtWidgets.QApplication.instance().quit()

    def create_log_folders(self):
        try:
            last_log = 0
            p = Path(self.folder_path + config.get_strategy_abbreviation())
            for folder in p.iterdir():
                if folder.is_dir():
                    if "Measurement_" in folder.stem:
                        log_num = folder.stem.replace('Measurement_', '')
                        if last_log < int(log_num):
                            last_log = int(log_num)
            self.folder_path = self.folder_path + config.get_strategy_abbreviation() + "/Measurement_{}/".format(last_log + 1)
            time.sleep(0.1)
            os.makedirs(self.folder_path)
        except:
            print("Folder could not be created!")
        with open(self.folder_path + "config.json", 'w', encoding='utf-8') as f:
            json.dump({'log time': str(write_time),
                       'number of loops': config.NUMBER_OF_LOOPS,
                       'simulation duration': config.SIMULATION_DURATION_SECONDS,
                       'sampling period': config.SAMPLING_PERIOD_S,
                       'Strategy': config.STRATEGY,
                       'GUI Enable': config.SHOW_GUI,
                       'Q': config.Q.flatten().tolist(),
                       'R': config.R.flatten().tolist()
                       }, f, ensure_ascii=False, indent=4)
        for p_num in range(1, config.NUMBER_OF_LOOPS + 1):
            try:
                os.makedirs(self.folder_path + "/Loop_{}".format(p_num))
            except:
                print("Folder could not be created!")
                exit(1)


if __name__ == "__main__":
    # Check whether there is already a running QApplication (e.g., if running
    # from an IDE).
    write_time = datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
    qapp = QtWidgets.QApplication.instance()
    if not qapp:
        qapp = QtWidgets.QApplication(sys.argv)

    app = ApplicationWindow()
    if config.SHOW_GUI:
        app.show()
        app.activateWindow()
        app.raise_()
    app.start_sim()
    qapp.exec_()
