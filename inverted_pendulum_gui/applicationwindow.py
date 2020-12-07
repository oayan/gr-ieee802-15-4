import config
import json
import time
from invertedpendulumwidget import InvertedPendulumWidget
from matplotlib.ticker import FormatStrFormatter
from matplotlib.backends.qt_compat import QtCore, QtWidgets
if int(QtCore.qVersion()[0]) == 5:
    from matplotlib.backends.backend_qt5agg import (
        FigureCanvas, NavigationToolbar2QT as NavigationToolbar)
else:
    from matplotlib.backends.backend_qt4agg import (
        FigureCanvas, NavigationToolbar2QT as NavigationToolbar)

from matplotlib.figure import Figure
from config import GUI_TYPE, GUIShowType, INVERTED_PENDULUM_COLOURS


class ApplicationWindow(QtWidgets.QMainWindow):
    def __init__(self, num_loops: int):

        super().__init__()
        self._main = QtWidgets.QWidget()
        self.setCentralWidget(self._main)
        self.layout = QtWidgets.QGridLayout(self._main)
        self.setWindowState(QtCore.Qt.WindowMaximized)
        self.N = num_loops
        self.widgets = []
        # set the draw_flag in widget to switch between ani and state
        self.plantRunningFlags = 0

        loop_id = 0
        for gridx in range(self.N):
            for gridy in range(1):
                loop_id += 1
                self.add_new_instance(gridx, gridy, loop_id)     # Loop ids from 1 to N

    def add_new_instance(self, gridx, gridy, loop_id) -> None:
        """
        :param gridx: x position of the widget in layout
        :param gridy: y position of the widget in layout
        :param loop_id: ID of the loop, ranges from 1 to N
        :return: None
        """
        self.plantRunningFlags = self.plantRunningFlags | (1 << loop_id)

        widget = InvertedPendulumWidget(parent=self, _loop_id=loop_id)
        self.layout.addWidget(widget, gridx, gridy)
        self.widgets.append(widget)



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
            if not os.path.exists(self.folder_path + config.get_strategy_abbreviation()):
                try:
                    os.makedirs(self.folder_path + config.get_strategy_abbreviation())
                except:
                    print("Folder could not be created!")
                    exit(1)
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