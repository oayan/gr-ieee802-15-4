# Copyright (C) 2020 Hasan Yagiz Özkan <yagiz.oezkan@tum.de>
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

from multiprocessing import Process, Queue
import matplotlib.pyplot as plt
import numpy as np
import sys
import config
# from applicationwindow import  ApplicationWindow
# from matplotlib.backends.qt_compat import QtCore, QtWidgets
from protocol import Protocol
import asyncio
import struct
from invertedpendulumwidget import InvertedPendulumWidget
import time
# if int(QtCore.qVersion()[0]) == 5:
#     from matplotlib.backends.backend_qt5agg import (
#         FigureCanvas, NavigationToolbar2QT as NavigationToolbar)
# else:
#     from matplotlib.backends.backend_qt4agg import (
#         FigureCanvas, NavigationToolbar2QT as NavigationToolbar)

from matplotlib.figure import Figure
from config import GUI_TYPE, GUIShowType, INVERTED_PENDULUM_COLOURS

from queue import Empty

class GUIProcess(Process):
    """
    GUIProcess class responsible for visualization of the plants' state usin QT GUI libraries.
    --  There is only one instance of GUIProcess for N control processes running on the same machine.

    --  Whenever a control loop updates it's state, which happens only after advancing to a new time step,
        i.e., sampling, the new state is injected into the queue monitored 'awaited' by the GUIProcess.

    --  With an (approximate) update frequency of f_gui (default 30 Hz):
        1. GUI Process wakes up
        2. empties the queue by reading all the packets
        3. re-renders the animation scene
    """

    def __init__(self, _queue: Queue):
        super().__init__(target=self.run, args=(None,))
        self.ctrl_to_gui_queue = _queue

        self.widgets = {}
        for i in range(1, config.NUMBER_OF_LOOPS + 1):
            self.add_new_instance(i)

        self.print("Initialization completed successfully!")

    @classmethod
    def print(cls, txt, _end='\n', _name='GUIP:'):
        print(f"{config.bcolors.GUIPROCESS}{_name} {txt}{config.bcolors.ENDC}", end=_end)

    def main_loop(self) -> None:
        canvas_refresh_period = 0.030 # approx. 30 ms
        self.print("Starting GUI Process")
        t_last = time.perf_counter()
        closed_loops = 0
        while closed_loops < config.NUMBER_OF_LOOPS:
            # Wait for rendering time to come (subtract the processing delay from refresh period)
            # This doesn't have to be accurate anyways, approximate behavior is enough for GUIProcess
            time.sleep(canvas_refresh_period - min(canvas_refresh_period, time.perf_counter() - t_last))
            t_last = time.perf_counter()

            # Wake up and read all available packets
            state_changed = False

            while True:
                try:
                    status_update = self.ctrl_to_gui_queue.get_nowait()
                    loop_id, seq_num, state = Protocol.decode_state(status_update)
                    if seq_num == config.SIMULATION_END_SEQ_NR:
                        self.widgets[loop_id].close_figure()
                        del self.widgets[loop_id]
                        closed_loops += 1
                    else:
                        self.update_widget(loop_id, seq_num, state)
                        state_changed = True
                except Empty:
                    # Queue Empty, stop reading the queue
                    break
                except struct.error as err:
                    self.print("Unpacking err", err)
                except:
                   raise

            # Finished reading all packets in the queue, now update the canvas
            if state_changed:
                self.render()

    def run(self) -> None:
        # Directly end process if GUI is not intended to be shown
        if not config.SHOW_GUI:
            return
        self.main_loop()
        print("close gui")

    # RENDERING METHODS

    def add_new_instance(self, loop_id) -> None:
        """
        :param gridx: x position of the widget in layout
        :param gridy: y position of the widget in layout
        :param loop_id: ID of the loop, ranges from 1 to N
        :return: None
        """
        #self.plantRunningFlags = self.plantRunningFlags | (1 << loop_id)
        widget = InvertedPendulumWidget(parent=self, _loop_id=loop_id)

        assert (loop_id not in self.widgets)
        self.widgets[loop_id] = widget
        #widget.fig.show()

    def update_widget(self, loop_id: int, time_step: int, state: np.array) -> None:
        w = self.widgets[loop_id]
        w.update_state(time_step, state)

    def render(self):
        for _, w in self.widgets.items():
            w.update_draw()


