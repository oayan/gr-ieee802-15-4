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
import sys
import config
from applicationwindow import  ApplicationWindow
from matplotlib.backends.qt_compat import QtCore, QtWidgets
from protocol import Protocol
import asyncio
import time
if int(QtCore.qVersion()[0]) == 5:
    from matplotlib.backends.backend_qt5agg import (
        FigureCanvas, NavigationToolbar2QT as NavigationToolbar)
else:
    from matplotlib.backends.backend_qt4agg import (
        FigureCanvas, NavigationToolbar2QT as NavigationToolbar)

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
        # Create new ApplicationWindow instance
        if config.SHOW_GUI:
            self.app_window = ApplicationWindow(config.NUMBER_OF_LOOPS)
            self.app_window.show()
            self.app_window.activateWindow()
            self.app_window.raise_()

    @classmethod
    def print(cls, txt, _end='\n', _name='GUIP:'):
        print(f"{config.bcolors.GUIPROCESS}{_name} {txt}{config.bcolors.ENDC}", end=_end)

    async def main_loop(self) -> None:
        canvas_refresh_period = 0.030  # approx. 30 ms
        self.print("Starting GUI Process")
        # self.app_window.start_sim()# TODO Remove
        t_last = time.perf_counter()
        while True:
            # Wait for rendering time to come (substract the processing delay from refresh period)
            # This doesn't have to be accurate anyways, approximate behavior is enough for GUIProcess
            await asyncio.sleep(canvas_refresh_period - min(canvas_refresh_period, time.perf_counter() - t_last))
            t_last = time.perf_counter()

            # Wake up and read all available packets
            while True:
                try:
                    status_update = self.ctrl_to_gui_queue.get_nowait()
                    loop_id, seq_num, state = Protocol.decode_state(status_update)
                    self.app_window.update_widget(loop_id, seq_num, state)
                except Empty:
                    # Queue Empty, stop reading the queue
                    break
                else:
                    self.print("queue handling, unknown exception")
                    raise ValueError

            # Finished reading all packets in the queue, now update the canvas
            self.app_window.render()

    def run(self) -> None:
        # Directly end process if GUI is not intended to be shown
        if not config.SHOW_GUI:
            return

        asyncio.run(self.main_loop())


qapp = QtWidgets.QApplication.instance()
if not qapp:
    qapp = QtWidgets.QApplication(sys.argv)
q = Queue()
gp = GUIProcess(q)
gp.start()
