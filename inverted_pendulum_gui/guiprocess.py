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

from multiprocessing import Process
import numpy as np
import sys
import config
from applicationwindow import  ApplicationWindow
from matplotlib.backends.qt_compat import QtCore, QtWidgets
if int(QtCore.qVersion()[0]) == 5:
    from matplotlib.backends.backend_qt5agg import (
        FigureCanvas, NavigationToolbar2QT as NavigationToolbar)
else:
    from matplotlib.backends.backend_qt4agg import (
        FigureCanvas, NavigationToolbar2QT as NavigationToolbar)

from matplotlib.figure import Figure
from config import GUI_TYPE, GUIShowType, INVERTED_PENDULUM_COLOURS


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

    def __init__(self, plant_id, width :int =5, height: int =4, dpi:int =100):
        super().__init__(target=self.run, args=(None,))

        # Create new ApplicationWindow instance
        if config.SHOW_GUI:
            self.app_window = ApplicationWindow(config.NUMBER_OF_LOOPS)
            self.app_window.show()
            self.app_window.activateWindow()
            self.app_window.raise_()

    def run(self) -> None:
        self.app_window.start_sim()

        while True:

            pass


