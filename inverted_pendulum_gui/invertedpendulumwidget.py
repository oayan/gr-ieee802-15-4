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

from matplotlib.ticker import FormatStrFormatter
from matplotlib.backends.qt_compat import QtCore, QtWidgets
import logging

if int(QtCore.qVersion()[0]) == 5:
    from matplotlib.backends.backend_qt5agg import (
        FigureCanvas, NavigationToolbar2QT as NavigationToolbar)
else:
    from matplotlib.backends.backend_qt4agg import (
        FigureCanvas, NavigationToolbar2QT as NavigationToolbar)

from matplotlib.figure import Figure
from config import GUI_TYPE, GUIShowType, INVERTED_PENDULUM_COLOURS, inv_pend_color_it, SAMPLING_PERIOD_S


class InvertedPendulumWidget(FigureCanvas):
    """
    This is the widget illustrating an inverted pendulum. Each control loop is drawn by this widget.
    N widgets are added to the application window while creating the GUI process.
    """
    def __init__(self, _loop_id, parent=None, width=5, height=4, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)     # Matplotlib Figure
        self.parent_widget = parent
        self.loop_id = _loop_id
        self.draw_flag = GUI_TYPE  # 0 for animation , 1 for 2D-plot
        self.freshest_timestep = -1
        self.freshest_state = np.zeros(shape=(4, 1))
        self.state4plot = []

        if self.draw_flag == GUIShowType.animation:
            self._init_animation()
            self.update_state = self.update_state_anim
        elif self.draw_flag == GUIShowType.realtime_plot:
            self._init_static_plot()
            self.update_state = self.update_state_2d
        super().__init__(self.fig)

    def update_animation(self):
        try:
            # TODO Please check these & Comment & Improve
            self.mass1.set_data(self.state4plot[0], 0)
            self.mass2.set_data(self.state4plot[1], self.state4plot[2])
            self.line.set_data([self.state4plot[0], self.state4plot[1]], [0, self.state4plot[2]])
            self._animation_ax.figure.canvas.draw()

        except:
            pass

    def _update_static_plot(self):
        try:
            # self._state_ax.set_xlim(self.time_step[-50],self.time_step[-1])
            self._pos.set_data(self.time_step[-500:], self.state[-500:, PlantStates.cart_position])
            self._phi.set_data(self.time_step[-500:], self.state[-500:, PlantStates.pendulum_angle])
            self._state_ax.xaxis.set_major_formatter(FormatStrFormatter('%.2f'))
            self._state_ax.relim()
            self._state_ax.autoscale()
            self._state_ax2.relim()
            self._state_ax2.autoscale()
            self.figure.canvas.draw()

        except:
            pass

    # def start(self, fps=30):
    #     if self.draw_flag == GUIShowType.animation:
    #         update = self._update_animation
    #     elif self.draw_flag == GUIShowType.realtime_plot:
    #         update = self._update_static_plot
    #     else:
    #         logging.error("Unknown GUI Type")
    #         raise TypeError
    #
    #     self._timer = self.new_timer(1000/fps, [(update, (), {})])
    #     self._timer.start()

    def _init_animation(self):
        try:
            # TODO redundant I believe
            self._state_ax.remove()
            self._state_ax2.remove()
        except:
            pass

        self._animation_ax = self.fig.add_subplot(111, autoscale_on=False, xlim=(-1., 1), ylim=(-0.4, 1.2))
        self._animation_ax.set_xlabel('Cart Position [m]')
        self._animation_ax.get_yaxis().set_visible(False)

        # crane_rail, = self._animation_ax.plot([-10,10],[-0,-0],'k-',lw=4)
        # start, = self._dynamic_ax.plot([-1,-1],[-1.5,1.5],'k:',lw=2)
        # ref, = self._dynamic_ax.plot([0,0],[-0.5,1.5],'k:',lw=2)  # TODO Remove

        # Draw Inverted Pendulum
        loop_color = next(inv_pend_color_it)

        self.mass1, = self._animation_ax.plot([], [], linestyle='None', marker='s',
                                              markersize=40, markeredgecolor='k',
                                              color=loop_color, markeredgewidth=2)

        self.mass2, = self._animation_ax.plot([], [], linestyle='None', marker='o',
                                              markersize=20, markeredgecolor='k',
                                              color=loop_color, markeredgewidth=2)

        self.line, = self._animation_ax.plot([], [], 'o-', color=loop_color, lw=4,
                                             markersize=6, markeredgecolor='k', markerfacecolor='k')

        # time_template = 'time = %.1fs'    # TODO remove
        # time_text = self._animation_ax.text(0.05,0.9,'',transform=self._animation_ax.transAxes)

    def _init_static_plot(self):
        try :
            self._animation_ax.remove()
        except:
            pass
        self._state_ax = self.fig.add_subplot(111)
        self._pos, = self._state_ax.plot([],[],color='blue')
        self._state_ax2 = self._state_ax.twinx()
        self._phi, = self._state_ax2.plot([],[],color='red')
        self._state_ax.set_xlabel("time",fontsize=14)
        self._state_ax.set_ylabel("Position",fontsize=14,color='blue')
        self._state_ax2.set_ylabel("PHI",fontsize=14, color='red')
        self._state_ax.tick_params(axis='y', labelcolor='blue')
        self._state_ax2.tick_params(axis='y', labelcolor='red')
        # self._state_ax.set_ylim(-0.4,0.5)
        # self._state_ax2.set_ylim(-0.02,0.02)

    def update_state_anim(self, timestep: int, state) -> bool:
        if timestep > self.freshest_timestep:
            self.freshest_state = np.copyto(self.freshest_state, state)
            self.freshest_timestep = timestep
            self.state4plot = self.state2coord(self.freshest_state)
        else:
            self.parent_widget.print(f'Out of order packet for loop {self.loop_id}')


    # def update_state(self, time_step: int, state) -> bool:
    #     #time_step, state = data
    #     real_time_s = time_step * SAMPLING_PERIOD_S
    #
    #     try:
    #         self.state = np.append(self.state, [state], axis=0)
    #         self.state4plot = self.state2coord(state)
    #         # self.time_step = np.append(self.time_step, [real_time_s], axis=0) # TODO Missing 2D implementation, log in FIFO Queue with popping last
    #         return True
    #     except:
    #         # TODO ?
    #         self.state = [state]
    #         self.state4plot = self.state2coord(state)
    #         self.time_step = [time_step]
    #         return True

    @staticmethod
    def state2coord(state, length=0.5):
        x = state[PlantStates.cart_position]
        phi = state[PlantStates.pendulum_angle]
        x0 = x
        x1 = -length * np.sin(phi) + x
        y1 = length * np.cos(phi)
        # TODO Isn't there a +x for y1 ?
        state4plot = [x0, x1, y1]
        return state4plot


class PlantStates:
    cart_position = 0
    cart_speed = 1
    pendulum_angle = 2
    pendulum_speed = 3
