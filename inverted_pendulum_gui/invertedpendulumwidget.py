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
from config import LENGTH_OF_THE_PENDULUM


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
        """ This method is called by the parent widget (ApplicationWindow) to trigger a drawing update.
            State4plot should contain the relevant coordinates of the state elements
            relative to the canvas' coordinate system."""
        # Cart's marker data x, y coordinates (y coordinate is always zero for the cart)
        self.cart_marker.set_data(self._get_cart_coordinate_x(), self._get_cart_coordinate_y())     # y-coord is 0
        # Pendulum head's marker data x, y coordinates
        self.pendulum_marker.set_data(self._get_pendulum_head_coordinate_x(), self._get_pendulum_head_coordinate_y())
        # Pendulum body as line
        self.pendulum_line.set_data([self._get_cart_coordinate_x(), self._get_pendulum_head_coordinate_x()],
                                    [self._get_cart_coordinate_y(), self._get_pendulum_head_coordinate_y()])
        # Re-draw with newly updated state
        self._animation_ax.figure.canvas.draw()

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

        # Square marker for the cart
        self.cart_marker, = self._animation_ax.plot([], [], linestyle='None', marker='s',
                                                    markersize=40, markeredgecolor='k',
                                                    color=loop_color, markeredgewidth=2)

        # Round marker for the head of the pendulum
        self.pendulum_marker, = self._animation_ax.plot([], [], linestyle='None', marker='o',
                                                        markersize=20, markeredgecolor='k',
                                                        color=loop_color, markeredgewidth=2)

        # Line for the pendulum's body
        self.pendulum_line, = self._animation_ax.plot([], [], 'o-', color=loop_color, lw=4,
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
            self.freshest_state[0, :] = state[0, :]
            self.freshest_timestep = timestep
#            self.state4plot = self.state2coord(self.freshest_state) # TODO remove this state4plot completely
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

    def _get_cart_coordinate_x(self) -> float:
        return self.freshest_state[PlantStates.cart_position, 0]

    def _get_cart_coordinate_y(self) -> float:
        return 0.0
    
    def _get_pendulum_head_coordinate_x(self) -> float:
        # phi positive in counterclockwise
        return -LENGTH_OF_THE_PENDULUM * np.sin(self.freshest_state[PlantStates.pendulum_angle, 0]) + self._get_cart_coordinate_x()

    def _get_pendulum_head_coordinate_y(self) -> float:
        return LENGTH_OF_THE_PENDULUM * np.cos(self.freshest_state[PlantStates.pendulum_angle, 0])

    @staticmethod
    def state2coord(state, length=0.2):
        x = state[0, PlantStates.cart_position]
        phi = state[0, PlantStates.pendulum_angle]
        x_cart = x
        x_pendulum_head = -length * np.sin(phi) + x
        y_pendulum_head = length * np.cos(phi)
        state4plot = [x_cart, x_pendulum_head, y_pendulum_head]
        return state4plot


class PlantStates:
    cart_position = 0
    cart_speed = 1
    pendulum_angle = 2
    pendulum_speed = 3
