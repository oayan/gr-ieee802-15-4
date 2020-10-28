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
from PySide2.QtCore import QThread, Signal,Slot

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


class InvertedPendulumWidget(FigureCanvas):

    def __init__(self, plant_id, parent=None, width=5, height=4, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)

        self.draw_flag = GUI_TYPE  #0 for animation , 1 for state

        if self.draw_flag == GUIShowType.animation:
            self._init_animation(plant_id)
        elif self.draw_flag == GUIShowType.realtime_plot:
            self._init_static_plot()
        super().__init__(self.fig)

    def _update_animation(self):
        try:
            self.mass1.set_data(self.state4plot[0],0)
            self.mass2.set_data(self.state4plot[1],self.state4plot[2])
            self.line.set_data([self.state4plot[0],self.state4plot[1]],[0,self.state4plot[2]])
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

    def start(self,fps=30):
        #target function for thread.

        #drawing
        if self.draw_flag ==0:
            update = self._update_animation
        elif self.draw_flag ==1:
            update = self._update_static_plot
        self._timer = self.new_timer(
            1000/fps, [(update, (), {})])
        self._timer.start()


    def _init_animation(self, plant_id):
        try :
            self._state_ax.remove()
            self._state_ax2.remove()
        except:
            pass
        self._animation_ax = self.fig.add_subplot(111, autoscale_on=False, xlim=(-1., 1), ylim=(-0.4, 1.2))
        self._animation_ax.set_xlabel('position')
        self._animation_ax.get_yaxis().set_visible(False)

        crane_rail, = self._animation_ax.plot([-10,10],[-0,-0],'k-',lw=4)
        # start, = self._dynamic_ax.plot([-1,-1],[-1.5,1.5],'k:',lw=2)
        # ref, = self._dynamic_ax.plot([0,0],[-0.5,1.5],'k:',lw=2)
        self.mass1, = self._animation_ax.plot([],[],linestyle='None',marker='s',\
                        markersize=40,markeredgecolor='k',\
                        color=INVERTED_PENDULUM_COLOURS[plant_id-1],markeredgewidth=2)
        self.mass2, = self._animation_ax.plot([],[],linestyle='None',marker='o',\
                        markersize=20,markeredgecolor='k',\
                        color=INVERTED_PENDULUM_COLOURS[plant_id-1],markeredgewidth=2)
        self.line, = self._animation_ax.plot([],[],'o-',color=INVERTED_PENDULUM_COLOURS[plant_id-1],lw=4,\
                        markersize=6,markeredgecolor='k',\
                        markerfacecolor='k')
        # time_template = 'time = %.1fs'
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


    def update_state(self,data):
        time_step, state = data
        time_step = time_step / 100
        try:
            self.state = np.append(self.state, [state], axis=0)
            self.state4plot = self.state2coord(state)
            self.time_step = np.append(self.time_step, [time_step], axis=0)
        except:
            self.state = [state]
            self.state4plot = self.state2coord(state)
            self.time_step = [time_step]

    @staticmethod
    def state2coord(state, l=0.5):
        x = state[PlantStates.cart_position]
        phi = state[PlantStates.pendulum_angle]
        x0 = x
        x1 = -l * np.sin(phi) + x
        y1 = l * np.cos(phi)
        state4plot = [x0, x1, y1]
        return state4plot


class PlantStates:
    cart_position = 0
    cart_speed = 1
    pendulum_angle = 2
    pendulum_speed = 3
