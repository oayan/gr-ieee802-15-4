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

# NETWORK CONFIGURATION
NUMBER_OF_LOOPS = 2
CONTROLLER_SEND_PORT_START = 5000
CONTROLLER_LISTEN_PORT_START = 5050
PLANT_SEND_PORT_START = 5100
PLANT_LISTEN_PORT_START = 5150
SIMULATION_DURATION_SECONDS = 20

PLANT_SEND_PORT_STOP = NUMBER_OF_LOOPS + PLANT_SEND_PORT_START
PLANT_LISTEN_PORT_STOP = NUMBER_OF_LOOPS + PLANT_LISTEN_PORT_START
CONTROLLER_SEND_PORT_STOP = NUMBER_OF_LOOPS + CONTROLLER_SEND_PORT_START
CONTROLLER_LISTEN_PORT_STOP = NUMBER_OF_LOOPS + CONTROLLER_LISTEN_PORT_START


class Strategy:
    fcfs_td = "First Come First Served - Tail Drop with Retransmission"
    lcfs_pd = "Last Come First Served Packet Discard with Retransmission"
    fcfs_td_tod = "First Come First Served - Tail Drop without Retransmission"
    fcfs_fd = "First Come First Served - Front Drop with Retransmission"
    fcfs_fd_tod = "First Come First Served - Front Drop without Retransmission"
    random = "Without Beacon"


STRATEGY = Strategy.lcfs_pd


def get_strategy_abbreviation():
    if STRATEGY == Strategy.fcfs_td:
        return "FCFS_TD"
    elif STRATEGY == Strategy.lcfs_pd:
        return "LCFS"
    elif STRATEGY == Strategy.fcfs_td_tod:
        return "FCFS_TOD_TD"
    elif STRATEGY == Strategy.fcfs_fd:
        return "FCFS_FD"
    elif STRATEGY == Strategy.fcfs_fd_tod:
        return "FCFS_TOD_FD"
    elif STRATEGY == Strategy.random:
        return "WO_BEACON"


# GUI CONFIGURATION
class GUIShowType:
    animation = 0
    realtime_plot = 1


INVERTED_PENDULUM_COLOURS = ['orange', 'lawngreen', 'fuchsia', 'salmon', 'royalblue']
GUI_TYPE = GUIShowType.animation
SHOW_GUI = True

# INVERTED PENDULUM CONFIGURATION
SAMPLING_PERIOD_S = 0.01
COEFFICIENT_OF_FRICTION_FOR_CART = 0.1  # friction coefficient
MASS_MOMENT_OF_INERTIA_OF_THE_PENDULUM = 0.006  # mass moment of the pendulum
GRAVITATIONAL_COEFFICIENT = 9.8  # gravitational coefficient
MASS_OF_THE_PENDULUM = 0.2  # mass of the pendulum
MASS_OF_THE_CART = 0.5  # mass of the cart
LENGTH_OF_THE_PENDULUM = 0.2  # length to pendulum center of mass
REFERENCE_VALUE = np.zeros(shape=(1, 10000))
PRECOMPENSATOR = np.array([[-61.55]])
Q = np.array([[5000, 0, 0, 0], [0, 0, 0, 0], [0, 0, 100, 0], [0, 0, 0, 0]])
R = np.array([[1]])


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'