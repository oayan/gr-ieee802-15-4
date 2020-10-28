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
import control, control.matlab
import math
import socket, sys, errno
from PySide2.QtCore import QThread, Signal,Slot, QObject
from time import sleep, time
from protocol import Protocol, Data
from config import PLANT_SEND_PORT_START, PLANT_LISTEN_PORT_START
from config import SAMPLING_PERIOD_S as T_s
from config import SIMULATION_DURATION_SECONDS as T_sim


class Client(QThread):

    def __init__(self, ls_ip='127.0.0.1', l_port=PLANT_LISTEN_PORT_START, s_port=PLANT_SEND_PORT_START, plant_id=1):
        self.Data = Data()
        self.l_ip = ls_ip
        self.l_port = l_port
        self.plant_id = plant_id
        self.s_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP Socket
        self.s_sock.setblocking(False)
        self.s_ip = ls_ip
        self.s_port = s_port
        self.s_dest = (self.s_ip, self.s_port)
        self.send_time = np.zeros(shape=(1, int(math.ceil(T_sim / T_s)) + 1))
        self.receive_time = np.zeros(shape=(1, int(math.ceil(T_sim / T_s)) + 1))
        super().__init__()

    def listen(self):
        l_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)   # UDP
        l_sock.setblocking(False)
        l_sock.bind((self.l_ip, self.l_port))
        print('Client Thread {} started >> Listening to port {}, sending to port {}.\n'.format(self.plant_id, self.l_port, self.s_port))
        thread_yield_duration_s = 0.0001  # see usage in the while loop, i.e., (sleep(thread_yield_duration_s))

        while True:
            try:
                data, addr = l_sock.recvfrom(1024)  # buffer size is 1024 bytes
                data = Protocol.decode_control(data)
                loop_id, seq_num, u = data
                if self.plant_id == loop_id:
                    self.receive_time[0, seq_num] = time()
                    self.update_control(seq_num, u)

            except socket.error as e:
                err = e.args[0]
                if err == errno.EAGAIN or err == errno.EWOULDBLOCK:
                    # No data available
                    # TODO This was the source of "STUCK GUI".
                    #  Using non-blocking socket in a while loop without yielding is problematic
                    #  Sleep solved the issue
                    sleep(thread_yield_duration_s)
                    continue
                else:
                    # a "real" error occurred
                    print("Server socket error: {}\n".format(e))
                    sleep(1)
                    sys.exit(1)

    def update_control(self, seq_num, u):
        self.Data.control.emit(seq_num, u)

    def send_state(self, msg, seq_num):
        self.send_time[0, seq_num] = time()
        self.s_sock.sendto(msg, self.s_dest)

    def send_exit(self):
        msg = Protocol.encode_exit_symbol()
        self.s_sock.sendto(msg, self.s_dest)

    def run(self):
        self.listen()


