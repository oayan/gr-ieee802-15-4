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

import socket, time, sys
import threading
import config
from protocol import Protocol
from controller import Controller
import errno
from time import sleep


def return_control_signal(data, send_sock, SEND_PORT, controller):
    loop_id, seq_num, x = data
    IP = "127.0.0.1"

    u = controller.get_control_input(x, seq_num)

    msg = Protocol.encode_control(loop_id, seq_num, u)
    send_sock.sendto(msg, (IP, SEND_PORT))


def controller_thread(l_port, s_port):
    IP = "127.0.0.1"    # localhost

    thread_yield_duration_s = 0.0002    # see usage in the while loop, i.e., (sleep(thread_yield_duration_s))

    l_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)    # UDP Listener Socket
    l_sock.setblocking(False)
    l_sock.bind((IP, l_port))

    s_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)    # UDP Sender Socket
    s_sock.setblocking(False)

    print('Controller Thread {} started >> Listening to port {}, sending to port {}.\n'.format(server_id,
                                                                                               PORTS[PortPair.listenPort],
                                                                                               PORTS[PortPair.sendPort]))
    controller = Controller()

    while True:
        try:
            data, addr = l_sock.recvfrom(1024)      # buffer size is 1024 bytes
            data = Protocol.decode_state(data)
        except socket.error as e:
            err = e.args[0]
            if err == errno.EAGAIN or err == errno.EWOULDBLOCK:
                # No data available, yield thread to other threads
                sleep(thread_yield_duration_s)  # TODO Check if this affects a lot?!
                continue
            else:
                # a "real" error occurred
                print("Server socket error: {}\n".format(e))
                sleep(1)
                sys.exit(1)

        t = threading.Thread(target=return_control_signal,
                             args=[data, s_sock, s_port, controller])
        t.start()


class PortPair:
    listenPort = 0
    sendPort = 1


if __name__ == "__main__":
    S_PORT = list(range(config.CONTROLLER_SEND_PORT_START, config.CONTROLLER_SEND_PORT_STOP))
    L_PORT = list(range(config.CONTROLLER_LISTEN_PORT_START, config.CONTROLLER_LISTEN_PORT_STOP))
    for server_id, PORTS in enumerate(zip(L_PORT, S_PORT)):
        print('Server {} started >> Listening to port {}, sending to port {}.\n'.format(server_id,
                                                                                      PORTS[PortPair.listenPort],
                                                                                      PORTS[PortPair.sendPort]))

        t = threading.Thread(target=controller_thread, args=PORTS)
        t.start()
        sleep(0.1)

