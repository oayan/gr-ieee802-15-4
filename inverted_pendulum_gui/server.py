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
import select



class PortPair:
    listenPort = 0
    sendPort = 1


class ControlLoopServer:
    addr = '127.0.0.1'
    id = 0
    controller = Controller()

    def __init__(self, listen_port, send_port):

        ControlLoopServer.id += 1   # Be careful here: LoopIds might not be in the same order with socket creating

        self.l_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)      # UDP Listener Socket
        self.l_port = listen_port
        self.s_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP Listener Socket
        self.s_port = send_port


        print(f'Server {ControlLoopServer.id} started >> Listening to port {l_port}, '
              f'sending to port {s_port}.\n')

    def bind_to_listen_port(self):
        self.l_sock.bind(('127.0.0.1', self.l_port))


def run(control_loop_servers):

    listener_sockets = {}
    sender_sockets = {}

    for cls in control_loop_servers:
        cls.bind_to_listen_port()
        listener_sockets[cls.l_sock] = cls
        sender_sockets[cls.s_sock] = cls


    while True:
        # Await a read event
        rlist, _, _ = select.select([*listener_sockets], [], [], 5)

        # Test for timeout
        if rlist == []:
            print('.')
            continue
        elif rlist:
            #t = time.time()
            _, wlist, _ = select.select([], [*sender_sockets], [], 5)
            # Loop through each socket in rlist, read and print the available data
            for sock in rlist:
                cls = listener_sockets[sock]
                if cls.s_sock in wlist:
                    data, addr = sock.recvfrom(100)  # buffer size is 100 bytes
                    loop_id, seq_num, x = Protocol.decode_state(data)
                    u = ControlLoopServer.controller.get_control_input(x, seq_num)

                    msg = Protocol.encode_control(loop_id, seq_num, u)
                    cls.s_sock.sendto(msg, (ControlLoopServer.addr, cls.s_port))
                    #print(time.time() - t)


if __name__ == "__main__":
    control_loop_servers = []
    s_ports = list(range(config.CONTROLLER_SEND_PORT_START, config.CONTROLLER_SEND_PORT_STOP))
    l_ports = list(range(config.CONTROLLER_LISTEN_PORT_START, config.CONTROLLER_LISTEN_PORT_STOP))

    for server_id, ports in enumerate(zip(l_ports, s_ports)):
        l_port = ports[PortPair.listenPort]
        s_port = ports[PortPair.sendPort]

        control_loop_servers.append(ControlLoopServer(listen_port=l_port, send_port=s_port))

    sleep(0.5)
    run(control_loop_servers)
