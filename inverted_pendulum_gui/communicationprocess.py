from multiprocessing import Process, Queue, Lock
from protocol import Protocol, P2C_PACKET_SIZE_bytes
import select
import socket
from config import NUMBER_OF_LOOPS as N
import config
import collections
from queue import Empty
import logging
import sys
import struct
import asyncio


class CPSocket(socket.socket):
    def __init__(self, handler):
        super().__init__(socket.AF_INET, socket.SOCK_DGRAM)
        self.ctrl_proc_handler = handler


class ControlProcessHandler:
    def __init__(self, _loop_id, _l_port, _s_port, _queue: asyncio.Queue):
        self.loop_id = _loop_id
        self.l_port = _l_port
        self.s_port = _s_port
        self.to_ctrl_queue = _queue

        self.l_sock = CPSocket(self)      # UDP Listener Socket
        self.s_sock = CPSocket(self)      # UDP Sender Socket

    def bind_to_listen_port(self):
        self.l_sock.bind(('127.0.0.1', self.l_port))

    def close_socket_connection(self):
        self.l_sock.close()
        self.s_sock.close()
        socket_msg = "sockets of loop " + str(self.loop_id) + " is removed"
        CommunicationProcess.print(socket_msg)
        pass


class CommunicationProcess(Process):
    """
    CommunicationProcess class that serves as the gateway of all control loops to outside world.
    It is responsible for reading and writing sockets that are GUI's entry point to GNURadio.
    There is only one communication process to manage the inbound/outbound traffic of multiple loops.
    It directs the packets to the right socket/process.

    - Each control process (e.g. InvertedPendulumProcess) is connected to CommunicationProcess through:
     -- a queue to write: N control processes-- 1 x queue --> 1 communication process
     -- N asyncio.Queue to read: 1 communication process -- N x queues --> N control processes
    """
    def __init__(self, _ctrl_to_comm_queue: Queue, _comm_to_ctrl_queues: dict, _logging_lock: Lock, cpu_bound: bool = True):
        if cpu_bound:
            run = self.run
        else:
            #TODO asyncio implementation
            self.print('asyncio implementation!')
            sys.exit(1)

        super().__init__(target=run, args=(None,))
        self.from_control_queue = _ctrl_to_comm_queue
        self.to_control_queues = _comm_to_ctrl_queues
        self.log_lock = _logging_lock

        # Create ports for sockets
        # Communication process decides which socket belongs to which loop. Not the main anymore!
        s_ports = collections.deque(range(config.PLANT_SEND_PORT_START, config.PLANT_SEND_PORT_STOP))
        l_ports = collections.deque(range(config.PLANT_LISTEN_PORT_START, config.PLANT_LISTEN_PORT_STOP))

        self.ctrl_proc_handlers = {}

        for i in range(1, N + 1):  # 1, 2, ..., N
            assert(i not in self.ctrl_proc_handlers)
            self.ctrl_proc_handlers[i] = ControlProcessHandler(i, l_ports.popleft(), s_ports.popleft(),
                                                               self.to_control_queues[i])

        self.print_socket_configuration()

    @classmethod
    def print(cls, txt, _end='\n', _name='CommP:'):
        print(f"{config.bcolors.COMMPROCESS}{_name} {txt}{config.bcolors.ENDC}", end=_end)

    def print_socket_configuration(self):
        blank_space = 6 * ' '
        CommunicationProcess.print(txt='Sockets selected as:')
        for id, cph in self.ctrl_proc_handlers.items():
            CommunicationProcess.print(f'{blank_space}i = {id} --> Listens: {cph.l_port}, Sends: {cph.s_port}', _name='')

    def run(self):
        listener_sockets = []
        sender_sockets = []

        for i, cph in self.ctrl_proc_handlers.items():
            cph.bind_to_listen_port()
            listener_sockets.append(cph.l_sock)
            sender_sockets.append(cph.s_sock)

        while True:
            # Get ready reader sockets, non-blocking
            rlist, _, _ = select.select(listener_sockets, [], [], 0)
            for sock in rlist:
                try:
                    data, addr = sock.recvfrom(128)     # buffer size is 128 bytes, should not block due to select
                    loop_id, _, _ = Protocol.decode_control(data)   # extract loop_id out of packet
                    assert (loop_id == sock.ctrl_proc_handler.loop_id)  # make sure that loop_id matches socket's
                    cph = sock.ctrl_proc_handler            # get handler of the control process matching loop_id
                    cph.to_ctrl_queue.put(data)             # forward packet via correct pipe towards control process
                except socket.timeout as err:
                    logging.error(err)
                    self.print("This should not happen, check implementation!")
                    continue

                except socket.error as err:
                    logging.error(err)
                    sys.exit(1)

                except struct.error as err:
                    print(err)
                    self.print(f"decode_control() failed due to unknown data: {data}")
                    continue
                except AssertionError as err:
                    self.print("Assertion falied", err)
                except:
                    self.print("Unknown error while unpacking!")
                    sys.exit(1)

            # Consume outgoing packets from the queue
            try:
                msg = self.from_control_queue.get_nowait()
                loop_id, seq_nr, _ = Protocol.decode_state(msg)
                if seq_nr == config.SIMULATION_END_SEQ_NR:
                    l_sock = self.ctrl_proc_handlers[loop_id].l_sock
                    listener_sockets.remove(l_sock)
                    self.ctrl_proc_handlers[loop_id].close_socket_connection()
                    self.print("Simulation complete signal received")
                else:
                    cph = self.ctrl_proc_handlers[loop_id]
                    sock = self.ctrl_proc_handlers[loop_id].s_sock
                    tx_bytes = sock.sendto(msg, ('127.0.0.1', cph.s_port))   # GNURadio is running on the same machine

                    if tx_bytes != P2C_PACKET_SIZE_bytes:
                        print(f"{config.bcolors.WARNING}'TX Bytes not matching'{config.bcolors.ENDC}")
                        self.from_control_queue.put(msg)    # Put packet back to the queue since it could not be sent

            except Empty:
                continue

            except struct.error as err:
                print(err)
                self.print(f"decode_state() failed due to unknown data: {data}")
                continue

    def log_communication_results(self):
        pass


if __name__ == '__main__':
    CommunicationProcess.print("Run from main_plant.py")
    exit(1)
