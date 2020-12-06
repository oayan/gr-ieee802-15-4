from multiprocessing import Process, Queue
import time
from protocol import Protocol, P2C_PACKET_SIZE_bytes

import select
import socket
from config import NUMBER_OF_LOOPS as N
import config
import collections
from queue import Empty


class CPSocket(socket.socket):
    def __init__(self, handler):
        super().__init__(socket.AF_INET, socket.SOCK_DGRAM)
        self.ctrl_proc_handler = handler


class ControlProcessHandler:
    def __init__(self, _loop_id, _l_port, _s_port, _pipe):
        self.loop_id = _loop_id
        self.l_port = _l_port
        self.s_port = _s_port
        self.s_ip = None # TODO FILL THIS !
        self.to_ctrl_pipe = _pipe

        self.l_sock = CPSocket(self)      # UDP Listener Socket
        self.s_sock = CPSocket(self)      # UDP Sender Socket

    def bind_to_listen_port(self):
        self.l_sock.bind(('127.0.0.1', self.l_port))


class CommunicationProcess(Process):
    """
    CommunicationProcess class that serves as the gateway of all control loops to outside world.
    It is responsible for reading and writing sockets that are GUI's entry point to GNURadio.
    There is only one communication process to manage the inbound/outbound traffic of multiple loops.
    It directs the packets to the right socket/process.

    - Each control process (e.g. InvertedPendulumProcess) is connected to CommunicationProcess through:
     -- a queue to write: N control processes-- 1 x queue --> 1 communication process
     -- N pipes to read: 1 communication process -- N x pipes --> N control processes
    """
    def __init__(self, _ctrl_to_comm_queue: Queue, _comm_to_ctrl_pipes: dict):
        super().__init__(target=self.run, args=(None,))

        self.from_control_queue = _ctrl_to_comm_queue
        self.to_control_pipes = _comm_to_ctrl_pipes

        # TODO for each key in pipes, create an handler below with the correct socket (read from config accordingly)
        # Create ports for sockets
        # Communication process decides which socket belongs to which loop. Not the main anymore!
        s_ports = collections.deque(range(config.PLANT_SEND_PORT_START, config.PLANT_SEND_PORT_STOP))
        l_ports = collections.deque(range(config.PLANT_LISTEN_PORT_START, config.PLANT_LISTEN_PORT_STOP))

        self.ctrl_proc_handlers = {}

        for i in range(1, N + 1):  # 1, 2, ..., N
            assert(i not in self.ctrl_proc_handlers)
            self.ctrl_proc_handlers[i] = ControlProcessHandler(i, l_ports.popleft(), s_ports.popleft(),
                                                               self.to_control_pipes[i])

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
                data, addr = sock.recvfrom(100)  # buffer size is 100 bytes, should not block due to select
                loop_id, _, _ = Protocol.decode_control(data)
                assert (loop_id == sock.ctrl_proc_handler.loop_id)
                cph = sock.ctrl_proc_handler[loop_id]
                # TODO deliver this packet to the corresponding loop through cph's pipe

            # Consume outgoing packets from the queue
            try:
                msg = self.from_control_queue.get_nowait()
                loop_id, _, _ = Protocol.decode_state()
                cph = self.ctrl_proc_handlers[loop_id]
                # TODO ip is still None
                tx_bytes = self.ctrl_proc_handlers[loop_id].s_sock.sendto(msg, (cph.s_ip, cph.s_port))

                if tx_bytes != P2C_PACKET_SIZE_bytes:
                    print(f"{config.bcolors.WARNING}'TX Bytes not matching'{config.bcolors.ENDC}")
                    self.from_control_queue.put(msg)    # Put packet back to the queue since it could not be sent

            except Empty:
                pass
            else:
                print('Unexpected Error')
                raise

    # async def main(self):
    #     aio_queue = asyncio.Queue()
    #     await asyncio.gather(self.read(aio_queue), self.write(aio_queue))
    #
    #
    # async def read(self, queue) -> None:
    #     while True:
    #         num = await queue.get()
    #         print('waiting For sleep', num)
    #     return
    #
    # async def write(self, queue) -> None:
    #     t_last = 0
    #     while True:
    #         t_now = time.perf_counter()  # ...
    #         if t_now > t_last + 0.1:
    #             t_last = t_now
    #             print(t_now)
    #             await queue.put(t_now)
    #         elif t_now < t_last + 0.01 - 0.002:
    #
    #             await asyncio.sleep(0.001)
    #
    #     print("finished")
    #     return


if __name__ == '__main__':
    exit(1)

    # p_queue = Queue()
    # p = CommunicationProcess(p_queue)
    # p2 = CommunicationProcess(p_queue)
    # # reader_p.daemon = True
    # p.start()
    # p2.start()
