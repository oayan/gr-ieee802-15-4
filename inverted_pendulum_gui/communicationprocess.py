from multiprocessing import Process, Queue
import time
import select


class CommunicationProcess(Process):
    """
    CommunicationProcess class that serves as the gateway of all control loops to outside world.
    It is responsible for reading and writing sockets that are GUI's entry point to GNURadio.
    There is only one communication process to manage the inbound/outbound traffic of multiple loops.
    It directs the packets to the right socket / process.

    - Each control process(e.g. InvertedPendulumProcess is connected to CommunicationProcess through:
     -- a queue to write: N control processes-- 1 x queue --> 1 communication process
     -- N pipes to read: 1 communication process -- N x pipes --> N control processes
    """

    def __init__(self, ctrl_to_comm_queue: Queue, comm_to_ctrl_pipes: dict):
        super().__init__(target=self.run, args=(None,))

        self.from_control_queue = ctrl_to_comm_queue
        self.to_control_pipes = comm_to_ctrl_pipes

        # TODO for each key in pipes, create an handler below with the correct socket (read from config accordingly)
        # Communication process knows which socket belongs to which loop.
        # Not the main anymore! Sockets are created here!

    class ControlProcessHandler:
        def __init__(self):
            self.loop_id = 0
            self.l_port = None
            self.s_port = None

        def bind_to_listen_port(self):
            self.l_sock.bind(('127.0.0.1', self.l_port))


    def run(self):
        p_name = self.name

        while True:
            pass

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
    pass
    # p_queue = Queue()
    # p = CommunicationProcess(p_queue)
    # p2 = CommunicationProcess(p_queue)
    # # reader_p.daemon = True
    # p.start()
    # p2.start()
