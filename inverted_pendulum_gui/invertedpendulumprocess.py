from multiprocessing import Process, Queue
import time
import asyncio
from concurrent.futures import ProcessPoolExecutor


# TODO https://pyrpl.readthedocs.io/en/latest/developer_guide/api/asynchronous/benchmark.html

def recur_fibo(n):
    if n <= 1:
        return n
    else:
        return (recur_fibo(n - 1) + recur_fibo(n - 2))


def prepare_next_state() -> int:
    return recur_fibo(30)


class InvertedPendulumProcess(Process):
    '''
    Inverted Pendulum Wrapper Class as A Process. It contains the control model and client socket to receive/send packets.
    Communicates with the (main) Application Window Process through queue / pipe?

    ------------------------
    | Communication Process|                                                     _________________________
    --------- v -----------                                                     |ApplicationWindowProcess|
              |                                                                 ----^--^-----------------
              |      __________________________                                    /  /
               ---> |InvertedPendulumProcess 1 |: Client --> Model --> State ---  /  /
              |     ---------------------------                                     /
              |      __________________________                                    /
               ---> |InvertedPendulumProcess 2 |: Client --> Model --> State ---  /
                    ---------------------------
    Perhaps, this Process should have its own logger
    '''

    def __init__(self):
        super().__init__(target=self.run, args=(None,))

    async def wait_for_received_packet(self) -> None:
        '''
        Await queue for incoming packet / trigger with a send (coroutine) by the communication process
        :return:
        '''
        while True:
            print('waiting For sleep', time.perf_counter())
            await asyncio.sleep(1)
            if False:
                break
        return

    async def run(self) -> None:
        asyncio_loop = asyncio.get_running_loop()
        t_last = 0
        flag = False
        queue = asyncio.Queue()
        while True:
            # TODO use time.perf_counter()

            t_now = time.perf_counter() # ...

            # Check if new sampling time has arrived --> sample -- > send through client socket
            #     --> Add to queue/pipe to be processed by the GUI process
            #     --> Post processing, e.g. logging
            #
            if t_now > t_last + 0.01:
                # Update state here
                # wait for the next state to be available from the worker process (reception through queue / pipe)
                t_last = t_now
                print(t_now)
                flag = False
            elif t_now < t_last + 0.01 - 0.002:
                # TODO https://stackoverflow.com/questions/51117209/combining-asyncio-with-a-multi-worker-processpoolexecutor
                # TODO create a worker process at the beginning, and all loops delegate tasks to it and receive back
                await asyncio.sleep(0.001)
                # if not flag:
                #     with ProcessPoolExecutor() as executor:
                #         t0 = time.perf_counter()
                #         result = await asyncio_loop.run_in_executor(executor, prepare_next_state)
                #         print('custom thread pool', result, time.perf_counter() - t0)
                #         flag = True
                # else:
                #     await asyncio.sleep(0.001)


        # TODO run wait_for_received_packet as thread
        print("finished")
        return


async def main():
    p = InvertedPendulumProcess()
    await asyncio.gather(p.run(), p.wait_for_received_packet())

asyncio.run(main())




