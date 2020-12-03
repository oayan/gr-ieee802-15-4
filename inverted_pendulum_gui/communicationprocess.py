from multiprocessing import Process, Queue
import time
import asyncio


class FirstProcess(Process):

    def __init__(self, queue):
        super().__init__(target=self.run, args=(None,))
        self.queue = queue

    async def read(self, queue) -> None:
        while True:
            num = await queue.get()
            print('waiting For sleep', num)
        return

    async def write(self, queue) -> None:
        t_last = 0
        while True:
            t_now = time.perf_counter()  # ...
            if t_now > t_last + 0.1:
                t_last = t_now
                print(t_now)
                await queue.put(t_now)
            elif t_now < t_last + 0.01 - 0.002:

                await asyncio.sleep(0.001)

        print("finished")
        return

    async def main(self):
        aio_queue = asyncio.Queue()
        await asyncio.gather(self.read(aio_queue), self.write(aio_queue))

    def run(self):
        p_name = self.name
        asyncio.run(self.main())


if __name__ == '__main__':
    p_queue = Queue
    p = FirstProcess(p_queue)
    p2 = FirstProcess(p_queue)
    # reader_p.daemon = True
    p.start()
    p2.start()
