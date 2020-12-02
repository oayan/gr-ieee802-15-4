# workerprocesspool.py

import time
from multiprocessing import Process, Queue
from queue import Empty


def run(pid : int, q : Queue) -> None:
    while True:
        try:
            d = q.get_nowait()
        except Empty:
            d = None

        if not d:
            break

        print(f'Process {pid} is sleeping for {d} seconds')
        time.sleep(d)
        print(f'{pid} finished')
    return


t_now = time.perf_counter()
processes = []
q = Queue()

for i in range(1, 5 + 1):
    q.put(i)

for pid in range(1, 2 + 1):
    p = Process(target=run, args=(pid, q))
    processes.append(p)
    p.start()

for p in processes:
    p.join()

print(f'Code completion time: {time.perf_counter() - t_now} seconds')