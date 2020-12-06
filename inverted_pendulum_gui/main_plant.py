import numpy as np
import time
from communicationprocess import CommunicationProcess
from invertedpendulumprocess import InvertedPendulumProcess
from guiprocess import GUIProcess
from multiprocessing import Queue, Pipe
from config import NUMBER_OF_LOOPS as N
from time import  sleep

def main():
    np.random.seed(int(time.time()))

    ctrl_to_comm_queue = Queue()  # Create control-to-communication queue x 1

    # Create communication-to-control pipe x N
    comm_to_ctrl_pipes = {}
    for i in range(1, N + 1):   # 1, 2, ..., N
        comm_to_ctrl_pipes[i] = Pipe()

    # Create communication process with:
    # - queue
    # - pipes as dict: {key, pipe}
    p_comm = CommunicationProcess(ctrl_to_comm_queue, comm_to_ctrl_pipes)
    sleep(0.5)
    p_comm.run()
    p_comm.join()
    exit(1)
    ctrl_to_gui_queue = Queue()  # Create control-to-GUI queue x 1

    # ----- Create N control processes with queues and pipe

    # ----- Create GUI process with queue
    p_gui = GUIProcess()

    # Start all processes & join all processes


if __name__ == "__main__":
    main()
