import numpy as np
import time
from communicationprocess import CommunicationProcess
from invertedpendulumprocess import InvertedPendulumProcess
from guiprocess import GUIProcess
from multiprocessing import Queue
from config import NUMBER_OF_LOOPS as N
from config import SHOW_GUI
from time import sleep
import sys
import asyncio


def main():
    np.random.seed(int(time.time()))

    ctrl_to_comm_queue = Queue()    # Create control-to-communication queue x 1
    comm_to_ctrl_queues = {}        # Create communication-to-control asyncio.Queue x N
    ctrl_to_gui_queue = Queue()     # Create control-to-GUI queue x 1

    for i in range(1, N + 1):   # 1, 2, ..., N
        try:
            assert (i not in comm_to_ctrl_queues)
            comm_to_ctrl_queues[i] = Queue()
        except AssertionError as err:
            print("Assertion failed")
            raise err
            exit(1)

    communication_process = CommunicationProcess(ctrl_to_comm_queue, comm_to_ctrl_queues)
    sleep(0.1)

    # ----- Create N control processes with corresponding queues passed as constructor argument
    control_processes = {}
    for i in range(1, N + 1):   # i = 1, 2, ..., N
        try:
            assert(i not in control_processes)
            control_processes[i] = InvertedPendulumProcess(i, ctrl_to_comm_queue, ctrl_to_gui_queue, comm_to_ctrl_queues[i])
        except KeyError as err:
            print(err)
            print("main_plant.py: Key not found")
            raise KeyError
        except AssertionError as err:
            print("Assertion failed")
            raise err
            exit(1)

    if SHOW_GUI:
        # If GUI is active, create the GUI process by passing the control to gui queue (multiprocessing.Queue)
        gui_proces = GUIProcess(ctrl_to_gui_queue)

    # Start all processes & join all processes
    communication_process.start()
    if SHOW_GUI:
        gui_proces.start()
    for i, p in control_processes.items():
        p.start()

    for i, p in control_processes.items():
        p.join()

    communication_process.join()
    if SHOW_GUI:
        gui_proces.join()
    print("Successful completion of the measurement! All processes finished running. Exiting...")
    exit(0)


if __name__ == "__main__":
    main()
