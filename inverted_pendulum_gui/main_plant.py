import numpy as np
import time
from communicationprocess import CommunicationProcess
from invertedpendulumprocess import InvertedPendulumProcess
from guiprocess import GUIProcess
from multiprocessing import Queue, Lock
from config import NUMBER_OF_LOOPS as N
from config import SHOW_GUI
from config import strategy_path
from time import sleep
from datetime import datetime
import os
import json
import config


def prepare_log_folders() -> str:
    last_log = 0
    try:
        if not os.path.exists(strategy_path):
            try:
                os.makedirs(strategy_path)
            except:
                print("Folder could not be created!")
                raise
        for folder in strategy_path.iterdir():
            if folder.is_dir():
                if "Measurement_" in folder.stem:
                    log_num = folder.stem.replace('Measurement_', '')
                    if last_log < int(log_num):
                        last_log = int(log_num)

        measurement_folder_path = str(strategy_path) + "/Measurement_{}/".format(last_log + 1)
        time.sleep(0.1)
        os.makedirs(measurement_folder_path)
    except:
        print("Measurement folder could not be created!")

    with open(measurement_folder_path + "config.json", 'w', encoding='utf-8') as f:
        json.dump({
            'log time': datetime.now().strftime("%Y-%m-%d_%H:%M:%S"),
            'number of loops': config.NUMBER_OF_LOOPS,
            'simulation duration': config.SIMULATION_DURATION_SECONDS,
            'sampling period': config.SAMPLING_PERIOD_S,
            'Strategy': config.STRATEGY,
            'GUI Enable': config.SHOW_GUI,
            'Q': config.Q.flatten().tolist(),
            'R': config.R.flatten().tolist()
        },
            f, ensure_ascii=False, indent=4)

    return measurement_folder_path


def main():
    np.random.seed(int(time.time()))

    measurement_folder = prepare_log_folders()
    logging_lock = Lock()
    ctrl_to_comm_queue = Queue()    # Create control-to-communication queue x 1
    comm_to_ctrl_queues = {}        # Create communication-to-control Queue x N
    ctrl_to_gui_queue = Queue()     # Create control-to-GUI queue x 1

    for i in range(1, N + 1):  # 1, 2, ..., N
        try:
            assert (i not in comm_to_ctrl_queues)
            comm_to_ctrl_queues[i] = Queue()
        except AssertionError as err:
            print("Assertion failed")
            raise err
            exit(1)

    communication_process = CommunicationProcess(ctrl_to_comm_queue, comm_to_ctrl_queues, True)
    setattr(communication_process, "measurement_folder", measurement_folder)
    sleep(0.1)

    # ----- Create N control processes with corresponding queues passed as constructor argument
    control_processes = {}
    for i in range(1, N + 1):   # i = 1, 2, ..., N
        try:
            assert(i not in control_processes)
            control_processes[i] = InvertedPendulumProcess(i, ctrl_to_comm_queue, ctrl_to_gui_queue, comm_to_ctrl_queues[i], logging_lock)
            control_processes[i].create_log_folders(measurement_folder)
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
