# Copyright (C) 2020 Hasan Yagiz Özkan <yagiz.oezkan@tum.de>
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

import numpy as np
import datetime
import control
import control.matlab
import math
from config import COEFFICIENT_OF_FRICTION_FOR_CART as b
from config import MASS_MOMENT_OF_INERTIA_OF_THE_PENDULUM as I
from config import GRAVITATIONAL_COEFFICIENT as g
from config import MASS_OF_THE_PENDULUM as m
from config import MASS_OF_THE_CART as M
from config import LENGTH_OF_THE_PENDULUM as l
from config import SAMPLING_PERIOD_S as T_s
from config import SIMULATION_DURATION_SECONDS as T_sim
from config import Q as Q
from config import R as R
import config
import scipy
import os
import warnings
from pathlib import Path
import json
from multiprocessing import Process, Queue
import time
from protocol import Protocol
from queue import Empty
import struct


# TODO https://pyrpl.readthedocs.io/en/latest/developer_guide/api/asynchronous/benchmark.html


class InvertedPendulumProcess(Process):
    """
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
    """

    def __init__(self, loop_id: int, ctrl_to_comm_queue: Queue, ctrl_to_gui_queue: Queue, comm_to_ctrl_queue: Queue):
        # Process related initialization
        super().__init__(target=self.run, args=(None,))
        self.to_comm_queue = ctrl_to_comm_queue
        self.to_gui_queue = ctrl_to_gui_queue
        self.from_comm_queue = comm_to_ctrl_queue

        # Control related initialization
        self.init_model()
        self.loop_id = loop_id
        self.update_time = np.zeros(shape=(1, int(math.ceil(T_sim / T_s)) + 1))
        self.aoi_list = np.zeros(shape=(1, int(math.ceil(T_sim / T_s)) + 1))
        self.state_values = np.zeros(shape=(4, int(math.ceil(T_sim / T_s)) + 1))
        self.control_values = np.zeros(shape=(1, int(math.ceil(T_sim / T_s)) + 1))
        self.k = 0
        self.k_last = -1
        self.tx_cnt = 0
        self.rx_cnt = 0
        self.A_x = 0
        self.noise = 0
        self.new_control_pkt = False
        self.ready_flag = False
        self.u = np.zeros(shape=(1, 1))
        self.x = np.random.normal(0, self.std_dev_vector_w, size=(self.A.shape[0], 1))
        self.print(" is initialized")

    def print(self, txt, _end='\n'):
        print(f"{config.bcolors.INVPPROCESS} InvpP {self.loop_id}: {txt}{config.bcolors.ENDC}", end=_end)

    def check_for_received_packet(self) -> None:
        """
        Await Pipe.recv() for incoming packet / trigger with a send (coroutine) by the communication process
        :return: None
        """
        while True:
            # Infinite loop listening to unique (for this loop only) queue from communication process
            try:
                pkt = self.from_comm_queue.get_nowait()    # Non-blocking
                loop_id, seq_num, u = Protocol.decode_control(pkt)
                assert (loop_id == self.loop_id)
                self.update_control(seq_num, u)
            except Empty:
                # If there aren't any packets to process, task is done! Return from this function.
                return
            except struct.error as err:
                self.print(err)
                self.print(f"decode_control() failed due to unknown data: {pkt}")
                raise err

    def main_loop(self, num_steps: int) -> None:
        t_last = 0
        while self.k < num_steps:
            t_now = time.perf_counter()
            # Receive & process all buffered incoming packets
            self.check_for_received_packet()
            # Check if new sampling time has arrived, i.e., last sampling instance + sampling period (10ms)
            if t_now > t_last + T_s:
                # Record last sampling instance as now
                t_last = t_now
                # Update state here, i.e., sample the plant
                self.update_state()
                # Send state to controller via communication process and update the GUI process
                self.send_state()

            elif t_now < t_last + T_s - 0.002:
                time.sleep(0.001)

        self.emit_simulation_complete()
        self.print("completed measurement! Exiting...")

        return

    def run(self):
        number_of_timesteps = int(config.SIMULATION_DURATION_SECONDS * config.SAMPLING_FREQUENCY_HZ)
        self.main_loop(number_of_timesteps)

    """ CONTROL RELATED METHODS """

    def update_state(self):
        self.update_time[0, self.k] = time.perf_counter()
        if self.ready_flag:
            self.x = self.A_x + np.dot(self.B, self.u) + self.noise
        else:
            self.x = np.dot(self.A, self.x) + np.dot(self.B, self.u) + \
                 np.random.normal(0, self.std_dev_vector_w, size=(self.A.shape[0], 1))

        self.k += 1
        self.aoi_list[0, self.k] = self.k - self.k_last
        self.state_values[:, self.k] = self.x[:, 0]
        self.control_values[0, self.k] = self.u

    def update_control(self, seq_num, u):
        self.u = u
        self.rx_cnt += 1
        if seq_num > self.k_last:
            self.k_last = seq_num
        elif seq_num == self.k_last:
            print("equal")
        else:
            print("Out of order packet\n")
            warnings.warn("Out of order packet\n")

    def send_state(self):
        try:
            msg = Protocol.encode_state(self.loop_id, self.k, self.x)
            self.to_comm_queue.put(msg)             # Send to communication process
            if config.SHOW_GUI:
                self.to_gui_queue.put(msg)          # Send to gui process if GUI is running

            self.tx_cnt += 1
        except:
            self.print(f"i={self.loop_id}: Error while sending state")
            # TODO: sending location tx timestamp

    def calculate_Ax_w(self):
        self.A_x = np.dot(self.A, self.x)
        self.noise = np.random.normal(0, self.std_dev_vector_w, size=(self.A.shape[0], 1))
        self.ready_flag = True

    def init_model(self, sampling_period_s=T_s):

        self.A, self.B, self.C, self.D, self.K = InvertedPendulumProcess.generate_discrete_model()

        self.std_dev_vector_w = np.multiply(1.0, np.array([[0.8 * math.pow(10, -4)],
                                                           [0.7 * math.pow(10, -4)],
                                                           [math.radians(0.2)],
                                                           [math.radians(0.2)]]))

        self.var_vector = np.multiply(self.std_dev_vector_w, self.std_dev_vector_w)
        self.cov_matrix = np.multiply(np.eye(self.A.shape[0]), self.var_vector.T)
        self.W = self.cov_matrix

        self.x = np.random.normal(0, self.std_dev_vector_w, size=(self.A.shape[0], 1))

        self.u = np.zeros(shape=(self.B.shape[1], 1))

    @staticmethod
    def dlqr(A, B, Q, R):
        """
        Solve the discrete time lqr controller.
        x[k+1] = A x[k] + B u[k]
        cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]

        References:
        LQR controller: https://en.wikipedia.org/wiki/Linear%E2%80%93quadratic_regulator
        http://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=ControlDigital

        """

        # first, try to solve the Ricatti equation
        X = np.array(scipy.linalg.solve_discrete_are(A, B, Q, R))

        # compute the LQR gain
        K = np.array(scipy.linalg.inv(B.T * X * B + R) * (B.T * X * A))

        eigVals, eigVecs = scipy.linalg.eig(A - B * K)

        return K, X, eigVals

    @staticmethod
    def generate_discrete_model():

        p = I * (M + m) + M * m * math.pow(l, 2)  # helper var

        # Matrices
        A = np.array([[0, 1, 0, 0],
                      [0, -(I + m * math.pow(l, 2)) * b / p, (math.pow(m, 2) * g * math.pow(l, 2)) / p, 0],
                      [0, 0, 0, 1],
                      [0, -(m * l * b) / p, m * g * l * (M + m) / p, 0]])

        B = np.array([[0], [(I + m * math.pow(l, 2)) / p], [0], [m * l / p]])

        C = np.array([[1, 0, 0, 0], [0, 0, 1, 0]])
        #self.C = np.eye(4)

        # obsv_matrix = control.obsv(self.A, self.C)
        # ctrl_matrix = control.ctrb(A, B)
        # print(np.linalg.matrix_rank(obsv_matrix))
        # print(np.linalg.matrix_rank(ctrl_matrix))

        D = np.zeros((C.shape[0], 1))

        sys_ss = control.StateSpace(A, B, C, D)

        sys_d = control.matlab.c2d(sys_ss, T_s)
        # print('open loop discrete poles: {}'.format(control.pole(self.sys_d)))

        # Overwrite A, B, C, D with discrete time matrices
        A = sys_d.A
        B = sys_d.B
        C = sys_d.C
        D = sys_d.D

        # Solve Discrete Algebraic Riccati Equation
        K, X, eigVals = InvertedPendulumProcess.dlqr(A, B, Q, R)

        return np.copy(A), np.copy(B), np.copy(C), np.copy(D), np.copy(K)

    def emit_simulation_complete(self):
        # Send a single packet to corresponding queues with a special sequence number to inform them about completion
        msg = Protocol.encode_state(self.loop_id, config.SIMULATION_END_SEQ_NR, np.zeros(shape=self.x.shape))
        self.to_comm_queue.put(msg, block=True)  # Send to communication process
        if config.SHOW_GUI:
            self.to_gui_queue.put(msg, block=True)
        self.print("Simulation complete signal sent!")

    def log_control_results(self, folder_path) -> None:
        try:
            # Age of Information
            with open(folder_path + f"/Loop_{self.loop_id}/aoi.json", 'w', encoding='utf-8') as f:
                json.dump({'age of information': self.aoi_list[0].tolist()}, f, ensure_ascii=False, indent=4)

            # Update time
            with open(folder_path + f"/Loop_{self.loop_id}/update_time.json", 'w', encoding='utf-8') as f:
                json.dump({'update time': self.update_time[0].tolist()}, f, ensure_ascii=False, indent=4)

            # Plant state evolution
            with open(folder_path + f"/Loop_{self.loop_id}/plant_state.json", 'w', encoding='utf-8') as f:
                json.dump({'cart position': self.plant_values[0].tolist(), 'cart velocity': self.plant_values[1].tolist(),
                           'pendulum angle': self.plant_values[2].tolist(), 'angular speed': self.plant_values[3].tolist()},
                          f, ensure_ascii=False, indent=4)

            # Control input evolution
            with open(folder_path + f"/Loop_{self.loop_id}/control_state.json", 'w', encoding='utf-8') as f:
                json.dump({'control value': self.control_values[0].tolist()}, f, ensure_ascii=False, indent=4)

        finally:
            self.print("Logging completed")

    def create_log_folders(self, measurement_folder_path: str) -> None:
        try:
            os.makedirs(measurement_folder_path + f"/Loop_{self.loop_id}")
        except:
            self.print("Loop folder could not be created!")
            raise


