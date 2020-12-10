# Copyright (C) 2020 Jiayu Zhong
#               2020 Hasan Yagiz Özkan <yagiz.oezkan@tum.de>
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
# from PySide2.QtCore import QThread, Signal, Slot, QObject
import struct

# Plant-To-Controller Packet Format
P2C_PACKET_FORMAT = 'iiffff'
P2C_PACKET_SIZE_bytes = struct.calcsize(P2C_PACKET_FORMAT)


# Controller-To-Plant Packet Format
C2P_PACKET_FORMAT = 'iif'
C2P_PACKET_SIZE_bytes = struct.calcsize(C2P_PACKET_FORMAT)

# Plant-End-Signal Packet Format
P_EXIT_FORMAT = 'i'
P_EXIT_SIZE_bytes = struct.calcsize(P_EXIT_FORMAT)
P_EXIT_SYMBOL = 0


class Protocol:

    @staticmethod
    def encode_exit_symbol():
        tx_data = [0] * len(P_EXIT_FORMAT)
        tx_data[ExitPacketField.exit_symbol] = P_EXIT_SYMBOL
        return struct.pack(P_EXIT_FORMAT, *tx_data)

    @staticmethod
    def encode_state(loop_id, seq, state: np.array):
        tx_data = [0] * len(P2C_PACKET_FORMAT)
        tx_data[PlantToControllerPacketFields.loop_id] = loop_id
        tx_data[PlantToControllerPacketFields.seq_num] = seq
        tx_data[PlantToControllerPacketFields.cart_position] = state[0, 0]
        tx_data[PlantToControllerPacketFields.cart_speed] = state[1, 0]
        tx_data[PlantToControllerPacketFields.pendulum_angle] = state[2, 0]
        tx_data[PlantToControllerPacketFields.pendulum_speed] = state[3, 0]
        return struct.pack(P2C_PACKET_FORMAT, *tx_data)

    @staticmethod
    def decode_state(rx_data):
        pkt = struct.unpack(P2C_PACKET_FORMAT, rx_data)
        loop_id = pkt[PlantToControllerPacketFields.loop_id]
        seq_num = pkt[PlantToControllerPacketFields.seq_num]
        cart_position = pkt[PlantToControllerPacketFields.cart_position]
        cart_speed = pkt[PlantToControllerPacketFields.cart_speed]
        pendulum_angle = pkt[PlantToControllerPacketFields.pendulum_angle]
        pendulum_speed = pkt[PlantToControllerPacketFields.pendulum_speed]
        state = np.array([[cart_position], [cart_speed], [pendulum_angle], [pendulum_speed]])
        return [loop_id, seq_num, state]
    
    @staticmethod
    def encode_control(loop_id: int, seq: int, u: np.array):
        tx_data = [0] * len(C2P_PACKET_FORMAT)
        tx_data[ControllerToPlantPacketFields.loop_id] = loop_id
        tx_data[ControllerToPlantPacketFields.seq_num] = seq
        tx_data[ControllerToPlantPacketFields.control_input] = u[0, 0]
        return struct.pack(C2P_PACKET_FORMAT, *tx_data)
    
    @staticmethod
    def decode_control(rx_data):
        pkt = struct.unpack(C2P_PACKET_FORMAT, rx_data)
        loop_id = pkt[ControllerToPlantPacketFields.loop_id]
        seq_num = pkt[ControllerToPlantPacketFields.seq_num]
        u = pkt[ControllerToPlantPacketFields.control_input]
        return [loop_id, seq_num, u]

#
# class Data(QObject):
#     #data= [time_step,state]
#     data = Signal(list)
#     state = Signal(bytes, int)
#     seq = Signal(float)
#     control = Signal(int, float)
#     stop = Signal(int)


class PlantToControllerPacketFields:
    loop_id = 0
    seq_num = 1
    cart_position = 2
    cart_speed = 3
    pendulum_angle = 4
    pendulum_speed = 5


class ControllerToPlantPacketFields:
    loop_id = 0
    seq_num = 1
    control_input = 2


class ExitPacketField:
    exit_symbol = 0

