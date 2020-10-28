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

from model import InvertedPendulum
from config import REFERENCE_VALUE as r
from config import PRECOMPENSATOR as Nbar
import numpy as np
import math


class Controller:
    def __init__(self):
        self.A, self.B, self.C, self.D, self.K = InvertedPendulum.generate_discrete_model()
        self.r = r
        for i in range(self.r.shape[1]):
            if math.floor(i / 500) % 2:
                self.r[0, i] = 0.0
        self.Nbar = Nbar

    def get_control_input(self, x: np.array, seq_num):
        return np.dot(self.r[0, seq_num], self.Nbar) - np.dot(self.K, x)



