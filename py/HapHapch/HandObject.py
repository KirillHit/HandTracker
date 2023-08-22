# -*- coding: utf-8 -*-

import numpy as np
import time


class HandTracker:
    def __init__(self, radius=60):
        self.SaveSize = None
        self.TrackingProcess = False

        self.height = 1
        self.width = 1
        self.Radius = radius
        self.CalibDist = 500
        self.CalibCam = 1

        self.StartTime = 0
        self.PrevTime = 0
        self.CalibTimer = 2

        self.FixedParam = 3
        self.FixedParam_Z = 14
        self.averaging_len = 30
        self.hand_mass = np.array([[0] * 3] * self.averaging_len, dtype=np.single)

        self.Hand = [0, 0, 0]
        self.Compress = False

        self.Change_XY = False
        self.Inv_X = False
        self.Inv_Y = False

    def stop_tracking(self):
        self.TrackingProcess = False
        self.StartTime = 0

    def give_Hand(self, center, size_factor, compress):
        now = time.time_ns()

        # Калибровка размеров руки
        if not self.TrackingProcess:
            if (center[0] ** 2 + center[1] ** 2) <= self.Radius ** 2:
                if self.StartTime == 0:
                    self.StartTime = now
                    self.SaveSize = np.array([size_factor])
                else:
                    self.SaveSize = np.append(self.SaveSize, size_factor)
                    if (now - self.StartTime) >= self.CalibTimer * 10 ** 9:
                        self.TrackingProcess = True
                        self.StartTime = 0
                        self.SaveSize = np.mean(self.SaveSize, dtype=np.single)
                        self.hand_mass = np.array([[0]*3]*self.averaging_len, dtype=np.single)
                        return "Success"
            else:
                self.StartTime = 0
            return "Calibration"
        # Отслеживание руки
        else:
            self.cam_z = self.CalibDist * self.SaveSize / size_factor

            self.Real_z = self.CalibDist - self.cam_z
            self.Real_x = self.CalibCam * self.cam_z * center[0] / self.CalibDist
            self.Real_y = self.CalibCam * self.cam_z * center[1] / self.CalibDist

            '''
            if (abs(self.Fixed_z - self.Real_z)) > self.FixedParam_Z:
                self.Fixed_z = self.Real_z
            if (abs(self.Fixed_y - self.Real_y)) > self.FixedParam:
                self.Fixed_y = self.Real_y
            if (abs(self.Fixed_x - self.Real_x)) > self.FixedParam:
                self.Fixed_x = self.Real_x
            '''

            self.Hand = [self.Real_x, self.Real_y]
            if self.Change_XY:
                self.Hand.reverse()
            if self.Inv_X:
                self.Hand[0] = -1 * self.Hand[0]
            if self.Inv_Y:
                self.Hand[1] = -1 * self.Hand[1]
            self.Hand.append(self.Real_z)
            self.Compress = compress

            self.hand_mass = np.append(self.hand_mass, [self.Hand], axis=0)
            self.hand_mass = self.hand_mass[-self.averaging_len:]
            # Dm = np.absolute(self.Hand - np.mean(self.hand_mass))
            #if Dm[0] < self.FixedParam or Dm[1] < self.FixedParam or Dm[2] < self.FixedParam_Z:
                # self.Hand = np.mean(self.hand_mass, axis=0)
            self.Hand = np.average(self.hand_mass, weights=np.arange(1, self.averaging_len + 1), axis=0)

            return ';'.join(["{:5.3f}".format(i) for i in self.Hand]) + f";{str(self.Compress)};"

    def NewAveragingLen(self, len):
        if len > self.averaging_len:
            self.hand_mass = np.insert(self.hand_mass, 0, [self.hand_mass[0]] * (len - self.averaging_len), axis=0)
        self.averaging_len = len


    def get_Hand(self):
        return self.Hand, self.Compress
