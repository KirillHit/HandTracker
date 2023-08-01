# -*- coding: utf-8 -*-

import numpy as np
import time


class HandTracker:
    def __init__(self):
        self.SaveSize = None
        self.TrackingProcess = False

        self.height = 1
        self.width = 1
        self.Radius = 80
        self.CalibDist = 500
        self.CalibCam = 1

        self.StartTime = 0
        self.PrevTime = 0
        self.CalibTimer = 1

        self.FixedParam = 3
        self.FixedParam_Z = 14
        # -----
        self.Fixed_x = 0
        self.Fixed_y = 0
        self.Fixed_z = 0
        # -----
        self.Hand = [0, 0, 0]
        self.Compress = False

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
                        self.SaveSize = np.mean(self.SaveSize, dtype=np.single)

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
            self.approx_z = np.append(self.approx_z, self.Real_z)
            self.approx_t = np.append(self.approx_t, now)
            self.approx_x = np.append(self.approx_x, self.Real_x)
            self.approx_y = np.append(self.approx_y, self.Real_y)

            self.approx_z = self.approx_z[-self.LenApprox:]
            self.approx_t = self.approx_t[-self.LenApprox:]
            self.approx_x = self.approx_x[-self.LenApprox:]
            self.approx_y = self.approx_y[-self.LenApprox:]

            for i, now in enumerate(reversed(self.approx_t[1:])):
                if (self.approx_t[-1] - now) >= self.TimeApprox * (10 ** 6):
                    break
            
            self.Real_z = np.mean(self.approx_z[-i:], dtype=np.single)
            self.Real_x = np.mean(self.approx_x[-i:], dtype=np.single)
            self.Real_y = np.mean(self.approx_y[-i:], dtype=np.single)
            '''

            '''
            if (abs(self.Fixed_z - self.Real_z)) > self.FixedParam_Z:
                self.Fixed_z = self.Real_z
            if (abs(self.Fixed_y - self.Real_y)) > self.FixedParam:
                self.Fixed_y = self.Real_y
            if (abs(self.Fixed_x - self.Real_x)) > self.FixedParam:
                self.Fixed_x = self.Real_x
            '''

            self.Hand = [self.Real_x, self.Real_y, self.Real_z]

            self.Compress = compress

            return ';'.join(["{:5.3f}".format(i) for i in self.Hand]) + f";{str(self.Compress)};"

    def get_Hand(self):
        return (self.Hand, self.Compress)
