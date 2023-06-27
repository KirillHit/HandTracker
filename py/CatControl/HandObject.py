# -*- coding: utf-8 -*-

import numpy as np
import time

class HandTracker:
    def __init__(self):
        self.SaveSize = None
        self.TrackingProcess = False

        self.height = 1
        self.width = 1
        self.Radius = 20
        self.CalibDist = 500
        self.CalibCam = 83
        self.CamAngle = 90 * 3.14 / 180

        self.StartTime = 0
        self.PrevTime = 0
        self.CalibTimer = 1
        self.LostHandTimer = 2

        self.TimeApprox = 500 # В миллисекундах
        self.LenApprox = 60 * self.TimeApprox // 1000
        self.approx_x = np.zeros(self.LenApprox, dtype=np.int16)
        self.approx_y = np.zeros(self.LenApprox, dtype=np.int16)
        self.approx_z = np.zeros(self.LenApprox, dtype=np.int16)
        self.approx_t = np.zeros(self.LenApprox, dtype=np.int16)

        self.FixedParam = 3
        self.FixedParam_Z = 14
        self.Fixed_x = 0
        self.Fixed_y = 0
        self.Fixed_z = 0

        self.Hand = [0, 0, self.CalibDist]
        self.Compress = False

    # При малых перемещениях вызывает ошибку слишком малого значения
    '''
    from scipy.optimize import curve_fit
    def mapping(self, t, a, b, c):
        return a * t + b
        
    def Approx(self, y, time):
        popt, _ = curve_fit(self.mapping, self.approx_t, y)
        a, b, c = popt
        return int(self.mapping(time, a, b, c))
    '''

    def give_Hand (self, Center, SizeFactor, PrecisionParam, Compress):
        Now = time.time_ns()
        self.Compress = Compress
        # Проверка пропажи руки
        if self.TrackingProcess and (Now - self.PrevTime) >= self.LostHandTimer*10**9:
            self.TrackingProcess = False
            self.Compress = False
        self.PrevTime = Now
        # Калибровка размеров руки
        if not self.TrackingProcess:
            if (Center[0]**2 + Center[1]**2) <= (self.Radius*PrecisionParam)**2:
                if self.StartTime == 0:
                    self.StartTime = Now
                    self.SaveSize = np.array([SizeFactor])
                else:
                    self.SaveSize = np.append(self.SaveSize, SizeFactor)
                    if (Now - self.StartTime) >= self.CalibTimer*10**9:
                        self.TrackingProcess = True
                        self.SaveSize = sum(self.SaveSize)//len(self.SaveSize)
                        self.approx_x = np.zeros(self.LenApprox, dtype=np.int16)
                        self.approx_y = np.zeros(self.LenApprox, dtype=np.int16)
                        self.approx_z = np.zeros(self.LenApprox, dtype=np.int16)

            else:
                self.StartTime = 0
            return "Calibration"
        # Отслеживание руки
        else:
            self.Real_z = int(self.CalibDist * self.SaveSize // SizeFactor)
            self.Real_x = self.CalibCam * self.Real_z * Center[0] // (self.CalibDist * PrecisionParam * 100)
            self.Real_y = self.CalibCam * self.Real_z * Center[1] // (self.CalibDist * PrecisionParam * 100)

            self.approx_z = np.append(self.approx_z, self.Real_z)
            self.approx_t = np.append(self.approx_t, Now)
            self.approx_x = np.append(self.approx_x, self.Real_x)
            self.approx_y = np.append(self.approx_y, self.Real_y)

            self.approx_z = self.approx_z[-self.LenApprox:]
            self.approx_t = self.approx_t[-self.LenApprox:]
            self.approx_x = self.approx_x[-self.LenApprox:]
            self.approx_y = self.approx_y[-self.LenApprox:]

            for i, Now in enumerate(reversed(self.approx_t[1:])):
                if (self.approx_t[-1] - Now) >= self.TimeApprox * (10 ** 6):
                    break

            self.Real_z = sum(self.approx_z[-i-1:])//(i+1)
            self.Real_x = sum(self.approx_x[-i-1:])//(i+1)
            self.Real_y = - sum(self.approx_y[-i-1:])//(i+1)

            if (abs(self.Fixed_z - self.Real_z)) > self.FixedParam_Z:
                self.Fixed_z = self.Real_z
            if (abs(self.Fixed_y - self.Real_y)) > self.FixedParam:
                self.Fixed_y = self.Real_y
            if (abs(self.Fixed_x - self.Real_x)) > self.FixedParam:
                self.Fixed_x = self.Real_x

            '''
            y = -int(self.Fixed_y * math.cos(self.CamAngle) - self.Fixed_z * math.sin(self.CamAngle))
            z = -int(self.Fixed_y * math.sin(self.CamAngle) + self.Fixed_z * math.cos(self.CamAngle))
            '''

            self.Hand = [self.Fixed_x, self.Fixed_y, self.Fixed_z]
            return f"X:{self.Fixed_x}, Y:{self.Fixed_y}, Z:{self.Fixed_z}"

    def get_Hand (self):
        Now = time.time_ns()
        # Проверка пропажи руки
        if self.TrackingProcess and (Now - self.PrevTime) >= self.LostHandTimer * 10 ** 9:
            self.TrackingProcess = False
            self.Compress = False

        return {"Hand": self.Hand, "width": self.width, "height": self.height,
                "CalibDist": self.CalibDist, "Compress": self.Compress}




