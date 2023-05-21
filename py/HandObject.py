import numpy as np
import math

class HandTracker:
    def __init__(self, height, width):
        self.SaveSize = None
        self.TrackingProcess = False

        self.height = height
        self.width = width
        self.Radius = 20
        self.CalibDist = 500
        self.CalibCam = 0.83
        self.CamAngle = 90 * 3.14 / 180

        self.StartTime = 0
        self.PrevTime = 0
        self.CalibTimer = 1
        self.LostHandTimer = 2

        self.TimeApprox = 300 # В миллисекундах
        self.LenApprox = 60 * self.TimeApprox // 1000
        self.approx_x = np.zeros(self.LenApprox, dtype=np.int16)
        self.approx_y = np.zeros(self.LenApprox, dtype=np.int16)
        self.approx_z = np.zeros(self.LenApprox, dtype=np.int16)
        self.approx_t = np.zeros(self.LenApprox, dtype=np.int16)

        self.FixedParam = 3
        self.Fixed_z_Modif = 4
        self.Fixed_x = 0
        self.Fixed_y = 0
        self.Fixed_z = 0

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

    def give_Hand (self, Center, time, SizeFactor, PrecisionParam):
        # Проверка пропажи руки
        if self.TrackingProcess and (time - self.PrevTime) >= self.LostHandTimer*10**9:
            self.TrackingProcess = False
            print(f"Del: {self.SaveSize}")
        self.PrevTime = time
        # Калибровка размеров руки
        if not self.TrackingProcess:
            if (Center[0]**2 + Center[1]**2) <= (self.Radius*PrecisionParam)**2:
                if self.StartTime == 0:
                    self.StartTime = time
                    self.SaveSize = np.array([SizeFactor])
                else:
                    self.SaveSize = np.append(self.SaveSize, SizeFactor)
                    if (time - self.StartTime) >= self.CalibTimer*10**9:
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
            self.Real_x = self.CalibCam * self.Real_z * Center[0] // (self.CalibDist * PrecisionParam)
            self.Real_y = self.CalibCam * self.Real_z * Center[1] // (self.CalibDist * PrecisionParam)

            self.approx_z = np.append(self.approx_z, self.Real_z)
            self.approx_t = np.append(self.approx_t, time)
            self.approx_x = np.append(self.approx_x, self.Real_x)
            self.approx_y = np.append(self.approx_y, self.Real_y)

            self.approx_z = self.approx_z[-self.LenApprox:]
            self.approx_t = self.approx_t[-self.LenApprox:]
            self.approx_x = self.approx_x[-self.LenApprox:]
            self.approx_y = self.approx_y[-self.LenApprox:]

            for i, time in enumerate(reversed(self.approx_t)):
                if (self.approx_t[-1] - time) >= self.TimeApprox * (10 ** 6):
                    break

            self.Real_z = sum(self.approx_z[-i-1:])//(i+1)
            self.Real_x = sum(self.approx_x[-i-1:])//(i+1)
            self.Real_y = sum(self.approx_y[-i-1:])//(i+1)

            if (abs(self.Fixed_z - self.Real_z)) > self.FixedParam * self.Fixed_z_Modif:
                self.Fixed_z = self.Real_z
            if (abs(self.Fixed_y - self.Real_y)) > self.FixedParam:
                self.Fixed_y = self.Real_y
            if (abs(self.Fixed_x - self.Real_x)) > self.FixedParam:
                self.Fixed_x = self.Real_x

            y = -int(self.Fixed_y * math.cos(self.CamAngle) - self.Fixed_z * math.sin(self.CamAngle))
            z = -int(self.Fixed_y * math.sin(self.CamAngle) + self.Fixed_z * math.cos(self.CamAngle))

            return f"X:{self.Fixed_x}, Y:{y}, Z:{z}"


