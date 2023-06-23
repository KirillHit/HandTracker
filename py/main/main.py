from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QPixmap, QIcon
import cv2
import numpy as np
import sys
import mediapipe as mp
import time
import HandObject
import RobotControl
from PyQt5.QtWidgets import QMessageBox

#pyuic5 -x PyQtWindow.ui -o PyQtWindow.py

class VideoThread(QThread):
    change_pixmap_signal = pyqtSignal(np.ndarray)
    Cam_error_signal = pyqtSignal(int)
    Hand_find = pyqtSignal(tuple, int, int, bool)

    def __init__(self):
        super().__init__()
        self._run_flag = True
        self.cap = cv2.VideoCapture()
        self.num_Cam = 0

        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_hands = mp.solutions.hands

        self.prev_time = 0
        # Устранение связи зависимости точности от разрешения камеры
        self.PrecisionParam = 4

    def set_cam(self, num_Cam):
        self.stop()
        self.num_Cam = num_Cam
        self.cap = cv2.VideoCapture(self.num_Cam)
        success, image = self.cap.read()
        if success:
            self.height, self.width = image.shape[:2]
            self._run_flag = True
            self.start()
        else:
            self.Cam_error_signal.emit(self.num_Cam)

    def run(self):
        with self.mp_hands.Hands( model_complexity=1, max_num_hands=1,
                                  min_detection_confidence=0.5, min_tracking_confidence=0.5) as hands:
            while self._run_flag:
                success, image = self.cap.read()
                if not success:
                    self.Cam_error_signal.emit(self.num_Cam)
                    self.stop()
                    break

                image = cv2.flip(image, 1)

                results = hands.process(image)
                if results.multi_hand_landmarks:
                    for hand_landmarks in results.multi_hand_landmarks:
                        self.mp_drawing.draw_landmarks(image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                        # Массив точек руки
                        Marks = [
                            hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST],
                            hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_MCP],
                            hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP],
                            hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_MCP],
                            hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_MCP]
                        ]
                        # Заполнение матрицы точек руки
                        all_x = [int(v.x * self.width * self.PrecisionParam) for v in Marks]
                        all_y = [int(v.y * self.height * self.PrecisionParam) for v in Marks]
                        all_z = [int(v.z * self.width * self.PrecisionParam) for v in
                                 Marks]  # Z использует примерно тот же масштаб, что и x
                        all_t = np.array([all_x, all_y, all_z], dtype=np.int32)
                        # Определение центра руки
                        Center_x = sum(all_t[0][:5]) // 5
                        Center_y = sum(all_t[1][:5]) // 5
                        cv2.circle(image, (Center_x // self.PrecisionParam, Center_y // self.PrecisionParam), 4, (0, 255, 0), -1)
                        # Нормаль к ладони
                        Norm = sum(
                            [np.cross(all_t[:, 1] - all_t[:, 0], all_t[:, i] - all_t[:, 0]) for i in range(2, 5)]) // 3
                        Norm = Norm * 50 // int(np.linalg.norm(Norm))
                        cv2.line(image, (Center_x // self.PrecisionParam, Center_y // self.PrecisionParam),
                                 (Center_x // self.PrecisionParam - Norm[0], Center_y // self.PrecisionParam - Norm[1]),
                                 (0, 0, 0), 2)
                        # Размер руки в кадре
                        # Для точек 0-1, 0-2, 0-3, 0-4
                        m = all_t[:, 1:].copy()
                        for i in range(2):
                            m[i] = m[i] - all_t[:, 0][i]
                        SizeFactor = int(sum(np.sqrt(sum(np.power(m[:, i], 2) for i in range(4)))) // 4)
                        # Для точек 1-2, 2-3, 3-4
                        '''
                        m = np.array([[all_t[:, v][i]-all_t[:, v-1][i] for v in reversed(range(2, 5))] for i in range(3)])
                        SizeFactor = sum(np.sqrt(sum(np.power(m[:, i], 2) for i in range(3)))) // 3
                        '''

                        cv2.putText(image, f"SizeFactor: {SizeFactor}", (5, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                    (0, 255, 255), 1)
                        cv2.putText(image, f"x: {Center_x // self.PrecisionParam}, y: {Center_y // self.PrecisionParam}", (5, 60),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
                        cv2.putText(image, f"Norm: {Norm}", (5, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)

                        if self._run_flag:
                            self.Hand_find.emit((Center_x - self.width * self.PrecisionParam // 2,
                                                Center_y - self.height * self.PrecisionParam // 2),
                                                SizeFactor, self.PrecisionParam, True)
                else:
                    self.Hand_find.emit((0,0), 0, 0, False)

                cur_time = time.time_ns()
                fps = 10 ** 9 // (cur_time - self.prev_time)
                self.prev_time = cur_time
                cv2.putText(image, f"FPS: {str(fps)}", (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)
                cv2.circle(image, (self.width // 2, self.height // 2), 20, (0, 255, 0), 1)

                if self._run_flag:
                    self.change_pixmap_signal.emit(image)
                    
    def stop(self):
        """Sets run flag to False and waits for thread to finish"""
        self._run_flag = False
        self.wait()
        self.cap.release()

class Ui_MainWindow(object):
    def __init__(self):
        self.CameraThread = VideoThread()
        self.RobotThread = RobotControl.RobotObject()
        self.HandTracker = HandObject.HandTracker()

    def setupUi(self, MainWindow):
        # Код окна из PQ дизайнера
        # region
        MainWindow.setObjectName("MainWindow")
        MainWindow.setEnabled(True)
        MainWindow.resize(1200, 600)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayout_6 = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout_6.setObjectName("gridLayout_6")
        self.verticalFrame = QtWidgets.QFrame(self.centralwidget)
        self.verticalFrame.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.verticalFrame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.verticalFrame.setLineWidth(2)
        self.verticalFrame.setObjectName("verticalFrame")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.verticalFrame)
        self.verticalLayout_4.setContentsMargins(-1, 1, -1, 5)
        self.verticalLayout_4.setSpacing(2)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.frame = QtWidgets.QFrame(self.verticalFrame)
        self.frame.setFrameShape(QtWidgets.QFrame.Panel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame.setLineWidth(2)
        self.frame.setObjectName("frame")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.frame)
        self.verticalLayout.setContentsMargins(-1, 5, -1, 5)
        self.verticalLayout.setSpacing(5)
        self.verticalLayout.setObjectName("verticalLayout")
        self.label_10 = QtWidgets.QLabel(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_10.sizePolicy().hasHeightForWidth())
        self.label_10.setSizePolicy(sizePolicy)
        self.label_10.setStyleSheet("font: 75 12pt \"Calibri\";")
        self.label_10.setFrameShape(QtWidgets.QFrame.WinPanel)
        self.label_10.setFrameShadow(QtWidgets.QFrame.Raised)
        self.label_10.setLineWidth(1)
        self.label_10.setAlignment(QtCore.Qt.AlignCenter)
        self.label_10.setObjectName("label_10")
        self.verticalLayout.addWidget(self.label_10)
        self.frame1 = QtWidgets.QFrame(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.frame1.sizePolicy().hasHeightForWidth())
        self.frame1.setSizePolicy(sizePolicy)
        self.frame1.setObjectName("frame1")
        self.gridLayout_3 = QtWidgets.QGridLayout(self.frame1)
        self.gridLayout_3.setContentsMargins(-1, 5, -1, 5)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.Start_cam = QtWidgets.QPushButton(self.frame1)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Start_cam.sizePolicy().hasHeightForWidth())
        self.Start_cam.setSizePolicy(sizePolicy)
        self.Start_cam.setObjectName("Start_cam")
        self.gridLayout_3.addWidget(self.Start_cam, 0, 2, 1, 1)
        self.label = QtWidgets.QLabel(self.frame1)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label.sizePolicy().hasHeightForWidth())
        self.label.setSizePolicy(sizePolicy)
        self.label.setAlignment(QtCore.Qt.AlignLeading | QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
        self.label.setObjectName("label")
        self.gridLayout_3.addWidget(self.label, 0, 0, 1, 1)
        self.NumCamEditLine = QtWidgets.QLineEdit(self.frame1)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(100)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.NumCamEditLine.sizePolicy().hasHeightForWidth())
        self.NumCamEditLine.setSizePolicy(sizePolicy)
        self.NumCamEditLine.setMinimumSize(QtCore.QSize(0, 0))
        self.NumCamEditLine.setMaximumSize(QtCore.QSize(50, 16777215))
        self.NumCamEditLine.setBaseSize(QtCore.QSize(0, 0))
        self.NumCamEditLine.setObjectName("NumCamEditLine")
        self.gridLayout_3.addWidget(self.NumCamEditLine, 0, 1, 1, 1)
        self.label_2 = QtWidgets.QLabel(self.frame1)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.MinimumExpanding, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_2.sizePolicy().hasHeightForWidth())
        self.label_2.setSizePolicy(sizePolicy)
        self.label_2.setText("")
        self.label_2.setObjectName("label_2")
        self.gridLayout_3.addWidget(self.label_2, 0, 3, 1, 1)
        self.verticalLayout.addWidget(self.frame1)
        self.horizontalFrame_2 = QtWidgets.QFrame(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.horizontalFrame_2.sizePolicy().hasHeightForWidth())
        self.horizontalFrame_2.setSizePolicy(sizePolicy)
        self.horizontalFrame_2.setObjectName("horizontalFrame_2")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.horizontalFrame_2)
        self.horizontalLayout_2.setContentsMargins(-1, 5, -1, 5)
        self.horizontalLayout_2.setSpacing(5)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.lab_2 = QtWidgets.QLabel(self.horizontalFrame_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.lab_2.sizePolicy().hasHeightForWidth())
        self.lab_2.setSizePolicy(sizePolicy)
        self.lab_2.setObjectName("lab_2")
        self.horizontalLayout_2.addWidget(self.lab_2)
        self.CalibDist = QtWidgets.QLineEdit(self.horizontalFrame_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.CalibDist.sizePolicy().hasHeightForWidth())
        self.CalibDist.setSizePolicy(sizePolicy)
        self.CalibDist.setMaximumSize(QtCore.QSize(50, 16777215))
        self.CalibDist.setObjectName("CalibDist")
        self.horizontalLayout_2.addWidget(self.CalibDist)
        self.label_3 = QtWidgets.QLabel(self.horizontalFrame_2)
        self.label_3.setObjectName("label_3")
        self.horizontalLayout_2.addWidget(self.label_3)
        self.label_4 = QtWidgets.QLabel(self.horizontalFrame_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.MinimumExpanding, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_4.sizePolicy().hasHeightForWidth())
        self.label_4.setSizePolicy(sizePolicy)
        self.label_4.setText("")
        self.label_4.setObjectName("label_4")
        self.horizontalLayout_2.addWidget(self.label_4)
        self.horizontalLayout_2.setStretch(0, 1)
        self.horizontalLayout_2.setStretch(1, 1)
        self.horizontalLayout_2.setStretch(2, 1)
        self.horizontalLayout_2.setStretch(3, 10)
        self.verticalLayout.addWidget(self.horizontalFrame_2)
        self.gridLayout_4 = QtWidgets.QGridLayout()
        self.gridLayout_4.setContentsMargins(-1, 7, -1, 7)
        self.gridLayout_4.setHorizontalSpacing(7)
        self.gridLayout_4.setVerticalSpacing(10)
        self.gridLayout_4.setObjectName("gridLayout_4")
        self.TimeApprox = QtWidgets.QSlider(self.frame)
        self.TimeApprox.setMaximum(2000)
        self.TimeApprox.setPageStep(1)
        self.TimeApprox.setProperty("value", 500)
        self.TimeApprox.setOrientation(QtCore.Qt.Horizontal)
        self.TimeApprox.setObjectName("TimeApprox")
        self.gridLayout_4.addWidget(self.TimeApprox, 2, 1, 1, 1)
        self.CalibCam = QtWidgets.QSlider(self.frame)
        self.CalibCam.setMaximum(200)
        self.CalibCam.setPageStep(1)
        self.CalibCam.setProperty("value", 80)
        self.CalibCam.setOrientation(QtCore.Qt.Horizontal)
        self.CalibCam.setObjectName("CalibCam")
        self.gridLayout_4.addWidget(self.CalibCam, 0, 1, 1, 1)
        self.Lab_TimeApprox = QtWidgets.QLabel(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Lab_TimeApprox.sizePolicy().hasHeightForWidth())
        self.Lab_TimeApprox.setSizePolicy(sizePolicy)
        self.Lab_TimeApprox.setObjectName("Lab_TimeApprox")
        self.gridLayout_4.addWidget(self.Lab_TimeApprox, 2, 2, 1, 1)
        self.FixedParam = QtWidgets.QSlider(self.frame)
        self.FixedParam.setMaximum(20)
        self.FixedParam.setPageStep(1)
        self.FixedParam.setProperty("value", 3)
        self.FixedParam.setOrientation(QtCore.Qt.Horizontal)
        self.FixedParam.setObjectName("FixedParam")
        self.gridLayout_4.addWidget(self.FixedParam, 3, 1, 1, 1)
        self.label_8 = QtWidgets.QLabel(self.frame)
        self.label_8.setObjectName("label_8")
        self.gridLayout_4.addWidget(self.label_8, 4, 0, 1, 1)
        self.Lab_CamAngle = QtWidgets.QLabel(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Lab_CamAngle.sizePolicy().hasHeightForWidth())
        self.Lab_CamAngle.setSizePolicy(sizePolicy)
        self.Lab_CamAngle.setObjectName("Lab_CamAngle")
        self.gridLayout_4.addWidget(self.Lab_CamAngle, 1, 2, 1, 1)
        self.Lab_FixedParam = QtWidgets.QLabel(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Lab_FixedParam.sizePolicy().hasHeightForWidth())
        self.Lab_FixedParam.setSizePolicy(sizePolicy)
        self.Lab_FixedParam.setObjectName("Lab_FixedParam")
        self.gridLayout_4.addWidget(self.Lab_FixedParam, 3, 2, 1, 1)
        self.label_5 = QtWidgets.QLabel(self.frame)
        self.label_5.setObjectName("label_5")
        self.gridLayout_4.addWidget(self.label_5, 1, 0, 1, 1)
        self.label_6 = QtWidgets.QLabel(self.frame)
        self.label_6.setObjectName("label_6")
        self.gridLayout_4.addWidget(self.label_6, 2, 0, 1, 1)
        self.Lab_FixedParam_Z = QtWidgets.QLabel(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Lab_FixedParam_Z.sizePolicy().hasHeightForWidth())
        self.Lab_FixedParam_Z.setSizePolicy(sizePolicy)
        self.Lab_FixedParam_Z.setObjectName("Lab_FixedParam_Z")
        self.gridLayout_4.addWidget(self.Lab_FixedParam_Z, 4, 2, 1, 1)
        self.label_7 = QtWidgets.QLabel(self.frame)
        self.label_7.setObjectName("label_7")
        self.gridLayout_4.addWidget(self.label_7, 3, 0, 1, 1)
        self.CamAngle = QtWidgets.QSlider(self.frame)
        self.CamAngle.setMaximum(180)
        self.CamAngle.setPageStep(1)
        self.CamAngle.setProperty("value", 90)
        self.CamAngle.setOrientation(QtCore.Qt.Horizontal)
        self.CamAngle.setObjectName("CamAngle")
        self.gridLayout_4.addWidget(self.CamAngle, 1, 1, 1, 1)
        self.Lab_CalibCam = QtWidgets.QLabel(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Lab_CalibCam.sizePolicy().hasHeightForWidth())
        self.Lab_CalibCam.setSizePolicy(sizePolicy)
        self.Lab_CalibCam.setMinimumSize(QtCore.QSize(40, 0))
        self.Lab_CalibCam.setObjectName("Lab_CalibCam")
        self.gridLayout_4.addWidget(self.Lab_CalibCam, 0, 2, 1, 1)
        self.lab = QtWidgets.QLabel(self.frame)
        self.lab.setObjectName("lab")
        self.gridLayout_4.addWidget(self.lab, 0, 0, 1, 1)
        self.FixedParam_Z = QtWidgets.QSlider(self.frame)
        self.FixedParam_Z.setMaximum(20)
        self.FixedParam_Z.setPageStep(1)
        self.FixedParam_Z.setProperty("value", 4)
        self.FixedParam_Z.setOrientation(QtCore.Qt.Horizontal)
        self.FixedParam_Z.setObjectName("FixedParam_Z")
        self.gridLayout_4.addWidget(self.FixedParam_Z, 4, 1, 1, 1)
        self.gridLayout_4.setColumnStretch(0, 1)
        self.gridLayout_4.setColumnStretch(1, 10)
        self.gridLayout_4.setColumnStretch(2, 2)
        self.verticalLayout.addLayout(self.gridLayout_4)
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setContentsMargins(-1, 5, -1, 5)
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.label_11 = QtWidgets.QLabel(self.frame)
        self.label_11.setObjectName("label_11")
        self.horizontalLayout_3.addWidget(self.label_11)
        self.Hand_Coords = QtWidgets.QLabel(self.frame)
        self.Hand_Coords.setStyleSheet("font: 75 14pt \"Calibri\";\n"
                                       "color: rgb(0, 170, 0);")
        self.Hand_Coords.setObjectName("Hand_Coords")
        self.horizontalLayout_3.addWidget(self.Hand_Coords)
        self.label_13 = QtWidgets.QLabel(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_13.sizePolicy().hasHeightForWidth())
        self.label_13.setSizePolicy(sizePolicy)
        self.label_13.setText("")
        self.label_13.setObjectName("label_13")
        self.horizontalLayout_3.addWidget(self.label_13)
        self.verticalLayout.addLayout(self.horizontalLayout_3)
        self.label_9 = QtWidgets.QLabel(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_9.sizePolicy().hasHeightForWidth())
        self.label_9.setSizePolicy(sizePolicy)
        self.label_9.setStyleSheet("font: 75 12pt \"Calibri\";")
        self.label_9.setFrameShape(QtWidgets.QFrame.WinPanel)
        self.label_9.setFrameShadow(QtWidgets.QFrame.Raised)
        self.label_9.setAlignment(QtCore.Qt.AlignCenter)
        self.label_9.setObjectName("label_9")
        self.verticalLayout.addWidget(self.label_9)
        self.horizontalFrame = QtWidgets.QFrame(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.horizontalFrame.sizePolicy().hasHeightForWidth())
        self.horizontalFrame.setSizePolicy(sizePolicy)
        self.horizontalFrame.setObjectName("horizontalFrame")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.horizontalFrame)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.verticalLayout.addWidget(self.horizontalFrame)
        self.horizontalLayout_5.addWidget(self.frame)
        self.Lab_Cam = QtWidgets.QLabel(self.verticalFrame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Ignored)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Lab_Cam.sizePolicy().hasHeightForWidth())
        self.Lab_Cam.setSizePolicy(sizePolicy)
        self.Lab_Cam.setMinimumSize(QtCore.QSize(0, 0))
        palette = QtGui.QPalette()
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Base, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Base, brush)
        brush = QtGui.QBrush(QtGui.QColor(240, 240, 240))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Base, brush)
        self.Lab_Cam.setPalette(palette)
        self.Lab_Cam.setWhatsThis("")
        self.Lab_Cam.setAutoFillBackground(False)
        self.Lab_Cam.setStyleSheet("font: 75 14pt \"Calibri\";")
        self.Lab_Cam.setFrameShape(QtWidgets.QFrame.Panel)
        self.Lab_Cam.setFrameShadow(QtWidgets.QFrame.Raised)
        self.Lab_Cam.setLineWidth(2)
        self.Lab_Cam.setAlignment(QtCore.Qt.AlignCenter)
        self.Lab_Cam.setObjectName("Lab_Cam")
        self.horizontalLayout_5.addWidget(self.Lab_Cam)
        self.horizontalLayout_5.setStretch(0, 10)
        self.horizontalLayout_5.setStretch(1, 18)
        self.verticalLayout_4.addLayout(self.horizontalLayout_5)
        self.verticalLayout_4.setStretch(0, 10)
        self.gridLayout_6.addWidget(self.verticalFrame, 0, 0, 1, 1)
        MainWindow.setCentralWidget(self.centralwidget)
        # endregion

        # Самая важная строчка
        MainWindow.setWindowIcon(QIcon("icon.jpg"))

        # Позволяет изображению деформироваться
        # self.Lab_Cam.setScaledContents(True)
        # Чёрный фон камеры
        self.Lab_Cam.setStyleSheet("background-color: black; color: rgb(255, 255, 255); font: 75 14pt 'Calibri'")

        # Настройка полей и ползунков
        # region
        self.retranslateUi(MainWindow)
        self.CalibCam.valueChanged['int'].connect(self.Lab_CalibCam.setNum)
        self.CamAngle.valueChanged['int'].connect(self.Lab_CamAngle.setNum)
        self.FixedParam.valueChanged['int'].connect(self.Lab_FixedParam.setNum)
        self.TimeApprox.valueChanged['int'].connect(self.Lab_TimeApprox.setNum)
        self.FixedParam_Z.valueChanged['int'].connect(self.Lab_FixedParam_Z.setNum)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

        self.Lab_CalibCam.setText(str(self.CalibCam.value()))
        self.Lab_CamAngle.setText(str(self.CamAngle.value()))
        self.Lab_FixedParam.setText(str(self.FixedParam.value()))
        self.Lab_TimeApprox.setText(str(self.TimeApprox.value()))
        self.Lab_FixedParam_Z.setText(str(self.FixedParam_Z.value()))

        self.CalibDist.editingFinished.connect(self.change_cam)
        self.CalibCam.valueChanged.connect(self.change_cam)
        self.CamAngle.valueChanged.connect(self.change_cam)
        self.FixedParam.valueChanged.connect(self.change_cam)
        self.TimeApprox.valueChanged.connect(self.change_cam)
        self.FixedParam_Z.valueChanged.connect(self.change_cam)

        self.Start_cam.clicked.connect(self.new_Cam)
        # endregion

        # connect its signal to the update_image slot
        self.CameraThread.change_pixmap_signal.connect(self.update_image)
        self.CameraThread.Cam_error_signal.connect(self.Cam_error)
        self.CameraThread.Hand_find.connect(self.Hand_update)
        self.RobotThread.GetHand.connect(self.HandToRobot)

        # Попытка подключится к камере по умолчанию
        self.new_Cam()
        # Обновление данных класса HandTracker
        self.change_cam()

        self.RobotThread.start()

    def HandToRobot(self):
        self.RobotThread.SetHand(self.HandTracker.Hand)

    def new_Cam(self):
        if self.NumCamEditLine.text().isdigit():
            self.CameraThread.set_cam(int(self.NumCamEditLine.text()))
            self.HandTracker.width = self.CameraThread.width
            self.HandTracker.height = self.CameraThread.height
        else:
            self.showDialog("Введите число большее и равное нулю")

    def change_cam(self):
        self.HandTracker.CalibDist = int(self.CalibDist.text())
        self.HandTracker.CalibCam = int(self.CalibCam.value())
        self.HandTracker.CamAngle = (int(self.CamAngle.value()) * 3.14) // 180
        self.HandTracker.FixedParam = int(self.FixedParam.value())
        self.HandTracker.TimeApprox = int(self.TimeApprox.value())
        self.HandTracker.LenApprox = 60 * int(self.TimeApprox.value()) // 1000
        self.HandTracker.FixedParam_Z = int(self.FixedParam_Z.value())

    def closeEvent(self, event):
        self.CameraThread.stop()
        event.accept()

    def update_image(self, cv_img):
        """Updates the image_label with a new opencv image"""
        self.Lab_Cam.setPixmap(self.convert_cv_qt(cv_img))

    def Hand_update(self, Cordinate, SizeFactor, PrecisionParam, HandExist):
        if HandExist:
            self.Hand_Coords.setText(self.HandTracker.give_Hand(Cordinate, SizeFactor, PrecisionParam))
        else:
            if self.Hand_Coords.text() != "No hand":
                self.Hand_Coords.setText("No hand")

    def Cam_error(self, Current_cam):
        self.Lab_Cam.setText(f"Камера {Current_cam} не найдена")
        self.Hand_Coords.setText("No hand")
        self.showDialog("Камера не найдена.\n"
                        "Всем подключённым камерам присваиваются номера от нуля и далее по возрастанию. \n"
                        "0 - по умолчанию веб-камера.")

    def convert_cv_qt(self, cv_img):
        """Convert from an opencv image to QPixmap"""
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        p = convert_to_Qt_format.scaled(self.Lab_Cam.size().width(), self.Lab_Cam.size().height(), Qt.KeepAspectRatio)
        return QPixmap.fromImage(p)

    def showDialog(self, text):
        msgBox = QMessageBox()
        msgBox.setWindowTitle("HandTracker")
        msgBox.setIcon(QMessageBox.Information)
        msgBox.setText(text)
        msgBox.setStandardButtons(QMessageBox.Ok)
        msgBox.exec()

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "HandTracker"))
        self.label_10.setText(_translate("MainWindow", "Интерфейс управления камерой"))
        self.Start_cam.setText(_translate("MainWindow", "Обновить"))
        self.label.setText(_translate("MainWindow", "Номер камеры:"))
        self.NumCamEditLine.setText(_translate("MainWindow", "0"))
        self.lab_2.setText(_translate("MainWindow", "Дистанция калибровки"))
        self.CalibDist.setText(_translate("MainWindow", "500"))
        self.label_3.setText(_translate("MainWindow", "мм"))
        self.Lab_TimeApprox.setText(_translate("MainWindow", "0"))
        self.label_8.setText(_translate("MainWindow", "Порог движения Z"))
        self.Lab_CamAngle.setText(_translate("MainWindow", "0"))
        self.Lab_FixedParam.setText(_translate("MainWindow", "0"))
        self.label_5.setText(_translate("MainWindow", "Наклон камеры"))
        self.label_6.setText(_translate("MainWindow", "Время усреднения (мс)"))
        self.Lab_FixedParam_Z.setText(_translate("MainWindow", "0"))
        self.label_7.setText(_translate("MainWindow", "Порог движения"))
        self.Lab_CalibCam.setText(_translate("MainWindow", "0"))
        self.lab.setText(_translate("MainWindow", "Эквивалент камеры"))
        self.label_11.setText(_translate("MainWindow", "Координаты руки:"))
        self.Hand_Coords.setText(_translate("MainWindow", "No hands"))
        self.label_9.setText(_translate("MainWindow", "Интерфейс робота"))
        self.Lab_Cam.setText(_translate("MainWindow", "<html><head/><body><p>Камеры нет</p></body></html>"))


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
