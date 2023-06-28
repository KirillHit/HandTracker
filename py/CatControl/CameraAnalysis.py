import mediapipe as mp
import numpy as np
import time
import cv2

from PyQt5.QtCore import QThread, pyqtSignal

class VideoThread(QThread):
    change_pixmap_signal = pyqtSignal(np.ndarray)
    Cam_error_signal = pyqtSignal(int)
    Hand_find = pyqtSignal(tuple, int, bool, bool)

    def __init__(self):
        super().__init__()
        self._run_flag = True
        self.cap = cv2.VideoCapture()
        self.num_Cam = 0

        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_hands = mp.solutions.hands

        self.prev_time = 0

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

                # image = cv2.flip(image, 1)

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
                            hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_MCP],
                            hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_PIP],
                            hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_PIP],
                            hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_PIP],
                            hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_PIP],
                        ]
                        # Заполнение матрицы точек руки
                        all_x = [int(v.x * self.width) for v in Marks]
                        all_y = [int(v.y * self.height) for v in Marks]
                        all_z = [int(v.z * self.width) for v in Marks]
                        # Z использует примерно тот же масштаб, что и x

                        all_t = np.array([all_x, all_y, all_z], dtype=np.int32)
                        # Определение центра руки
                        Center_x = sum(all_t[0][:5]) // 5
                        Center_y = sum(all_t[1][:5]) // 5
                        cv2.circle(image, (Center_x, Center_y), 4, (0, 255, 0), -1)
                        # Нормаль к ладони
                        Norm = sum(
                            [np.cross(all_t[:, 1] - all_t[:, 0], all_t[:, i] - all_t[:, 0]) for i in range(2, 5)]) // 3
                        Norm = Norm * 50 // int(np.linalg.norm(Norm))
                        cv2.line(image, (Center_x, Center_y),
                                 (Center_x - Norm[0], Center_y - Norm[1]),
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

                        # Проверка на сжатие руки
                        angles = []

                        wirst = all_t[:, 0]
                        for fingerNumber in range(4):
                            baseOfFinger = all_t[:, fingerNumber + 1]
                            fingerPip = all_t[:, fingerNumber + 5]

                            baseVector = wirst - baseOfFinger
                            fingerVector = fingerPip - baseOfFinger
                            ##baseVector = [wirst[0] - baseOfFinger[0], wirst[1] - baseOfFinger[1], wirst[2] - baseOfFinger[2]]
                            ##fingerVector = [fingerPip[0] - baseOfFinger[0], fingerPip[1] - baseOfFinger[1], fingerPip[2] - baseOfFinger[2]]

                            baseVectorModeule = np.sqrt(np.sum(np.power(baseVector, 2)))
                            fingerVectorModule = np.sqrt(np.sum(np.power(fingerVector, 2)))

                            vsum = np.sum(baseVector * fingerVector)
                            ##vsum = baseVector[0]*fingerVector[0] + baseVector[1]*fingerVector[1] + baseVector[2]*fingerVector[2]
                            angle = np.arccos(vsum / (baseVectorModeule * fingerVectorModule))

                            # Записываем углы для каждого пальца
                            angles.append(angle)

                        # Получение среднеарифметическое всех углов * 2.6
                        avgAngle = np.average(angles) < 2.6

                        cv2.putText(image, f"SizeFactor: {SizeFactor}", (5, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                    (0, 255, 255), 1)
                        cv2.putText(image, f"x: {Center_x}, y: {Center_y}", (5, 60),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
                        cv2.putText(image, f"Norm: {Norm}", (5, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)

                        if self._run_flag:
                            self.Hand_find.emit((Center_x - self.width // 2,
                                                 Center_y - self.height // 2),
                                                SizeFactor, True, avgAngle)
                else:
                    self.Hand_find.emit((0, 0), 0, False, 0)

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