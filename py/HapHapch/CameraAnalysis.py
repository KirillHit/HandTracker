import mediapipe as mp
import numpy as np
import time
import cv2

from PyQt5.QtCore import QThread, pyqtSignal


class VideoThread(QThread):
    change_pixmap_signal = pyqtSignal(np.ndarray)
    Cam_error_signal = pyqtSignal(int)
    Hand_find = pyqtSignal(np.single, np.single, np.single, bool, bool)

    def __init__(self):
        super().__init__()
        self._run_flag = False
        self.game_flag = False
        self.calibration_flag = True
        self.cap = cv2.VideoCapture()
        self.num_cam = 0

        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_hands = mp.solutions.hands

        self.prev_time = 0
        self.height = 1
        self.width = 1

    def set_cam(self, num_cam):
        self.stop()
        self.num_cam = num_cam
        self.cap = cv2.VideoCapture(self.num_cam)
        success, image = self.cap.read()
        if success:
            self.height, self.width = image.shape[:2]
            self._run_flag = True
            self.start()
        else:
            self.Cam_error_signal.emit(self.num_cam)

    def run(self):
        fps = 0
        with self.mp_hands.Hands(model_complexity=1, max_num_hands=1, min_detection_confidence=0.05,
                                 min_tracking_confidence=0.05, static_image_mode=False) as hands:
            while self._run_flag:
                success, image = self.cap.read()
                if not success:
                    self.Cam_error_signal.emit(self.num_cam)
                    self.stop()
                    break

                image = cv2.flip(image, 1)
                if self.game_flag:
                    results = hands.process(image)
                    if results.multi_hand_landmarks:
                        for hand_landmarks in results.multi_hand_landmarks:
                            self.mp_drawing.draw_landmarks(image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

                            # Массив точек руки
                            marks = [
                                hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST],
                                hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_MCP],
                                hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP],
                                hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_MCP],
                                hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_MCP],
                                hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP],
                                hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP],
                                hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_TIP],
                                hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_TIP],
                            ]

                            # Заполнение матрицы точек руки
                            all_point = np.array([[point.x * self.width, point.y * self.height, point.z * self.width * 1.5]
                                                  for point in marks], dtype=np.single)
                            """Z использует примерно тот же масштаб, что и x (* 1.5 точнее). Если при наклоне руки 
                            камера фиксирует перемещение вверх-вниз, изменение этого коэффициента может помочь."""

                            # Определение центра руки
                            center = np.mean(all_point[:5], dtype=np.single, axis=0)
                            cv2.circle(image, (int(center[0]), int(center[1])), 4, (0, 255, 0), -1)

                            # Размер руки в кадре
                            # Для точек 0-1, 0-2, 0-3, 0-4
                            size_factor = np.mean([np.sqrt(sum(np.power(point - all_point[0], 2)))
                                                   for point in all_point[1:5]], dtype=np.single)

                            # Определение сжатия руки
                            avg_angle = size_factor > np.mean([np.sqrt(sum(np.power(point - all_point[0], 2)))
                                                              for point in all_point[5:9]], dtype=np.single)

                            # cv2.putText(image, f"size_factor: {size_factor}", (5, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
                            # cv2.putText(image, f"x: {center[0]}, y: {center[1]}", (5, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)

                            if self._run_flag:
                                self.Hand_find.emit(center[0] - np.single(self.width / 2),
                                                    center[1] - np.single(self.height / 2),
                                                    size_factor, True, avg_angle)
                    else:
                        self.Hand_find.emit(np.single(0), np.single(0), np.single(0), False, 0)

                    if self.calibration_flag:
                        cv2.circle(image, (self.width // 2, self.height // 2), 80, (0, 200, 0), 6)

                # Счётчик fps
                cur_time = time.time_ns()
                fps = ((10 ** 9 // (cur_time - self.prev_time)) + fps) // 2
                self.prev_time = cur_time
                cv2.putText(image, f"FPS: {str(fps)}", (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)

                if self._run_flag:
                    self.change_pixmap_signal.emit(image)

    def stop(self):
        """Sets run flag to False and waits for thread to finish"""
        self._run_flag = False
        self.wait()
        self.cap.release()
