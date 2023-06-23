import cv2
import mediapipe as mp
import time
import numpy as np
import HandObject as Ho

mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands

cap = cv2.VideoCapture(0)
cv2.namedWindow("HandTracker")
success, image = cap.read()
if not success:
    print("Ignoring empty camera frame.")
    exit()
height, width = image.shape[:2]

HandTracker = Ho.HandTracker(height, width)

prev_time = 0

# ДОДЕЛАТЬ
PrecisionParam = 2 # Устрание связи зависимости точности от разрешения камеры

with mp_hands.Hands(
        model_complexity=1,
        max_num_hands=1,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5) as hands:
    while cap.isOpened():
        success, image = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
            break

        image = cv2.flip(image, 1)

        results = hands.process(image)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                # Массив точек руки
                Marks = [
                    hand_landmarks.landmark[mp_hands.HandLandmark.WRIST],
                    hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP],
                    hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP],
                    hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_MCP],
                    hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_MCP]
                ]
                # Заполнение матрицы точек руки
                all_x = [int(v.x * width * PrecisionParam) for v in Marks]
                all_y = [int(v.y * height * PrecisionParam) for v in Marks]
                all_z = [int(v.z * width * PrecisionParam) for v in Marks] # Z использует примерно тот же масштаб, что и x
                all_t = np.array([all_x, all_y, all_z], dtype=np.int32)
                # Определение центра руки
                Center_x = sum(all_t[0][:5]) // 5
                Center_y = sum(all_t[1][:5]) // 5
                cv2.circle(image, (Center_x // PrecisionParam, Center_y // PrecisionParam), 4, (0, 255, 0), -1)
                # Нормаль к ладони
                Norm = sum([np.cross(all_t[:, 1] - all_t[:, 0], all_t[:, i] - all_t[:, 0]) for i in range(2, 5)]) // 3
                Norm = Norm * 50 // int(np.linalg.norm(Norm))
                cv2.line(image, (Center_x // PrecisionParam, Center_y // PrecisionParam),
                         (Center_x // PrecisionParam - Norm[0], Center_y // PrecisionParam - Norm[1]), (0, 0, 0), 2)
                # Размер руки в кадре
                # Для точек 0-1, 0-2, 0-3, 0-4
                m = all_t[:, 1:].copy()
                for i in range(2):
                    m[i] = m[i]-all_t[:, 0][i]
                SizeFactor = sum(np.sqrt(sum(np.power(m[:, i], 2) for i in range(4)))) // 4
                # Для точек 1-2, 2-3, 3-4
                '''
                m = np.array([[all_t[:, v][i]-all_t[:, v-1][i] for v in reversed(range(2, 5))] for i in range(3)])
                SizeFactor = sum(np.sqrt(sum(np.power(m[:, i], 2) for i in range(3)))) // 3
                '''

                cv2.putText(image, f"SizeFactor: {SizeFactor}", (5, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
                cv2.putText(image, f"x: {Center_x // PrecisionParam}, y: {Center_y // PrecisionParam}", (5, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
                cv2.putText(image, f"Norm: {Norm}", (5, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)

                cv2.putText(image,
                            HandTracker.give_Hand((Center_x - width*PrecisionParam//2, Center_y - height*PrecisionParam//2),
                            time.time_ns(), SizeFactor, PrecisionParam), (5, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)

        cur_time = time.time_ns()
        fps = 10**9//(cur_time - prev_time)
        prev_time = cur_time
        cv2.putText(image, f"FPS: {str(fps)}", (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)
        cv2.circle(image, (width//2, height//2), 20, (0, 255, 0), 1)

        cv2.imshow('HandTracker', image)
        if cv2.waitKey(5) & 0xFF == 27:
            break
cap.release()
