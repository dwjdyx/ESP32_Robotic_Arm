import numpy as np
import time
import cv2
import numpy
import threading

FRAME_SIZE = (640,360)
INVALID_POS = 0xFFFF
FPS_POS = (10,30)
MIN_COUNTOUR_AREA = 500

lost_count = 0
running = True
exit = False
# ================= HSV =================
#HSV 空间中，色调（H）的标准范围通常是 0 ~ 180
# OpenCV 特例，为了能用 8 位无符号整型 uchar 存储 360 度
# 饱和度（S）和明度（V）的范围是 0 ~ 255
LOWER_RED = np.array([0, 100, 100])
UPPER_RED = np.array([10, 255, 255])

LOWER_YELLOW = np.array([20, 100, 100])
UPPER_YELLOW = np.array([30, 255, 255])

LOWER_GREEN = np.array([35, 60, 60])
UPPER_GREEN = np.array([80, 255, 255])

# 退出逻辑
def exit_program():
    global running,exit
    if exit:
        return
    exit = True
    running = False
    print("程序已退出") 
    cv2.destroyAllWindows()

# 标记画面
def draw_center_cross(frame, arm_length=15, thickness=2):
    h, w = frame.shape[:2] #取图像数组 shape 的前两个维度，分别赋值给高和宽
    cx, cy = w // 2, h // 2
    cv2.line(frame, (cx - arm_length, cy), (cx + arm_length, cy), (0,0,0), thickness)
    cv2.line(frame, (cx, cy - arm_length), (cx, cy + arm_length), (0,0,0), thickness)

# 计算帧率
def get_fps(prev_time):
    current_time = cv2.getTickCount()
    delta_time = (current_time - prev_time) / cv2.getTickFrequency()
    fps = 1 / delta_time if delta_time > 0 else 0
    return int(fps), current_time

# 检测流程
def vision_loop():
    try:
        cap = cv2.VideoCapture(1)
        if not cap.isOpened():
            raise RuntimeError("摄像头打开失败:请检查摄像头是否接入/被占用")

        kernel = np.ones((5, 5), np.uint8)
        prev_time = cv2.getTickCount()

        valid_points = []   # 有效坐标

        while cap.isOpened() and running:
            ret, frame = cap.read()

            if not ret:
                print("视频读取失败")
                continue
            
            draw_center_cross(frame)

            fps, prev_time = get_fps(prev_time)
            cv2.putText(
                frame, f"fps:{fps}", FPS_POS,
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2
            )

            frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(frame_hsv, LOWER_GREEN, UPPER_GREEN)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            target_found = False

            if contours:
                max_contour = max(contours, key=cv2.contourArea)

                if cv2.contourArea(max_contour) > MIN_COUNTOUR_AREA:
                    moments = cv2.moments(max_contour)

                    if moments["m00"] != 0:
                        cx = int(moments["m10"] / moments["m00"])
                        cy = int(moments["m01"] / moments["m00"])

                        target_found = True

                        # 只保存有效检测结果
                        valid_points.append((cx, cy))

                        cv2.drawContours(
                            frame, [max_contour], -1, (0, 0, 255), 2
                        )
                        cv2.circle(frame, (cx, cy), 5, (0, 0, 0), -1)

                        cv2.putText(
                            frame,
                            f"Tracking ({cx},{cy})",
                            (cx + 10, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (255, 255, 255), 2
                        )

            if not target_found:
                cv2.putText(
                    frame, "TARGET LOST", (200, 240),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2
                )

            cv2.putText(
                frame,
                f"Valid samples: {len(valid_points)}/60",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2
            )

            cv2.imshow("frame", frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

            # 收集满 60 个有效坐标后退出
            if len(valid_points) >= 60:
                break

        cap.release()
        cv2.destroyAllWindows()

        if not valid_points:
            raise RuntimeError("没有检测到有效目标，无法计算平均坐标")

        points = np.asarray(valid_points, dtype=np.float64)
        pixel_x, pixel_y = np.mean(points, axis=0)

        print(f"有效采样数：{len(valid_points)}")
        print(f"平均像素坐标：({pixel_x:.2f}, {pixel_y:.2f})")

        return pixel_x, pixel_y
    except Exception as e:
        print("异常:",e)
        return None, None


# if __name__ == "__main__":
#     pixel_x, pixel_y = vision_loop()