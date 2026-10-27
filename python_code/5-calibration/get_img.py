import cv2
import os
import threading

FRAME_SIZE = (640, 480)

def capture_and_save_images():
    # 创建保存图片的文件夹
    base_path = r"D:/OpenCV/5-calibration"
    #形成新路径
    save_dir = os.path.join(base_path, "handeyes_calib_images2")

    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
        print(f"[+] 创建文件夹: {save_dir}")

    img_count = 0
    cap = cv2.VideoCapture(1)

    if not cap.isOpened():
        print("[-] 无法打开摄像头，请检查设备连接。")
        return
    else:
        print("摄像头打开中...")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_SIZE[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_SIZE[1])

    print("==========================================")
    print("操作指南:")
    print("1. 鼠标左键：保存当前帧")
    print("2. 鼠标右键：退出程序")
    print("==========================================")

    # 保存当前帧，供鼠标回调函数使用
    current_frame = None
    running = True

    def mouse_callback(event, x, y, flags, param):
        nonlocal img_count, current_frame, running

        # 鼠标左键保存当前帧
        if event == cv2.EVENT_LBUTTONDOWN:
            if current_frame is not None:
                img_count += 1
                file_name = os.path.join(
                    save_dir, f"calib_{img_count:03d}.jpg"
                )
                # cv2.imwrite(保存路径字符串, opencv图像数组)
                if cv2.imwrite(file_name, current_frame):
                    print(f"[+] 已保存图像: {file_name}")
                else:
                    img_count -= 1
                    print("[-] 图片保存失败")

        # 鼠标右键退出程序
        elif event == cv2.EVENT_RBUTTONDOWN:
            print("[+] 正在退出...")
            running = False

    window_name = "frame"
    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, mouse_callback)

    while cap.isOpened() and running:
        ret, frame = cap.read()

        if not ret:
            print("[-] 无法获取视频帧，退出...")
            break

        # 调整分辨率
        frame = cv2.resize(frame, FRAME_SIZE)

        # 保存当前帧的副本，避免后续画面修改影响保存内容
        current_frame = frame.copy()

        # 显示已保存图片数量
        cv2.putText(
            frame,
            f"Saved: {img_count}",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
            cv2.LINE_AA
        )

        cv2.putText(
            frame,
            "Left Click: Save | Right Click: Quit",
            (10, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
            cv2.LINE_AA
        )

        cv2.imshow(window_name, frame)

        # 处理窗口事件，鼠标回调依赖此函数运行
        cv2.waitKey(1)

    # 释放摄像头并关闭窗口
    cap.release()
    cv2.destroyAllWindows()

    print(f"[+] 采集结束，共保存了 {img_count} 张标定图片。")


thread = threading.Thread(
    target=capture_and_save_images,
    daemon=False
)
thread.start()
thread.join()

print("[+] 主线程退出，所有子线程已结束。")