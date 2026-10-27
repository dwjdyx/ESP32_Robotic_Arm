import numpy as np
import cv2
import glob
#数据准备 -特征提取 -矩阵求解 -精度评估 -结果保存
# ================= 配置参数 =================
CHESSBOARD_SIZE = (9, 6)  # 棋盘格内部角点的行列数（注意：是内角点，不是格子数）
SQUARE_SIZE = 0.027  # 每个棋盘格方块的实际物理尺寸（单位：米）
IMAGES_PATH = 'D:/OpenCV/5-calibration/camera_calib_images2/*.jpg'  # 标定图片所在的文件夹路径

# ===========================================

def calibrate_camera():
    # 终止迭代条件：当达到最大循环次数 30 或精度达到 0.001 时停止角点精细化
    # cv2.TERM_CRITERIA_MAX_ITER：达到最大迭代次数
    # cv2.TERM_CRITERIA_EPS：精度变化低于设定阈值
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # 1. 准备世界坐标系中的 3D 点，例如 (0,0,0), (1,0,0), (2,0,0) ...
    # 假设棋盘格固定在 Z=0 的平面上
    # (CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1]行, 3列(代表XYZ)二维矩阵
    objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
    # 生成一个网格坐标(对XY进行赋值，Z默认0)
    objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE  # 转换为实际物理尺寸

    objpoints = []  # 存储所有图像中真实的 3D 点
    imgpoints = []  # 存储所有图像中检测到的 2D 像素点

    # 读取标定图片 返回的是图片路径
    images = glob.glob(IMAGES_PATH)
    if not images:
        print(f"[-] 未找到标定图片，请检查路径: {IMAGES_PATH}")
        return

    print(f"[+] 找到 {len(images)} 张图片，开始处理...")

    valid_images_count = 0
    img_shape = None
    for fname in images:
        img = cv2.imread(fname)
        if img is None:
            print(f"[-] 图片读取失败: {fname}")
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        if img_shape is None:
            img_shape = gray.shape[::-1]  # 记录尺寸,顺序反转

        # 2. 寻找棋盘格角点，保存坐标(找出的角点像素坐标只是整数级别的)
        # 输入8位无符号整型单通道灰度图
        ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)

        # 如果找到了，添加对应的 3D 点和 2D 像素点
        if ret:
            objpoints.append(objp)

            # 亚像素级角点精细化，提高标定精度(像素坐标精确到小数位（亚像素级别）)
            #当计算了 30 次，或者精度达到了 0.001 时停止迭代
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            # 可选：绘制并显示角点检测结果
            cv2.drawChessboardCorners(img, CHESSBOARD_SIZE, corners2, ret)
            cv2.imshow('Chessboard Detection', img)
            cv2.waitKey(100)

            valid_images_count += 1
        else:
            print(f"[-] 图片未成功检测到棋盘格角点: {fname}")

    cv2.destroyAllWindows()
    print(f"[+] 成功处理有效图片数量: {valid_images_count}/{len(images)}")

    if valid_images_count < 10:
        print("[-] 有效标定图片较少（建议 15-20 张以上），标定结果可能存在较大误差。")

    if len(objpoints) < 10:
        print("[-] 有效标定图片不足，无法可靠标定")
        return

    # 3. 执行相机标定，求解相机内参矩阵
    # mtx：相机内参矩阵
    # dist：畸变系数
    # rvecs, tvecs：每张标定图片对应的旋转向量和位移向量
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, img_shape, None, None
    )

    print("\n================ 相机标定结果 ================")
    print(f"1. 标定是否成功: {ret}")
    print(f"2. 相机内参矩阵 (Camera Matrix / Intrinsic):\n{mtx}")
    print(f"3. 畸变系数 (Distortion Coefficients):\n{dist.ravel()}")

    # 计算重投影误差，评估标定质量
    # 看看算出来的 2D 点和实际检测到的 2D 角点（imgpoints）偏离了多少
    # 计算重投影误差，评估标定质量
    total_error = 0
    total_points = 0
    for i in range(len(objpoints)):
        # 把现实中的 3D 点（objpoints）重新投影回图像上
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)

        # 【修改点】将两者的 shape 统一转换为 (N, 2)，避免维度不匹配报错
        err = cv2.norm(imgpoints[i].reshape(-1, 2), imgpoints2.reshape(-1, 2), cv2.NORM_L2)

        total_error += err ** 2  # 通常重投影误差采用均方根误差 (RMSE) 的思路累加
        total_points += len(objpoints[i])

    mean_error = np.sqrt(total_error / total_points)
    print(f"4. 平均重投影误差 (Reprojection Error): {mean_error:.4f} 像素")
    print("==============================================")

    # 4. 保存标定参数到本地文件
    np.savez('camera_calib_params.npz', mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)
    print("[+] 标定参数已成功保存至 camera_calib_params.npz")

def test_undistort():
    # 读取标定参数
    data = np.load("camera_calib_params.npz")
    mtx = data["mtx"]
    dist = data["dist"]

    # 读取一张用于测试的图片
    img = cv2.imread("D:/OpenCV/5-calibration/camera_calib_images2/calib_046.jpg")

    if img is None:
        print("图片读取失败，请检查路径")
        return

    # 去畸变
    undistorted = cv2.undistort(img, mtx, dist)

    # 对比显示
    cv2.imshow("Original", img)
    cv2.imshow("Undistorted", undistorted)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    calibrate_camera()
    #test_undistort()
