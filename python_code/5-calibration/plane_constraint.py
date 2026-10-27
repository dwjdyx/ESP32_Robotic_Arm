import cv2
import numpy as np
from pathlib import Path

# =========================
# 1. 文件路径
# =========================

BASE_DIR = Path(r"D:\OpenCV\5-calibration")

CAMERA_FILE = BASE_DIR / "camera_calib_params.npz"
HAND_EYE_FILE = BASE_DIR / "hand_eye_result.npz"

# =========================
# 2. 桌面参数
# =========================

# 桌面在机械臂基座坐标系中的 Z 坐标，单位 mm
TABLE_Z = 0.0


# =========================
# 3. 读取相机内参
# =========================

def load_camera_params():
    if not CAMERA_FILE.exists():
        raise FileNotFoundError(f"找不到相机内参文件：{CAMERA_FILE}")

    with np.load(CAMERA_FILE) as data:
        camera_matrix = np.asarray(data["mtx"], dtype=np.float64)
        dist_coeffs = np.asarray(data["dist"], dtype=np.float64)

    return camera_matrix, dist_coeffs


# =========================
# 4. 读取手眼标定结果
# =========================

def load_hand_eye():
    if not HAND_EYE_FILE.exists():
        raise FileNotFoundError(f"找不到手眼标定文件：{HAND_EYE_FILE}")

    with np.load(HAND_EYE_FILE) as data:
        # 相机坐标系到末端坐标系的变换
        T_gripper_camera = np.asarray(
            data["T_cam2gripper"],
            dtype=np.float64
        )

    return T_gripper_camera


# =========================
# 5. 计算相机相对于基座的变换
# =========================

def calculate_T_base_camera(T_base_gripper, T_gripper_camera):
    """
    T_base_gripper:
        末端坐标系相对于基座坐标系的变换，由 FK 得到。

    T_gripper_camera:
        相机坐标系到末端坐标系的变换，由手眼标定得到。

    返回:
        相机坐标系相对于基座坐标系的变换。
    """
    return T_base_gripper @ T_gripper_camera


# =========================
# 6. 像素坐标与桌面求交
# =========================

def pixel_to_table(
    u,
    v,
    camera_matrix,
    dist_coeffs,
    T_base_camera,
    table_z
):
    """
    将像素坐标转换为桌面上的基座坐标。

    返回值单位与 T_base_camera 的平移单位一致。
    本项目统一使用 mm。
    """

    # 1. 去畸变，获得归一化相机坐标
    pixel = np.array([[[u, v]]], dtype=np.float64)

    normalized = cv2.undistortPoints(
        pixel,
        camera_matrix,
        dist_coeffs
    )

    x = normalized[0, 0, 0]
    y = normalized[0, 0, 1]

    # 2. 相机坐标系中的射线方向
    d_camera = np.array([x, y, 1.0], dtype=np.float64)

    # 3. 转换到基座坐标系
    R_base_camera = T_base_camera[:3, :3]
    origin_base = T_base_camera[:3, 3]

    d_base = R_base_camera @ d_camera

    # 4. 判断射线是否与桌面近乎平行
    if abs(d_base[2]) < 1e-8:
        raise ValueError("相机射线与桌面近乎平行，无法求交")

    # 5. 求射线与桌面的交点
    lam = (table_z - origin_base[2]) / d_base[2]

    if lam <= 0:
        raise ValueError("桌面交点位于相机后方")

    P_base = origin_base + lam * d_base

    return P_base


# =========================
# 7. 主程序
# =========================

def main():
    # 读取相机参数
    camera_matrix, dist_coeffs = load_camera_params()

    # 读取手眼标定结果
    T_gripper_camera = load_hand_eye()

    # TODO：这里需要调用你的 FK 函数，得到当前姿态的 T_base_gripper
    # 下面仅为矩阵格式示例，不能直接用于实际抓取。
    #
    # T_base_gripper = forward_kinematics(theta)[0]

    raise NotImplementedError(
        "请接入你的 FK 函数，计算当前姿态的 T_base_gripper，"
        "再计算 T_base_camera。"
    )

    # 计算相机相对于基座的变换
    T_base_camera = calculate_T_base_camera(
        T_base_gripper,
        T_gripper_camera
    )

    # 示例像素坐标：图像中心
    u, v = 320, 240

    # 像素坐标转换为桌面上的基座坐标
    P_base = pixel_to_table(
        u,
        v,
        camera_matrix,
        dist_coeffs,
        T_base_camera,
        TABLE_Z
    )

    print("T_base_camera：")
    print(T_base_camera)

    print("目标基座坐标（mm）：")
    print(P_base)


if __name__ == "__main__":
    main()