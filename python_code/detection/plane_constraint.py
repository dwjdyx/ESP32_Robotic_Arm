import cv2
import numpy as np
from pathlib import Path

from homogeneous_transform import forward_kinematics

# ============================================================
# 1. 文件路径与参数配置
# ============================================================

BASE_DIR = Path(r"D:\ESP32-S3\2-esp_project\6-5DOF_arm\python_code\detection")
CAMERA_FILE = BASE_DIR / "camera_calib_params.npz"
HAND_EYE_FILE = BASE_DIR / "hand_eye_result.npz"

# 桌面在机械臂基座坐标系中的 Z 坐标,单位 mm
TABLE_Z = -98.0

INIT_THETA = [14.0, -84.0, -204.0, -90.0, -90.0]

TEST_PIXEL = (257.06, 267.00)

# ============================================================
# 2. 读取相机内参与手眼标定结果
# ============================================================

def load_camera_params():
    if not CAMERA_FILE.is_file():
        raise FileNotFoundError(f"找不到相机内参文件:{CAMERA_FILE}")

    with np.load(CAMERA_FILE) as data:
        camera_matrix = np.asarray(data["mtx"], dtype=np.float64)
        dist_coeffs = np.asarray(data["dist"], dtype=np.float64)

    if camera_matrix.shape != (3, 3):
        raise ValueError(f"相机内参矩阵应为 3x3,实际为 {camera_matrix.shape}")

    return camera_matrix, dist_coeffs


def load_hand_eye():
    if not HAND_EYE_FILE.is_file():
        raise FileNotFoundError(f"找不到手眼标定文件:{HAND_EYE_FILE}")

    with np.load(HAND_EYE_FILE) as data:
        # T_cam2gripper:将相机坐标系中的点变换到末端/夹爪坐标系。
        T_gripper_camera = np.asarray(data["T_cam2gripper"], dtype=np.float64)

    if T_gripper_camera.shape != (4, 4):
        raise ValueError(
            f"T_cam2gripper 应为 4x4,实际为 {T_gripper_camera.shape}"
        )

    return T_gripper_camera


# ============================================================
# 3. 计算相机相对于基座的位姿
# ============================================================

def calculate_T_base_camera(theta, T_gripper_camera):
    # FK 返回基座到第 5 关节末端坐标系的变换。
    T_base_gripper, _, _ = forward_kinematics(theta)
    # 转成 numpy 数组
    T_base_gripper = np.asarray(T_base_gripper, dtype=np.float64)

    if T_base_gripper.shape != (4, 4):
        raise ValueError("forward_kinematics(theta) 返回的 T05 必须为 4x4")

    T_base_camera = T_base_gripper @ T_gripper_camera
    return T_base_gripper, T_base_camera

# ============================================================
# 4. 像素射线与桌面平面求交
# ============================================================

def pixel_to_table(u, v, camera_matrix, dist_coeffs, T_base_camera, table_z):
    try:
        pixel = np.array([[[u, v]]], dtype=np.float64)

        # 去畸变并转换为归一化相机坐标。
        normalized = cv2.undistortPoints(pixel, camera_matrix, dist_coeffs)
        x, y = normalized[0, 0]

        # 相机坐标系中的射线方向。
        d_camera = np.array([x, y, 1.0], dtype=np.float64)

        # 转换至基座坐标系。
        R_base_camera = T_base_camera[:3, :3]
        origin_base = T_base_camera[:3, 3]
        d_base = R_base_camera @ d_camera

        if abs(d_base[2]) < 1e-8:
            raise ValueError("相机射线与桌面近乎平行,无法求交")

        # 射线参数 lambda；交点为 origin_base + lambda * d_base。
        lam = (table_z - origin_base[2]) / d_base[2]
        print(f"相机在基座坐标系的位置: {origin_base}")
        print(f"相机射线在基座坐标系的方向: {d_base}")
        print(f"射线参数 lambda: {lam:.3f}")
        if lam <= 0:
            raise ValueError("桌面交点位于相机后方,请检查相机姿态、像素坐标和桌面高度")

        P_base = origin_base + lam * d_base
        P_base[2] = table_z
        return P_base
    
    except Exception as e:
        print("异常:",e)
        return None


# ============================================================
# 5. 主程序
# ============================================================

def pos_calculate(pixel):
    try:
        camera_matrix, dist_coeffs = load_camera_params()
        T_gripper_camera = load_hand_eye()

        T_base_gripper, T_base_camera = calculate_T_base_camera(
            INIT_THETA, T_gripper_camera
        )

        u, v = pixel
        P_base = pixel_to_table(
            u, v, camera_matrix, dist_coeffs, T_base_camera, TABLE_Z
        )
        if P_base is None:
            raise ValueError("P_base无法计算")

        np.set_printoptions(precision=4, suppress=True)
        print(f"\n像素坐标 (u, v):({u}, {v})")
        print("目标点在基座坐标系中的坐标 (mm):")
        print(P_base)
        return P_base
    except Exception as e:
        print("异常:",e)
        return None


if __name__ == "__main__":
    pos_calculate(TEST_PIXEL)
