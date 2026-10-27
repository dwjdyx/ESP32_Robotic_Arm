import cv2
import numpy as np
from pathlib import Path

from homogeneous_transform import forward_kinematics


# ============================================================
# 1. 配置参数
# ============================================================

# 棋盘格图片目录
IMAGE_DIR = Path(r"D:\OpenCV\5-calibration\handeyes_calib_images2")

# 关节角度文件:每行 theta1,theta2,theta3,theta4,theta5
ANGLE_FILE = Path(r"D:\OpenCV\5-calibration\joint_angles.txt")

# 相机内参文件
CAMERA_FILE = Path(r"D:\OpenCV\5-calibration\camera_calib_params.npz")

# 手眼标定结果保存路径
OUTPUT_FILE = Path(r"D:\OpenCV\5-calibration\hand_eye_result.npz")

# 棋盘格内角点数量(列数, 行数)，不是方格数量
PATTERN_SIZE = (9, 6)

# 棋盘格相邻内角点之间的实际距离，单位 mm
SQUARE_SIZE = 27.0

# 手眼标定算法
HAND_EYE_METHOD = cv2.CALIB_HAND_EYE_TSAI

# 棋盘格检测亚像素优化条件
SUBPIX_CRITERIA = (
    cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
    30,
    0.001,
)


# ============================================================
# 2. 创建棋盘格三维角点
# ============================================================

def create_chessboard_points(pattern_size, square_size):
    """
    创建棋盘格内角点在棋盘格坐标系中的三维坐标。

    棋盘格位于 Z=0 平面。
    pattern_size 为 (列数, 行数)。
    返回 shape=(角点数量, 3)，单位 mm。
    """
    cols, rows = pattern_size

    object_points = np.zeros((rows * cols, 3), dtype=np.float32)
    object_points[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    object_points *= square_size

    return object_points


# ============================================================
# 3. 读取关节角度
# ============================================================

def load_joint_angles(file_path):
    """
    读取关节角度文件。

    每行格式:
    theta1,theta2,theta3,theta4,theta5

    角度单位为 degree。
    """
    if not file_path.is_file():
        raise FileNotFoundError(f"找不到关节角度文件:{file_path}")

    angles = np.loadtxt(
        file_path,
        delimiter=",",
        comments="#",
        ndmin=2,
        dtype=np.float64,
    )

    if angles.shape[1] != 5:
        raise ValueError(
            f"关节角度文件每行必须有 5 个角度，当前列数为 {angles.shape[1]}"
        )

    if not np.all(np.isfinite(angles)):
        raise ValueError("关节角度文件中存在无效数值")

    return angles


# ============================================================
# 4. 计算机械臂末端相对于基座的位姿
# ============================================================

def get_robot_pose(theta):
    """
    根据当前五个关节角度计算末端相对于基座的位姿。

    forward_kinematics(theta) 应返回:
        T05:4x4 齐次变换矩阵
        R05:3x3 旋转矩阵
        p05:3 维平移向量

    注意:
    1. 这里假设 forward_kinematics 的角度输入单位为 degree。
    2. R05、p05 必须对应末端相对于基座的位姿。
    3. p05 的单位应为 mm，与棋盘格尺寸单位一致。
    """
    result = forward_kinematics(theta)

    if not isinstance(result, (tuple, list)) or len(result) != 3:
        raise ValueError(
            "forward_kinematics(theta) 必须返回 (T05, R05, p05)"
        )

    T05, R05, p05 = result

    T05 = np.asarray(T05, dtype=np.float64)
    R05 = np.asarray(R05, dtype=np.float64)
    p05 = np.asarray(p05, dtype=np.float64).reshape(3, 1)

    if T05.shape != (4, 4):
        raise ValueError(f"T05 应为 4x4，实际形状为 {T05.shape}")

    if R05.shape != (3, 3):
        raise ValueError(f"R05 应为 3x3，实际形状为 {R05.shape}")

    if not np.all(np.isfinite(T05)) or not np.all(np.isfinite(R05)):
        raise ValueError("正运动学返回的变换矩阵中存在无效数值")

    return R05, p05


# ============================================================
# 5. 检测棋盘格并求棋盘格相对于相机的位姿
# ============================================================

def estimate_chessboard_pose(image, object_points, camera_matrix, dist_coeffs):
    """
    检测棋盘格内角点，并使用 solvePnP 求棋盘格相对于相机的位姿。

    返回:
        success
        R_target2cam:棋盘格坐标系到相机坐标系的旋转矩阵
        t_target2cam:棋盘格坐标系原点在相机坐标系中的位置，单位 mm
        corners:检测到的棋盘格角点
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    found, corners = cv2.findChessboardCorners(
        gray,
        PATTERN_SIZE,
        None,
    )

    if not found:
        return False, None, None, None

    corners = cv2.cornerSubPix(
        gray,
        corners,
        (11, 11),
        (-1, -1),
        SUBPIX_CRITERIA,
    )

    success, rvec, tvec = cv2.solvePnP(
        object_points,
        corners,
        camera_matrix,
        dist_coeffs,
    )

    if not success:
        return False, None, None, corners

    R_target2cam, _ = cv2.Rodrigues(rvec)
    t_target2cam = np.asarray(tvec, dtype=np.float64).reshape(3, 1)

    return True, R_target2cam, t_target2cam, corners


# ============================================================
# 6. 主程序
# ============================================================

def main():
    # --------------------------------------------------------
    # 6.1 检查输入路径
    # --------------------------------------------------------
    if not IMAGE_DIR.is_dir():
        raise FileNotFoundError(f"找不到图片目录:{IMAGE_DIR}")

    if not CAMERA_FILE.is_file():
        raise FileNotFoundError(f"找不到相机内参文件:{CAMERA_FILE}")

    # ============================================================
    # 6.2 读取相机内参
    # ============================================================
    with np.load(CAMERA_FILE) as camera_data:
        camera_matrix = np.asarray(
            camera_data["mtx"],
            dtype=np.float64
        )

        dist_coeffs = np.asarray(
            camera_data["dist"],
            dtype=np.float64
        )

    print("相机内参矩阵:")
    print(camera_matrix)

    print("\n相机畸变系数:")
    print(dist_coeffs)

    # --------------------------------------------------------
    # 6.3 读取关节角度和图片
    # --------------------------------------------------------
    joint_angles = load_joint_angles(ANGLE_FILE)

    # 文件名排序后与 joint_angles.txt 的行顺序一一对应。
    # 建议图片使用 001.jpg、002.jpg 等零填充编号。
    image_paths = sorted(
        path for path in IMAGE_DIR.iterdir()
        if path.is_file() and path.suffix.lower() in {".jpg", ".jpeg", ".png"}
    )

    if len(image_paths) != len(joint_angles):
        raise ValueError(
            f"图片数量为 {len(image_paths)}，"
            f"关节角度组数为 {len(joint_angles)}；"
            "两者必须相同，并且排序后逐行对应。"
        )

    print(f"\n图片数量:{len(image_paths)}")
    print(f"关节角度组数:{len(joint_angles)}")

    # --------------------------------------------------------
    # 6.4 创建棋盘格三维角点
    # --------------------------------------------------------
    object_points = create_chessboard_points(
        PATTERN_SIZE,
        SQUARE_SIZE,
    )

    # --------------------------------------------------------
    # 6.5 初始化手眼标定所需数据
    # --------------------------------------------------------
    # OpenCV 输入约定:
    # R_gripper2base / t_gripper2base 表示 gripper -> base
    # R_target2cam / t_target2cam 表示 target -> camera
    R_gripper2base = []
    t_gripper2base = []
    R_target2cam_list = []
    t_target2cam_list = []

    # 记录成功处理的图片，便于排查
    valid_image_names = []
    failed_image_names = []

    # --------------------------------------------------------
    # 6.6 逐张处理图片
    # --------------------------------------------------------
    for index, (image_path, theta) in enumerate(
        zip(image_paths, joint_angles), start=1
    ):
        print(f"\n[{index}/{len(image_paths)}] 处理图片:{image_path.name}")
        print("关节角度(degree):", theta)

        image = cv2.imread(str(image_path))
        if image is None:
            print("图片读取失败，跳过该组数据")
            failed_image_names.append(image_path.name)
            continue

        # A. 根据拍摄时的关节角度计算末端相对于基座的位姿
        try:
            R05, p05 = get_robot_pose(theta)
        except (ValueError, TypeError) as exc:
            print(f"正运动学计算失败，跳过该组数据:{exc}")
            failed_image_names.append(image_path.name)
            continue

        # B. 检测棋盘格，并求棋盘格相对于相机的位姿
        success, R_target2cam, t_target2cam, _ = estimate_chessboard_pose(
            image,
            object_points,
            camera_matrix,
            dist_coeffs,
        )

        if not success:
            print("棋盘格检测或 solvePnP 失败，跳过该组数据")
            failed_image_names.append(image_path.name)
            continue

        # C. 只有这一张图片的两部分计算都成功时，才同时保存四组数据
        R_gripper2base.append(
            np.asarray(R05, dtype=np.float64)
        )
        t_gripper2base.append(
            np.asarray(p05, dtype=np.float64).reshape(3, 1)
        )
        R_target2cam_list.append(
            np.asarray(R_target2cam, dtype=np.float64)
        )
        t_target2cam_list.append(
            np.asarray(t_target2cam, dtype=np.float64).reshape(3, 1)
        )

        valid_image_names.append(image_path.name)

        print("棋盘格检测成功")
        print("末端位置(mm):", p05.ravel())
        print("棋盘格原点在相机坐标系中的位置(mm):", t_target2cam.ravel())

    # --------------------------------------------------------
    # 6.7 检查有效样本数量
    # --------------------------------------------------------
    sample_count = len(R_gripper2base)
    print(f"\n有效标定样本数:{sample_count}")
    print(f"失败图片数:{len(failed_image_names)}")

    if failed_image_names:
        print("失败图片:")
        for name in failed_image_names:
            print("  ", name)

    if sample_count < 10:
        raise RuntimeError(
            f"当前有效样本只有 {sample_count} 组，数量偏少。"
            "请检查图片、棋盘格检测结果和关节角度对应关系。"
        )

    # --------------------------------------------------------
    # 6.8 执行手眼标定
    # --------------------------------------------------------
    # 返回的变换是 camera -> gripper，即 ^gripper T_camera
    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        R_gripper2base,
        t_gripper2base,
        R_target2cam_list,
        t_target2cam_list,
        method=HAND_EYE_METHOD,
    )

    R_cam2gripper = np.asarray(R_cam2gripper, dtype=np.float64)
    t_cam2gripper = np.asarray(
        t_cam2gripper, dtype=np.float64
    ).reshape(3, 1)

    # --------------------------------------------------------
    # 6.9 组合成相机坐标系到末端坐标系的齐次变换矩阵
    # --------------------------------------------------------
    T_cam2gripper = np.eye(4, dtype=np.float64)
    T_cam2gripper[:3, :3] = R_cam2gripper
    T_cam2gripper[:3, 3] = t_cam2gripper.reshape(3)

    # --------------------------------------------------------
    # 6.10 输出并保存标定结果
    # --------------------------------------------------------
    np.set_printoptions(precision=5, suppress=True)

    print("\n相机坐标系到末端坐标系的旋转矩阵 R_cam2gripper:")
    print(R_cam2gripper)

    print("\n相机坐标系原点在末端坐标系中的位置 t_cam2gripper(mm):")
    print(t_cam2gripper)

    print("\n相机坐标系到末端坐标系的齐次变换矩阵 T_cam2gripper:")
    print(T_cam2gripper)

    OUTPUT_FILE.parent.mkdir(parents=True, exist_ok=True)

    np.savez(
        OUTPUT_FILE,
        R_cam2gripper=R_cam2gripper,
        t_cam2gripper=t_cam2gripper,
        T_cam2gripper=T_cam2gripper,
        valid_image_names=np.asarray(valid_image_names),
        failed_image_names=np.asarray(failed_image_names),
        sample_count=np.asarray(sample_count),
    )

    print(f"\n标定结果已保存到:{OUTPUT_FILE}")


if __name__ == "__main__":
    main()
    R = np.array([
        [0.95685, 0.09485, 0.27468],
        [-0.12088, 0.98948, 0.07943],
        [-0.26426, -0.10920, 0.95825]
    ])

    print("R转置乘R:")
    print(R.T @ R)

    print("行列式:", np.linalg.det(R))
