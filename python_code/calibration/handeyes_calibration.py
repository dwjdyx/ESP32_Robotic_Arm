import cv2
import numpy as np
from pathlib import Path

from homogeneous_transform import forward_kinematics
from scipy.spatial.transform import Rotation

# ============================================================
# 1. 配置参数
# ============================================================
IMG_DIR = Path(r"D:\ESP32-S3\2-esp_project\6-5DOF_arm\python_code\calibration")
BASE_DIR = Path(r"D:\ESP32-S3\2-esp_project\6-5DOF_arm\python_code\detection")
# 棋盘格图片目录
IMAGE_DIR = IMG_DIR / "handeyes_initial"

# 关节角度文件:每行 theta1,theta2,theta3,theta4,theta5
ANGLE_FILE = BASE_DIR / "joints_init.txt"

# 相机内参文件
CAMERA_FILE = BASE_DIR / "camera_calib_params.npz"

# 手眼标定结果保存路径
OUTPUT_FILE = BASE_DIR / "hand_eye_result.npz"

# 棋盘格内角点数量(列数, 行数),不是方格数量
PATTERN_SIZE = (5, 6)

# 棋盘格相邻内角点之间的实际距离,单位 mm
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
    返回 shape=(角点数量, 3),单位 mm。
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
    theta1,theta2,theta3,theta4,theta5,theta6

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

    if angles.shape[1] != 6:
        raise ValueError(
            f"关节角度文件每行必须有 6 个角度,当前列数为 {angles.shape[1]}"
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
        T06:4x4 齐次变换矩阵
        R06:3x3 旋转矩阵
        p06:3 维平移向量

    注意:
    1. 这里假设 forward_kinematics 的角度输入单位为 degree。
    2. R06、p06 必须对应末端相对于基座的位姿。
    3. p06 的单位应为 mm,与棋盘格尺寸单位一致。
    """
    result = forward_kinematics(theta)

    if not isinstance(result, (tuple, list)) or len(result) != 3:
        raise ValueError(
            "forward_kinematics(theta) 必须返回 (T06, R06, p06)"
        )

    T06, R06, p06 = result

    T06 = np.asarray(T06, dtype=np.float64)
    R06 = np.asarray(R06, dtype=np.float64)
    p06 = np.asarray(p06, dtype=np.float64).reshape(3, 1)

    if T06.shape != (4, 4):
        raise ValueError(f"T06 应为 4x4,实际形状为 {T06.shape}")

    if R06.shape != (3, 3):
        raise ValueError(f"R06 应为 3x3,实际形状为 {R06.shape}")

    if not np.all(np.isfinite(T06)) or not np.all(np.isfinite(R06)):
        raise ValueError("正运动学返回的变换矩阵中存在无效数值")

    return R06, p06


# ============================================================
# 5. 检测棋盘格并求棋盘格相对于相机的位姿
# ============================================================

def estimate_chessboard_pose(image, object_points, camera_matrix, dist_coeffs):
    """
    检测棋盘格内角点,并使用 solvePnP 求棋盘格相对于相机的位姿。

    返回:
        success
        R_target2cam:棋盘格坐标系到相机坐标系的旋转矩阵
        t_target2cam:棋盘格坐标系原点在相机坐标系中的位置,单位 mm
        corners:检测到的棋盘格角点
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    found, corners = cv2.findChessboardCornersSB(
        gray,
        PATTERN_SIZE,
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

    # 计算重投影误差
    projected_points, _ = cv2.projectPoints(
        object_points,
        rvec,
        tvec,
        camera_matrix,
        dist_coeffs
    )

    errors = corners.reshape(-1, 2) - projected_points.reshape(-1, 2)

    # RMS 重投影误差
    reprojection_error = np.sqrt(np.mean(np.sum(errors ** 2, axis=1)))

    # 最大单点重投影误差
    max_reprojection_error = np.max(
        np.linalg.norm(errors, axis=1)
    )

    print(f"重投影误差(RMS): {reprojection_error:.4f} 像素")
    print(f"最大单点重投影误差: {max_reprojection_error:.4f} 像素")

    return True, R_target2cam, t_target2cam, corners

def build_transform(R, t):
    """根据旋转矩阵和平移向量构建齐次变换矩阵"""
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t).reshape(3)

    return T

def calibrate_hand_eye(
    R_gripper2base,
    t_gripper2base,
    R_target2cam_list,
    t_target2cam_list,
    method
):
    """执行手眼标定,返回相机到末端的变换"""

    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        R_gripper2base,
        t_gripper2base,
        R_target2cam_list,
        t_target2cam_list,
        method=method
    )

    T_cam2gripper = build_transform(
        R_cam2gripper,
        t_cam2gripper
    )

    return R_cam2gripper, t_cam2gripper, T_cam2gripper

def validate_hand_eye( R_gripper2base, t_gripper2base, R_target2cam_list, t_target2cam_list, T_cam2gripper):
    """计算每组数据下标定板在基座坐标系中的位姿"""
    board_poses = []

    for R_g2b, t_g2b, R_t2c, t_t2c in zip(
        R_gripper2base,
        t_gripper2base,
        R_target2cam_list,
        t_target2cam_list
    ):
        T_base_gripper = build_transform(R_g2b, t_g2b)
        T_camera_target = build_transform(R_t2c, t_t2c)

        T_base_target = (
            T_base_gripper
            @ T_cam2gripper
            @ T_camera_target
        )

        board_poses.append(T_base_target)

    return board_poses

def calculate_board_error(board_poses):
    """统计标定板位置和姿态相对于平均位姿的偏差"""

    from scipy.spatial.transform import Rotation

    board_positions = np.array([
        T[:3, 3] for T in board_poses
    ])

    board_rotations = np.array([
        T[:3, :3] for T in board_poses
    ])

    # 平均位置
    position_mean = np.mean(board_positions, axis=0)

    # 平均旋转矩阵
    mean_rotation = Rotation.from_matrix(
        board_rotations
    ).mean().as_matrix()

    # 每组样本相对于平均位置的偏差
    position_errors = np.linalg.norm(
        board_positions - position_mean,
        axis=1
    )

    # 每组样本相对于平均姿态的偏差
    rotation_errors = []

    for R in board_rotations:
        R_diff = mean_rotation.T @ R
        angle = Rotation.from_matrix(R_diff).magnitude()
        rotation_errors.append(np.rad2deg(angle))

    position_std = np.std(board_positions, axis=0)

    print("\n========== 标定板位置一致性 ==========")
    print("平均位置(mm):", position_mean)
    print("位置标准差(mm):", position_std)
    print("平均位置偏差(mm):", np.mean(position_errors))
    print("最大位置偏差(mm):", np.max(position_errors))

    print("\n========== 标定板姿态一致性 ==========")
    print("平均姿态偏差(deg):", np.mean(rotation_errors))
    print("最大姿态偏差(deg):", np.max(rotation_errors))

    return position_mean, position_std, position_errors, rotation_errors

def compare_hand_eye_methods(
    R_gripper2base,
    t_gripper2base,
    R_target2cam_list,
    t_target2cam_list
):
    """对比 OpenCV 3种手眼标定算法"""

    methods = {
        "TSAI": cv2.CALIB_HAND_EYE_TSAI,
        "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
        "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF
    }

    results = {}

    for name, method in methods.items():

        R_c2g, t_c2g, T_c2g = calibrate_hand_eye( R_gripper2base, t_gripper2base, R_target2cam_list, t_target2cam_list, method)
        board_poses = validate_hand_eye( R_gripper2base, t_gripper2base, R_target2cam_list, t_target2cam_list, T_c2g)

        print(f"\n========== {name} ==========")
        print("旋转矩阵:\n", R_c2g)
        print("平移向量(mm):", t_c2g.reshape(3))

        position_mean, position_std, position_errors, rotation_errors = (
            calculate_board_error(board_poses)
        )

        results[name] = {
            "R_cam2gripper": R_c2g,
            "t_cam2gripper": t_c2g,
            "T_cam2gripper": T_c2g,
            "position_mean": position_mean,
            "position_std": position_std,
            "position_errors": position_errors,
            "rotation_errors": rotation_errors
        }

    return results

# ============================================================
# 6. 主程序
# ============================================================

def main():
    # 6.1 检查输入路径
    if not IMAGE_DIR.is_dir():
        raise FileNotFoundError(f"找不到图片目录:{IMAGE_DIR}")

    if not CAMERA_FILE.is_file():
        raise FileNotFoundError(f"找不到相机内参文件:{CAMERA_FILE}")

    # 6.2 读取相机内参
    with np.load(CAMERA_FILE) as camera_data:
        camera_matrix = np.asarray( camera_data["mtx"], dtype=np.float64)
        dist_coeffs = np.asarray( camera_data["dist"], dtype=np.float64)

    # 6.3 读取关节角度和图片
    joint_angles = load_joint_angles(ANGLE_FILE)

    # 文件名排序后与 joint_angles.txt 的行顺序一一对应
    image_paths = sorted(
        path for path in IMAGE_DIR.iterdir()
        if path.is_file() and path.suffix.lower() in {".jpg", ".jpeg", ".png"}
    )

    if len(image_paths) != len(joint_angles):
        raise ValueError(
            f"图片数量为 {len(image_paths)},"
            f"关节角度组数为 {len(joint_angles)};"
            "两者必须相同,并且排序后逐行对应。"
        )

    # print(f"\n图片数量:{len(image_paths)}")
    # print(f"关节角度组数:{len(joint_angles)}")

    # 6.4 创建棋盘格三维角点
    object_points = create_chessboard_points( PATTERN_SIZE, SQUARE_SIZE,)

    # 6.5 初始化手眼标定所需数据
    R_gripper2base = []
    t_gripper2base = []
    R_target2cam_list = []
    t_target2cam_list = []

    # 记录成功处理的图片
    valid_image_names = []
    failed_image_names = []

    # 6.6 逐张处理图片
    for index, (image_path, theta) in enumerate(
        zip(image_paths, joint_angles), start=1
    ):
        print(f"\n[{index}/{len(image_paths)}] 处理图片:{image_path.name}")
        print("关节角度(degree):", theta)

        image = cv2.imread(str(image_path))
        if image is None:
            print("图片读取失败,跳过该组数据")
            failed_image_names.append(image_path.name)
            continue

        # A. 根据拍摄时的关节角度计算末端相对于基座的位姿
        try:
            R06, p06 = get_robot_pose(theta)
        except (ValueError, TypeError) as exc:
            print(f"正运动学计算失败,跳过该组数据:{exc}")
            failed_image_names.append(image_path.name)
            continue

        # B. 检测棋盘格,并求棋盘格相对于相机的位姿
        success, R_target2cam, t_target2cam, _ = estimate_chessboard_pose(
            image,
            object_points,
            camera_matrix,
            dist_coeffs,
        )

        if not success:
            print("棋盘格检测或 solvePnP 失败,跳过该组数据")
            failed_image_names.append(image_path.name)
            continue

        distance = np.linalg.norm(t_target2cam)
        print(f"标定板到相机光心距离: {distance:.2f} mm")

        # C. 只有这一张图片的两部分计算都成功时,才同时保存四组数据
        R_gripper2base.append(
            np.asarray(R06, dtype=np.float64)
        )
        t_gripper2base.append(
            np.asarray(p06, dtype=np.float64).reshape(3, 1)
        )
        R_target2cam_list.append(
            np.asarray(R_target2cam, dtype=np.float64)
        )
        t_target2cam_list.append(
            np.asarray(t_target2cam, dtype=np.float64).reshape(3, 1)
        )

        valid_image_names.append(image_path.name)

        # print("棋盘格检测成功")
        # print("末端位置(mm):", p06.ravel())
        # print("棋盘格原点在相机坐标系中的位置(mm):", t_target2cam.ravel())

    # 6.7 检查有效样本数量
    sample_count = len(R_gripper2base)
    print(f"\n有效标定样本数:{sample_count}")
    print(f"失败图片数:{len(failed_image_names)}")

    if failed_image_names:
        print("失败图片:")
        for name in failed_image_names:
            print("  ", name)

    if sample_count < 10:
        raise RuntimeError(
            f"当前有效样本只有 {sample_count} 组,数量偏少。"
            "请检查图片、棋盘格检测结果和关节角度对应关系。"
        )

    # --------------------------------------------------------
    # 6.8 五种手眼标定算法对比
    # --------------------------------------------------------
    results = compare_hand_eye_methods(
        R_gripper2base,
        t_gripper2base,
        R_target2cam_list,
        t_target2cam_list
    )

    # --------------------------------------------------------
    # 6.9 使用指定算法的结果
    # --------------------------------------------------------
    R_cam2gripper, t_cam2gripper, T_cam2gripper = (
        calibrate_hand_eye(
            R_gripper2base,
            t_gripper2base,
            R_target2cam_list,
            t_target2cam_list,
            HAND_EYE_METHOD
        )
    )

    # ============================================================
    # 逐样本检查棋盘格在基座坐标系下的位置和姿态
    # ============================================================

    T_cam2gripper = np.eye(4)
    T_cam2gripper[:3, :3] = R_cam2gripper
    T_cam2gripper[:3, 3] = t_cam2gripper.reshape(3)

    board_poses = []

    for i in range(sample_count):

        # 末端 -> 基座
        T_base_gripper = np.eye(4)
        T_base_gripper[:3, :3] = R_gripper2base[i]
        T_base_gripper[:3, 3] = np.asarray(
            t_gripper2base[i]
        ).reshape(3)

        # 棋盘格 -> 相机
        T_camera_target = np.eye(4)
        T_camera_target[:3, :3] = R_target2cam_list[i]
        T_camera_target[:3, 3] = np.asarray(
            t_target2cam_list[i]
        ).reshape(3)

        # 棋盘格 -> 基座
        T_base_target = (
            T_base_gripper @ T_cam2gripper @ T_camera_target
        )

        board_poses.append(T_base_target)
        

    # 计算平均位姿及每组样本偏差
    mean_position, position_std, position_errors, rotation_errors = (
        calculate_board_error(board_poses)
    )
    print("\n========== 逐样本一致性分析 ==========")

    for i in range(sample_count):
        print(
            f"{valid_image_names[i]:20s} | "
            f"位置偏差: {position_errors[i]:.2f} mm | "
            f"姿态偏差: {rotation_errors[i]:.2f} deg"
        )

    # 6.10 输出并保存标定结果
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