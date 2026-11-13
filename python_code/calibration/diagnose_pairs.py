import cv2
import numpy as np
from pathlib import Path
from homogeneous_transform import forward_kinematics
from scipy.spatial.transform import Rotation

BASE = Path(r"D:\ESP32-S3\2-esp_project\6-5DOF_arm\python_code\detection")
IMG_DIR = Path(r"D:\ESP32-S3\2-esp_project\6-5DOF_arm\python_code\calibration\handeyes_calib_images")
ANGLE_FILE = BASE / "joints.txt"
CAMERA_FILE = BASE / "camera_calib_params.npz"
PATTERN = (9, 6)
SQUARE = 27.0

def build(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t).reshape(3)
    return T

def rot_angle(R):
    return np.rad2deg(Rotation.from_matrix(R).magnitude())

with np.load(CAMERA_FILE) as d:
    mtx = np.asarray(d["mtx"], float)
    dist = np.asarray(d["dist"], float)

angles = np.loadtxt(ANGLE_FILE, delimiter=",", ndmin=2)
img_paths = sorted(p for p in IMG_DIR.iterdir() if p.suffix.lower() in {".jpg", ".png", ".jpeg"})

objp = np.zeros((PATTERN[0] * PATTERN[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:PATTERN[0], 0:PATTERN[1]].T.reshape(-1, 2) * SQUARE

T_g2b = []
T_t2c = []
names = []
for ip, theta in zip(img_paths, angles):
    img = cv2.imread(str(ip))
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ok, corners = cv2.findChessboardCorners(gray, PATTERN, None)
    if not ok:
        print("NO BOARD:", ip.name)
        continue
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                               (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
    _, rvec, tvec = cv2.solvePnP(objp, corners, mtx, dist)
    R_t2c, _ = cv2.Rodrigues(rvec)
    T_t2c.append(build(R_t2c, tvec))

    _, R_g2b, p = forward_kinematics(theta)
    T_g2b.append(build(R_g2b, p))
    names.append(ip.name)

n = len(T_g2b)
print(f"valid samples: {n}")

diffs = []
worst = []
for i in range(n):
    for j in range(i + 1, n):
        A = np.linalg.inv(T_g2b[i]) @ T_g2b[j]      # gripper motion i->j
        B = T_t2c[i] @ np.linalg.inv(T_t2c[j])      # target motion i->j
        a = rot_angle(A[:3, :3])
        b = rot_angle(B[:3, :3])
        d = abs(a - b)
        diffs.append(d)
        worst.append((d, names[i], names[j], a, b))

diffs = np.array(diffs)
print(f"\nrotation-angle mismatch (A vs B), over {len(diffs)} pairs:")
print(f"  mean = {diffs.mean():.2f} deg")
print(f"  median = {np.median(diffs):.2f} deg")
print(f"  max = {diffs.max():.2f} deg")

worst.sort(reverse=True)
print("\nworst 10 pairs (mismatch, img_i, img_j, gripper_angle, target_angle):")
for d, ni, nj, a, b in worst[:10]:
    print(f"  {d:7.2f}  {ni} <-> {nj}   A={a:6.2f}  B={b:6.2f}")
