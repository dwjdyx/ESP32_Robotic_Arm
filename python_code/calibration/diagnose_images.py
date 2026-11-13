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

with np.load(CAMERA_FILE) as d:
    mtx = np.asarray(d["mtx"], float)
    dist = np.asarray(d["dist"], float)

angles = np.loadtxt(ANGLE_FILE, delimiter=",", ndmin=2)
img_paths = sorted(p for p in IMG_DIR.iterdir() if p.suffix.lower() in {".jpg", ".png", ".jpeg"})

objp = np.zeros((PATTERN[0] * PATTERN[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:PATTERN[0], 0:PATTERN[1]].T.reshape(-1, 2) * SQUARE

print("img          reproj(px)  dist(mm)  normal_rvec(deg)  theta")
for ip, theta in zip(img_paths, angles):
    img = cv2.imread(str(ip))
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ok, corners = cv2.findChessboardCorners(gray, PATTERN, None)
    if not ok:
        print(f"{ip.name}  NO BOARD   theta={theta}")
        continue
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                               (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
    _, rvec, tvec = cv2.solvePnP(objp, corners, mtx, dist)
    proj, _ = cv2.projectPoints(objp, rvec, tvec, mtx, dist)
    err = np.linalg.norm((proj.reshape(-1, 2) - corners.reshape(-1, 2)), axis=1).mean()
    dist_cam = np.linalg.norm(tvec)
    normal = np.rad2deg(np.linalg.norm(rvec))
    print(f"{ip.name}  {err:9.4f}   {dist_cam:7.1f}   {normal:7.2f}         {theta}")
