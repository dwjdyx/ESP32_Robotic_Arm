import cv2
import numpy as np
from pathlib import Path
from itertools import product
from scipy.spatial.transform import Rotation

BASE = Path(r"D:\ESP32-S3\2-esp_project\6-5DOF_arm\python_code\detection")
IMG_DIR = Path(r"D:\ESP32-S3\2-esp_project\6-5DOF_arm\python_code\calibration\handeyes_images")
ANGLE_FILE = BASE / "joints.txt"
CAMERA_FILE = BASE / "camera_calib_params.npz"
PATTERN = (9, 6); SQUARE = 27.0

A = [0.0, 10.25, 105.0, 0.0, 0.0]
D = [0.0, 21.0, 21.0, 62.0, 107.0]

def build(R, t):
    T = np.eye(4); T[:3,:3]=R; T[:3,3]=np.asarray(t).reshape(3); return T

def mdh(a, al, d, th):
    al_, th_ = np.deg2rad(al), np.deg2rad(th)
    ca, sa = np.cos(al_), np.sin(al_); ct, st = np.cos(th_), np.sin(th_)
    return np.array([[ct,-st,0,a],[st*ca,ct*ca,-sa,-d*sa],[st*sa,ct*sa,ca,d*ca],[0,0,0,1]])

def fk(theta, alpha):
    T = np.eye(4)
    for i in range(5): T = T @ mdh(A[i], alpha[i], D[i], theta[i])
    return T

with np.load(CAMERA_FILE) as d:
    mtx = np.asarray(d["mtx"], float); dist = np.asarray(d["dist"], float)
thetas = np.loadtxt(ANGLE_FILE, delimiter=",", ndmin=2)
img_paths = sorted(p for p in IMG_DIR.iterdir() if p.suffix.lower() in {".jpg",".png",".jpeg"})
objp = np.zeros((54,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)*SQUARE
Ttc = []
for ip in img_paths:
    g = cv2.cvtColor(cv2.imread(str(ip)), cv2.COLOR_BGR2GRAY)
    ok,c = cv2.findChessboardCorners(g, PATTERN, None)
    c = cv2.cornerSubPix(g,c,(11,11),(-1,-1),(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER,30,0.001))
    _,rv,tv = cv2.solvePnP(objp,c,mtx,dist)
    R,_ = cv2.Rodrigues(rv); Ttc.append(build(R,tv))

def eval_alpha(alpha):
    Tg = [fk(th, alpha) for th in thetas]
    Rg = [t[:3,:3] for t in Tg]; tg = [t[:3,3:4] for t in Tg]
    Rt = [t[:3,:3] for t in Ttc]; tt = [t[:3,3:4] for t in Ttc]
    Rc, tc = cv2.calibrateHandEye(Rg, tg, Rt, tt, method=cv2.CALIB_HAND_EYE_TSAI)
    X = build(Rc, tc)
    poses = [Tg[i] @ X @ Ttc[i] for i in range(len(Tg))]
    pos = np.array([p[:3,3] for p in poses])
    Rref = poses[0][:3,:3]
    ang = np.rad2deg([Rotation.from_matrix(Rref.T@p[:3,:3]).magnitude() for p in poses])
    return np.std(pos,axis=0), np.mean(ang), X

vals = [0.0, 90.0, -90.0, 180.0]
results = []
for combo in product(vals, repeat=5):
    alpha = list(combo)
    std, mean_ang, X = eval_alpha(alpha)
    score = std.sum() + mean_ang  # 综合评分（越小越好）
    results.append((score, std, mean_ang, alpha, X))

results.sort(key=lambda r: r[0])
print("top 10 alpha 组合（综合评分 = 位置std求和 + 平均姿态误差）：")
for score, std, mean_ang, alpha, X in results[:10]:
    print(f"\n  alpha={alpha}  score={score:.1f}")
    print(f"    pos std(mm)={np.round(std,1)}  mean rot={mean_ang:.2f}deg")
    print(f"    X t = {X[:3,3].round(1)}")

print("\n当前表: alpha=[0, -90, -180, -90, 0]")
