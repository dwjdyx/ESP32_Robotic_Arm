import cv2
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares

BASE = Path(r"D:\ESP32-S3\2-esp_project\6-5DOF_arm\python_code\detection")
IMG_DIR = Path(r"D:\ESP32-S3\2-esp_project\6-5DOF_arm\python_code\calibration\handeyes_images")
ANGLE_FILE = BASE / "joints.txt"
CAMERA_FILE = BASE / "camera_calib_params.npz"
PATTERN = (9, 6); SQUARE = 27.0

# 固定参数：连杆长度 a / 偏距 d（用户已量测），alpha 作为待优化变量
A = [0.0, 10.25, 105.0, 0.0, 0.0]
D = [0.0, 21.0, 21.0, 62.0, 107.0]
ALPHA0 = np.array([0.0, -90.0, -180.0, -90.0, 0.0])

def build(R, t):
    T = np.eye(4); T[:3,:3] = R; T[:3,3] = np.asarray(t).reshape(3); return T

def mdh(a, alpha_deg, d, theta_deg):
    alpha = np.deg2rad(alpha_deg); theta = np.deg2rad(theta_deg)
    ca, sa = np.cos(alpha), np.sin(alpha)
    ct, st = np.cos(theta), np.sin(theta)
    return np.array([[ct, -st, 0, a],
                     [st*ca, ct*ca, -sa, -d*sa],
                     [st*sa, ct*sa, ca, d*ca],
                     [0,0,0,1]], dtype=float)

def fk(theta, alpha):
    T = np.eye(4)
    for i in range(5):
        T = T @ mdh(A[i], alpha[i], D[i], theta[i])
    return T

# 相机 + 棋盘格位姿（固定）
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

W_ROT = 500.0  # 旋转残差权重（rad -> 近似 mm，板距约 900mm）

def unpack(v):
    alpha = v[:5]
    X = build(Rotation.from_rotvec(v[5:8]).as_matrix(), v[8:11])
    return alpha, X

def residuals(v):
    alpha, X = unpack(v)
    poses = [fk(th, alpha) @ X @ Ttc[i] for i, th in enumerate(thetas)]
    pos = np.array([p[:3,3] for p in poses])
    pos_mean = pos.mean(axis=0)
    Rref = poses[0][:3,:3]
    res = []
    for p in poses:
        res += (p[:3,3] - pos_mean).tolist()
        res += (Rotation.from_matrix(Rref.T @ p[:3,:3]).as_rotvec() * W_ROT).tolist()
    return np.array(res)

def spread(v):
    alpha, X = unpack(v)
    poses = [fk(th, alpha) @ X @ Ttc[i] for i, th in enumerate(thetas)]
    pos = np.array([p[:3,3] for p in poses])
    Rref = poses[0][:3,:3]
    ang = np.rad2deg([Rotation.from_matrix(Rref.T@p[:3,:3]).magnitude() for p in poses])
    return np.std(pos,axis=0), np.mean(ang), np.max(ang)

# 初始 X 用当前 TSAI 结果
R0 = Rotation.from_matrix(np.array([[0.77110603,0.63394498,0.05923892],
    [-0.63127951,0.74909495,0.20085552],[0.08295578,-0.19227722,0.97782811]]))
t0 = np.array([-4.86332729,-66.60650077,-96.34359067])
v0 = np.concatenate([ALPHA0, R0.as_rotvec(), t0])

print("initial spread (mm std, mean rot deg, max rot deg):")
print(spread(v0))

sol = least_squares(residuals, v0, method="lm", max_nfev=40000, xtol=1e-12, ftol=1e-12)

print("\noptimized spread:")
print(spread(sol.x))

print("\nalpha (current -> optimized):")
for i,(a,b) in enumerate(zip(ALPHA0, sol.x[:5])):
    print(f"  alpha{i+1}: {a:8.2f} -> {b:9.2f}   (delta {b-a:+.2f})")

alpha_opt, X_opt = unpack(sol.x)
print("\noptimized X (cam2gripper):")
print("R =\n", X_opt[:3,:3].round(5))
print("t =", X_opt[:3,3].round(3))
