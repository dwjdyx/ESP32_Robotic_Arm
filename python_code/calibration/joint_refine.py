import cv2
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
from homogeneous_transform import forward_kinematics

BASE = Path(r"D:\ESP32-S3\2-esp_project\6-5DOF_arm\python_code\detection")
IMG_DIR = Path(r"D:\ESP32-S3\2-esp_project\6-5DOF_arm\python_code\calibration\handeyes_calib_images")
ALPHA_FILE = Path(r"D:\ESP32-S3\2-esp_project\6-5DOF_arm\python_code\calibration\angles.txt")
CAMERA_FILE = BASE / "camera_calib_params.npz"
PATTERN = (9, 6); SQUARE = 27.0

def build(R, t):
    T = np.eye(4); T[:3,:3]=R; T[:3,3]=np.asarray(t).reshape(3); return T

with np.load(CAMERA_FILE) as d:
    mtx = np.asarray(d["mtx"], float); dist = np.asarray(d["dist"], float)

alpha = np.loadtxt(ALPHA_FILE, delimiter=",", ndmin=2)
img_paths = sorted(p for p in IMG_DIR.iterdir() if p.suffix.lower() in {".jpg",".png",".jpeg"})
objp = np.zeros((54,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)*SQUARE

# target poses in camera (fixed, from solvePnP)
Ttc = []
for ip in img_paths:
    g = cv2.cvtColor(cv2.imread(str(ip)), cv2.COLOR_BGR2GRAY)
    ok,c = cv2.findChessboardCorners(g, PATTERN, None)
    c = cv2.cornerSubPix(g,c,(11,11),(-1,-1),(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER,30,0.001))
    _,rv,tv = cv2.solvePnP(objp,c,mtx,dist)
    R,_ = cv2.Rodrigues(rv); Ttc.append(build(R,tv))

def theta_of(alpha_i, c):
    return np.array([c[0]-alpha_i[0], alpha_i[1]-c[1], c[2]-alpha_i[2],
                     alpha_i[3]-c[3], alpha_i[4]-c[4]])

def fk_pose(alpha_i, c):
    _, R, p = forward_kinematics(theta_of(alpha_i, c))
    return build(R, p)

# variables: c(5) + X rot(3) + X trans(3) = 11
def unpack(v):
    c = v[:5]
    X = build(Rotation.from_rotvec(v[5:8]).as_matrix(), v[8:11])
    return c, X

def residuals(v):
    c, X = unpack(v)
    poses = [fk_pose(a, c) @ X @ Ttc[i] for i, a in enumerate(alpha)]
    pos = np.array([p[:3,3] for p in poses])
    pos_mean = pos.mean(axis=0)
    Rref = poses[0][:3,:3]
    res = []
    for i, p in enumerate(poses):
        res += (p[:3,3] - pos_mean).tolist()
        res += Rotation.from_matrix(Rref.T @ p[:3,:3]).as_rotvec().tolist()
    return np.array(res)

# initial: current offsets + TSAI X
c0 = np.array([135.0, 231.0, 45.0, 225.0, 225.0])
R0 = Rotation.from_matrix(np.array([[0.49588164,0.84336782,0.20695925],
     [-0.86634235,0.49681467,0.05124563],[-0.05960147,-0.20470933,0.97700653]]))
t0 = np.array([-29.74135362,-15.97293074,-82.20155432])
v0 = np.concatenate([c0, R0.as_rotvec(), t0])

print("initial spread (mm, per-axis std):")
c,X = unpack(v0); poses=[fk_pose(a,c)@X@Ttc[i] for i,a in enumerate(alpha)]
print(np.std(np.array([p[:3,3] for p in poses]),axis=0).round(2))

sol = least_squares(residuals, v0, method="lm", max_nfev=20000, xtol=1e-12, ftol=1e-12)

c_opt, X_opt = unpack(sol.x)
poses=[fk_pose(a,c_opt)@X_opt@Ttc[i] for i,a in enumerate(alpha)]
pos=np.array([p[:3,3] for p in poses])
print("\noptimized spread (mm, per-axis std):")
print(np.std(pos,axis=0).round(2))

# rotation spread
Rref = poses[0][:3,:3]
ang = [Rotation.from_matrix(Rref.T@p[:3,:3]).magnitude() for p in poses]
print("optimized rotation spread (deg): mean=%.2f max=%.2f"%(np.rad2deg(np.mean(ang)), np.rad2deg(np.max(ang))))

print("\noptimized zero-offsets c (current -> optimized):")
for i,(a,b) in enumerate(zip(c0,c_opt)):
    print(f"  c{i+1}: {a:8.3f} -> {b:8.3f}  (delta {b-a:+.3f})")

print("\noptimized X (cam2gripper):")
print("R =\n", X_opt[:3,:3].round(5))
print("t =", X_opt[:3,3].round(3))
