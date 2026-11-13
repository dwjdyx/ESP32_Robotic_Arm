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
    T = np.eye(4); T[:3,:3]=R; T[:3,3]=np.asarray(t).reshape(3); return T

def ang(R): return np.rad2deg(Rotation.from_matrix(R).magnitude())

with np.load(CAMERA_FILE) as d:
    mtx = np.asarray(d["mtx"], float); dist = np.asarray(d["dist"], float)

angles = np.loadtxt(ANGLE_FILE, delimiter=",", ndmin=2)
img_paths = sorted(p for p in IMG_DIR.iterdir() if p.suffix.lower() in {".jpg",".png",".jpeg"})
objp = np.zeros((54,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)*SQUARE

Tg, Tt, names = [], [], []
for ip, th in zip(img_paths, angles):
    g = cv2.cvtColor(cv2.imread(str(ip)), cv2.COLOR_BGR2GRAY)
    ok,c = cv2.findChessboardCorners(g, PATTERN, None)
    if not ok: continue
    c = cv2.cornerSubPix(g,c,(11,11),(-1,-1),(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER,30,0.001))
    _,rv,tv = cv2.solvePnP(objp,c,mtx,dist)
    R,_ = cv2.Rodrigues(rv); Tt.append(build(R,tv))
    _,Rg,p = forward_kinematics(th); Tg.append(build(Rg,p)); names.append(ip.name)

n = len(Tg)
score = np.zeros(n)
for i in range(n):
    s = []
    for j in range(n):
        if i==j: continue
        A = np.linalg.inv(Tg[i]) @ Tg[j]
        B = Tt[i] @ np.linalg.inv(Tt[j])
        s.append(abs(ang(A[:3,:3]) - ang(B[:3,:3])))
    score[i] = np.mean(s)

print("per-image mean rotation mismatch (deg), sorted:")
for k in np.argsort(-score):
    print(f"  {names[k]}  {score[k]:5.2f}  (theta={angles[k]})")

print(f"\noverall mean mismatch = {score.mean():.2f} deg")

# try removing the worst k images and see how hand-eye consistency improves
def consistency(mask):
    Tg2=[Tg[i] for i in range(n) if mask[i]]
    Tt2=[Tt[i] for i in range(n) if mask[i]]
    Rg=[t[:3,:3] for t in Tg2]; tg=[t[:3,3:4] for t in Tg2]
    Rt=[t[:3,:3] for t in Tt2]; tt=[t[:3,3:4] for t in Tt2]
    Rc,tc = cv2.calibrateHandEye(Rg,tg,Rt,tt,method=cv2.CALIB_HAND_EYE_TSAI)
    X=build(Rc,tc)
    poses=[Tg2[i]@X@Tt2[i] for i in range(len(Tg2))]
    pos=np.array([p[:3,3] for p in poses])
    return np.std(pos,axis=0)

mask = np.ones(n, bool)
print("\nboard position std (mm) as we drop worst images:")
print(f"  all {n}: std={consistency(mask).round(1)}")
for r in range(1, 8):
    worst = int(np.argsort(-score)[:r].tolist()[-1])
    mask2 = mask.copy(); mask2[np.argsort(-score)[:r]] = False
    print(f"  drop {r} (worst={names[worst]}): std={consistency(mask2).round(1)}")
