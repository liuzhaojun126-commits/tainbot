#!/usr/bin/env python3
"""
Offline racing node: follows a precomputed raceline CSV (x,y,v_target,kappa).
- Pure pursuit steering + curvature feedforward (delta_ff = atan(L*kappa))
- Speed directly tracks v_target from CSV (rate-limited by ACC/DEC)
- Same lap-counting logic as pp_race.py

CSV format (comma-separated, optional # comments):
  x,y,v_target,kappa
"""
import rospy, csv, math, os, rospkg
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# ===== 可调参数 (rosparam) =====
rospy.init_node('pp_race_offline')
_world = os.getenv('TIANRACER_WORLD', 'tianracer_racetrack')
_pkg = rospkg.RosPack().get_path('tianracer_gazebo')
DEF_REF = os.path.join(_pkg, 'scripts', 'waypoint_race', _world + '_raceline.csv')

# --- 核心参数 ---
REF       = rospy.get_param('~ref', DEF_REF)
VMAX_OVR  = rospy.get_param('~vmax_override', 0.0)  # >0 时覆盖 CSV 中的 vmax
ACC       = rospy.get_param('~acc', 6.0)
DEC       = rospy.get_param('~dec', 9.0)

# --- 转向参数 ---
K_LD      = rospy.get_param('~k_ld', 0.22)      # 前视距离随速系数
LD0       = rospy.get_param('~ld0', 0.45)         # 最小前视距离
PP_GAIN   = rospy.get_param('~pp_gain', 1.0)     # 纯追踪增益
FF_GAIN   = rospy.get_param('~ff_gain', 1.0)     # 曲率前馈增益
WHEELBASE = rospy.get_param('~wheelbase', 0.26)   # 轴距
DELTA_LIMIT = rospy.get_param('~delta_limit', 0.58)  # 舵角上限 rad

# --- 速度取点参数 ---
SPEED_LOOKAHEAD = rospy.get_param('~speed_lookahead', 0.5)  # 速度前视距离 m

# --- 其他 ---
LAPS      = rospy.get_param('~laps', 5)
RATE      = 40

# ===== 读取离线赛车线 CSV =====
ref_x, ref_y, ref_v, ref_kappa = [], [], [], []
with open(REF) as f:
    for row in csv.reader(f):
        if not row or row[0].startswith('#'):
            continue
        vals = [float(v) for v in row]
        ref_x.append(vals[0])
        ref_y.append(vals[1])
        ref_v.append(vals[2])
        if len(vals) >= 4:
            ref_kappa.append(vals[3])
        else:
            ref_kappa.append(0.0)

ref = np.column_stack([ref_x, ref_y])
ref_v = np.array(ref_v)
ref_kappa = np.array(ref_kappa)
N = len(ref)

# Apply VMAX override
if VMAX_OVR > 0:
    ref_v = np.minimum(ref_v, VMAX_OVR)

def wrap(i):
    return i % N

rospy.loginfo(f'Loaded raceline: {N} points from {REF}')
rospy.loginfo(f'  v_target range: [{ref_v.min():.2f}, {ref_v.max():.2f}] m/s')
rospy.loginfo(f'  kappa range: [{ref_kappa.min():.3f}, {ref_kappa.max():.3f}] 1/m')
rospy.loginfo(f'  pp_gain={PP_GAIN}, ff_gain={FF_GAIN}, speed_lookahead={SPEED_LOOKAHEAD}m')

# ===== 状态 =====
st = {'x': 0., 'y': 0., 'yaw': 0., 'idx': 0, 'have': False}
pub = rospy.Publisher('/tianracer/cmd_vel', Twist, queue_size=1)

def odom_cb(m):
    st['x'] = m.pose.pose.position.x
    st['y'] = m.pose.pose.position.y
    q = m.pose.pose.orientation
    st['yaw'] = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
    st['have'] = True

rospy.Subscriber('/tianracer/odom', Odometry, odom_cb)

def advance(i, dist):
    d = 0.
    while d < dist:
        d += np.linalg.norm(ref[wrap(i+1)] - ref[wrap(i)])
        i += 1
    return wrap(i)

# ===== 主循环 =====
lap_times = []
last_cross = None
crossed = True
v = 0.
r = rospy.Rate(RATE)
dt = 1.0 / RATE

rospy.loginfo('pp_race_offline ready, waiting odom...')
while not st['have'] and not rospy.is_shutdown():
    r.sleep()

d = np.hypot(ref[:,0] - st['x'], ref[:,1] - st['y'])
st['idx'] = int(np.argmin(d))
t0 = rospy.get_time()

while not rospy.is_shutdown() and len(lap_times) < LAPS:
    x, y, yaw = st['x'], st['y'], st['yaw']
    idx = st['idx']

    # Update closest point (local search)
    best = idx
    bd = 1e9
    for j in range(idx - 6, idx + 10):
        k = wrap(j)
        dd = np.hypot(ref[k,0] - x, ref[k,1] - y)
        if dd < bd:
            bd = dd
            best = k
    idx = best
    st['idx'] = idx

    # Lookahead point for steering (pure pursuit target)
    kf = min(abs(ref_kappa[idx]), 3.0) / 3.0
    Ld = (K_LD * v + LD0) * (1.30 - 0.15 * kf)
    ti_steer = advance(idx, Ld)
    tx, ty = ref[ti_steer]

    # Pure pursuit steering angle
    dxw, dyw = tx - x, ty - y
    bx = dxw * math.cos(yaw) + dyw * math.sin(yaw)
    by = -dxw * math.sin(yaw) + dyw * math.cos(yaw)
    alpha = math.atan2(by, bx)
    kappa_pp = 2 * math.sin(alpha) / Ld
    steer_pp = math.atan(WHEELBASE * kappa_pp)

    # Curvature feedforward: delta_ff = atan(L * kappa)
    kappa_ff = ref_kappa[idx]
    steer_ff = math.atan(WHEELBASE * kappa_ff)

    # Total steering = PP_GAIN * PP + FF_GAIN * feedforward
    steer = PP_GAIN * steer_pp + FF_GAIN * steer_ff
    steer = max(-DELTA_LIMIT, min(DELTA_LIMIT, steer))

    # Speed target: look ahead by SPEED_LOOKAHEAD distance
    ti_speed = advance(idx, SPEED_LOOKAHEAD)
    v_target = ref_v[ti_speed]
    # Rate limit
    dv = v_target - v
    dv = max(-DEC * dt, min(ACC * dt, dv))
    v += dv

    m = Twist()
    m.linear.x = v
    m.angular.z = steer
    pub.publish(m)

    # Lap counting (crossing start line near (0,0) going +y)
    if abs(x) < 0.6 and y > 0.05:
        if not crossed:
            now = rospy.get_time()
            if last_cross is not None:
                lap_times.append(now - last_cross)
                rospy.loginfo('LAP %d: %.2f s', len(lap_times), now - last_cross)
            last_cross = now
            crossed = True
    if y < -0.1:
        crossed = False

    r.sleep()

# Stop the car
for _ in range(10):
    z = Twist()
    pub.publish(z)
    rospy.sleep(0.03)

rospy.loginfo('PP_OFFLINE DONE laps=%d times=%s total=%.2f',
              len(lap_times), [round(a, 2) for a in lap_times],
              rospy.get_time() - t0)
