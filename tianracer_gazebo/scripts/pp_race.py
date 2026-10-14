#!/usr/bin/env python3
import rospy, csv, math, os, rospkg
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# ===== 可调参数 =====
rospy.init_node('pp_race')   # 必须在 get_param 之前初始化, 否则命令行参数不生效
_world=os.getenv('TIANRACER_WORLD','tianracer_racetrack')
_pkg=rospkg.RosPack().get_path('tianracer_gazebo')
DEF_REF=os.path.join(_pkg,'scripts','waypoint_race',_world+'_centerline.csv')
REF=rospy.get_param('~ref',DEF_REF)
VMAX   = rospy.get_param('~vmax', 6.0)    # 最高速 m/s
ALAT   = rospy.get_param('~alat', 8.0)    # 横向加速度限制 m/s^2
ACC    = rospy.get_param('~acc', 6.0)     # 最大加速
DEC    = rospy.get_param('~dec', 9.0)     # 最大减速
K_LD   = rospy.get_param('~k_ld', 0.22)   # 前视距离随速系数
LD0    = rospy.get_param('~ld0', 0.45)    # 最小前视
WHEELBASE = rospy.get_param('~wheelbase', 0.26)  # 轴距, angular.z 直接是舵机角
LAPS   = rospy.get_param('~laps', 5)
RATE   = 40
# ============================

ref=np.array([[float(a),float(b)] for a,b in csv.reader(open(REF))])
N=len(ref)
def wrap(i): return i%N
# 预计算曲率(三点外接圆)
kappa=np.zeros(N)
for i in range(N):
    p0=ref[wrap(i-1)];p1=ref[i];p2=ref[wrap(i+1)]
    a=np.linalg.norm(p1-p0);b=np.linalg.norm(p2-p1);c=np.linalg.norm(p2-p0)
    area=abs((p1[0]-p0[0])*(p2[1]-p0[1])-(p1[1]-p0[1])*(p2[0]-p0[0]))/2
    if area>1e-6:
        R_=a*b*c/(4*area); kappa[i]=1.0/R_
# 平滑曲率(前后各2点最大,保守)
ks=np.zeros(N)
for i in range(N):
    ks[i]=max(kappa[wrap(j)] for j in range(i-2,i+3))
kappa=ks

st={'x':0.,'y':0.,'yaw':0.,'idx':0,'have':False}
pub=rospy.Publisher('/tianracer/cmd_vel',Twist,queue_size=1)

def odom_cb(m):
    st['x']=m.pose.pose.position.x; st['y']=m.pose.pose.position.y
    q=m.pose.pose.orientation
    st['yaw']=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    st['have']=True
rospy.Subscriber('/tianracer/odom',Odometry,odom_cb)

def advance(i,dist):
    d=0.
    while d<dist:
        d+=np.linalg.norm(ref[wrap(i+1)]-ref[wrap(i)]);i+=1
    return wrap(i)

# 圈计时
lap_times=[];last_cross=None;crossed=True;v=0.
r=rospy.Rate(RATE);dt=1.0/RATE
rospy.loginfo('pp_race ready, waiting odom...')
while not st['have'] and not rospy.is_shutdown(): r.sleep()
d=np.hypot(ref[:,0]-st['x'],ref[:,1]-st['y']);st['idx']=int(np.argmin(d))
t0=rospy.get_time()

while not rospy.is_shutdown() and len(lap_times)<LAPS:
    x,y,yaw=st['x'],st['y'],st['yaw'];idx=st['idx']
    # 更新最近点(局部搜索)
    best=idx;bd=1e9
    for j in range(idx-6,idx+10):
        k=wrap(j);dd=np.hypot(ref[k,0]-x,ref[k,1]-y)
        if dd<bd:bd=dd;best=k
    idx=best;st['idx']=idx
    # 转向目标点: 前视随速度, 并按曲率自适应(直道增大稳方向, 发卡略减贴线)
    kf=min(kappa[idx],3.0)/3.0
    Ld=(K_LD*v+LD0)*(1.30-0.15*kf)
    ti=advance(idx,Ld)
    tx,ty=ref[ti]
    dxw,dyw=tx-x,ty-y
    bx=dxw*math.cos(yaw)+dyw*math.sin(yaw)
    by=-dxw*math.sin(yaw)+dyw*math.cos(yaw)
    alpha=math.atan2(by,bx)
    kappapp=2*math.sin(alpha)/Ld
    # 速度: 前方制动距离内最大曲率
    brake=v*v/(2*DEC)
    scan=max(1.5,brake+1.2)
    j=idx;kk=0.;dd=0.
    while dd<scan:
        kk=max(kk,kappa[wrap(j)]);dd+=np.linalg.norm(ref[wrap(j+1)]-ref[wrap(j)]);j+=1
    vcurve=math.sqrt(ALAT/kk) if kk>1e-3 else VMAX
    vt=min(VMAX,vcurve)
    dv=vt-v;dv=max(-DEC*dt,min(ACC*dt,dv));v+=dv
    steer=math.atan(WHEELBASE*kappapp)
    m=Twist();m.linear.x=v;m.angular.z=steer;pub.publish(m)
    # 过起点(0,0) y上穿
    if abs(x)<0.6 and y>0.05:
        if not crossed:
            now=rospy.get_time()
            if last_cross is not None:
                lap_times.append(now-last_cross);rospy.loginfo('LAP %d: %.2f s',len(lap_times),now-last_cross)
            last_cross=now;crossed=True
    if y< -0.1: crossed=False
    r.sleep()

for _ in range(10):
    z=Twist();pub.publish(z);rospy.sleep(0.03)
rospy.loginfo('PP DONE laps=%d times=%s total_from_t0=%.2f',len(lap_times),[round(a,2) for a in lap_times],rospy.get_time()-t0)
