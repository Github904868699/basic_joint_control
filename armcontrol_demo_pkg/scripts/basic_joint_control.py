#!/usr/bin/env python3
"""
multi_motor_control.py  — 6‑轴超慢软启动 + 每轴独立 KP/KD
=========================================================
电机 2、3 由于负载较大，需要更高的刚度。
本版本允许 **每个关节独立 KP/KD**，通过私有参数传入：

```yaml
~kp_defaults: [10, 60, 30, 10, 10, 10]   # len==6
~kd_defaults: [0.5, 1.0, 1.0, 0.5, 0.5, 0.5]
```
如未指定，回退到标量 `~kp_default` / `~kd_default`。
其余接口与之前完全兼容。
"""

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from can_msgs.msg import Frame
from typing import List

# ---------- 协议常量 ----------
KP_MAX, KD_MAX = 500, 5

class Motor:
    P_MIN, P_MAX = -12.5, 12.5
    V_MIN, V_MAX = -30.0, 30.0
    T_MIN, T_MAX = -10.0, 10.0

    def __init__(self, motor_id:int, kp_def:float, kd_def:float):
        self.id = motor_id
        self.kp_def = kp_def
        self.kd_def = kd_def
        self.desired_pos = 0.0
        self.pos_target  = 0.0
        self.vel_target  = 0.0
        self.kp = 0.0
        self.kd = 0.0
        self.t_target = 0.0
        self.soft_cnt = 0  # 软启动计数器

    # ---------- 打包 ----------
    @staticmethod
    def f2u(x, mn, mx, bits):
        return int((x-mn)*( (1<<bits)-1)/(mx-mn)+0.5)

    def frame(self):
        p  = self.f2u(self.pos_target, self.P_MIN, self.P_MAX,16)
        v  = self.f2u(self.vel_target, self.V_MIN, self.V_MAX,12)
        kp = self.f2u(self.kp,0,KP_MAX,12)
        kd = self.f2u(self.kd,0,KD_MAX,12)
        t  = self.f2u(self.t_target, self.T_MIN, self.T_MAX,12)
        d=[0]*8
        d[0]=p>>8;            d[1]=p&0xFF
        d[2]=v>>4;            d[3]=((v&0xF)<<4)|(kp>>8)
        d[4]=kp&0xFF;         d[5]=kd>>4
        d[6]=((kd&0xF)<<4)|(t>>8); d[7]=t&0xFF
        return Frame(id=self.id, dlc=8, is_extended=False, data=d)

    def enable_frame(self):
        return Frame(id=self.id, dlc=8, is_extended=False, data=[0xFF]*7+[0xFC])

    def disable_frame(self):
        return Frame(id=self.id, dlc=8, is_extended=False, data=[0xFF]*7+[0xFD])

# ---------- 主节点 ----------

def read_list_param(name:str, default:List[float]):
    lst = rospy.get_param(name, default)
    if len(lst)!=6:
        rospy.logwarn("[%s] param %s length !=6, using default", rospy.get_name(), name)
        lst = default
    return lst

def main():
    rospy.init_node('multi_motor_control')

    motor_ids   = rospy.get_param('~motor_ids', [1,2,3,4,5,6])
    kp_defaults = read_list_param('~kp_defaults', [10, 110, 30, 10, 10, 10])
    kd_defaults = read_list_param('~kd_defaults', [0.5, 1.6, 1.0, 0.5, 0.5, 0.5])

    MAX_SPEED   = float(rospy.get_param('~max_speed',0.))  # rad/s 超慢默认
    RATE_HZ     = int(rospy.get_param('~rate',100))
    HOLD_T      = float(rospy.get_param('~soft_hold_time',0.5))
    RAMP_T      = float(rospy.get_param('~soft_ramp_time',1.5))

    HOLD_N = int(HOLD_T*RATE_HZ)
    RAMP_N = int(RAMP_T*RATE_HZ)

    motors=[Motor(mid,kp_defaults[i],kd_defaults[i]) for i,mid in enumerate(motor_ids)]
    can_pub = rospy.Publisher('/can1_tx', Frame, queue_size=100)

    # ----- 订阅 -----
    def pos_cb(msg:JointState):
        if len(msg.position)>=6:
            for i,m in enumerate(motors):
                m.desired_pos = msg.position[i]
    rospy.Subscriber('/joint_cmds', JointState, pos_cb)

    def vel_cb(msg:JointState):
        if len(msg.position)>=6:
            for i,m in enumerate(motors):
                m.vel_target = msg.position[i]
    rospy.Subscriber('/vel_cmds', JointState, vel_cb)

    state_enabled=False
    def en_cb(msg:Bool):
        nonlocal state_enabled
        if msg.data and not state_enabled:
            state_enabled=True
            rospy.loginfo("[multi_motor] ENABLE")
            for m in motors:
                m.pos_target=m.desired_pos
                m.kp=m.kd=0.0
                m.soft_cnt=HOLD_N+RAMP_N
                can_pub.publish(m.enable_frame())
        elif (not msg.data) and state_enabled:
            state_enabled=False
            rospy.loginfo("[multi_motor] DISABLE")
            for m in motors: can_pub.publish(m.disable_frame())
    rospy.Subscriber('/motor_enable', Bool, en_cb)

    rate=rospy.Rate(RATE_HZ)
    step=MAX_SPEED/RATE_HZ
    while not rospy.is_shutdown():
        if state_enabled:
            for m in motors:
                # 软启
                if m.soft_cnt>0:
                    m.soft_cnt-=1
                    if m.soft_cnt>=RAMP_N:
                        m.kp=m.kd=0.0
                    else:
                        ramp_idx=RAMP_N-m.soft_cnt
                        fac=ramp_idx/float(RAMP_N)
                        m.kp=m.kp_def*fac
                        m.kd=m.kd_def*fac
                else:
                    m.kp=m.kp_def; m.kd=m.kd_def
                # 爬坡
                d=m.desired_pos-m.pos_target
                if d>step: m.pos_target+=step
                elif d<-step: m.pos_target-=step
                else: m.pos_target=m.desired_pos
                can_pub.publish(m.frame())
        rate.sleep()

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

