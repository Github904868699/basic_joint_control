# basic_joint_control
这个程序是一个基于 ROS 的多关节电机控制节点，主要用于6轴机械臂的关节控制，具备超慢软启动和每个关节独立 KP/KD 参数设置的功能。其主要特点和流程如下：

参数读取

支持通过 ROS 参数传入每个关节的 KP（比例增益）和 KD（微分增益）数组（~kp_defaults 和 ~kd_defaults），也可以设置最大速度、控制频率、软启动时间等参数。
电机对象初始化

为每个关节创建一个 Motor 实例，分别保存各自的目标位置、速度、KP/KD 等参数。
订阅指令

订阅 /joint_cmds 话题，接收目标关节位置。
订阅 /vel_cmds 话题，接收目标关节速度。
订阅 /motor_enable 话题，控制电机使能/失能。
软启动与控制循环

使能时，所有关节进入软启动阶段，KP/KD 从0逐步爬升到设定值，防止机械冲击。
控制循环中，关节目标位置以最大速度 MAX_SPEED 平滑逼近目标，避免突变。
每个周期将关节的控制指令通过 CAN 总线发送出去。
CAN 帧打包

每个电机的控制指令会被打包成 CAN 帧，通过 /can1_tx 话题发布。
用途：
适用于需要高安全性、柔顺启动和精细参数调节的多关节机械臂或机器人关节控制场景。

主要优点：

每轴独立 KP/KD，适应不同负载和刚度需求
软启动，保护机械结构
参数灵活，易于集成和调试

sudo slcand -o -c -s8 /dev/ttyACM1 can1

sudo ifconfig can1 up

sudo ifconfig can1 txqueuelen 1000 

roscore

roslaunch armcontrol_demo_pkg can1_bridge.launch

rosrun armcontrol_demo_pkg basic_joint_control.py

rostopic pub -r 20 /joint_cmds sensor_msgs/JointState "{name: ['j1','j2','j3','j4','j5','j6'], position: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]}"
