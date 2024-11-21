clear;
clc;
 
global T beta Tf Ts M g simulink_step force_threshold L W H l1 l2 l3 floor_x floor_y floor_h z_init h_init damping stiff friction_v friction_p VMC_PID_P VMC_PID_D hd Ktd K_z K_vx K_vy K_vz K_wx K_wy K_wz;
global t S state_vars error_prev_matrix p_init_swing state_d stop_time status K_roll K_pitch2 K_vx K_h K_h_dot;
% 初始化上一次的误差值矩阵，初始化为全零矩阵，维度为 (4, 3)，对应四条腿，每个腿的误差是三维向量
error_prev_matrix = zeros(4, 3);
% 初始化每条摆动腿的起始位置坐标（初始化为对应维度的全零矩阵，后续在 t == 0 时赋值）
p_init_swing = zeros(4, 3);
%三轴加速度和三轴角速度
status = zeros(6, 1);
% 初始化状态变量矩阵，包含速度和角度信息，维度为 (6, 1)，分别对应 [vx; vy; vz; roll; pitch; yaw]
state_vars = zeros(6, 1);
%期望四足运行状态,[vx; vy; vz; roll; pitch; yaw]
state_d = zeros(6, 1);
%摆动相运行时间t
t=0;
%状态机状态S
S=-1;
% 步态周期 T/s
T = 1;
% 摆动相占空比 beta
beta = 0.5;
% 摆动相周期 Tf
Tf = beta * T;
% 支撑相周期 Ts
Ts = T - Tf;
%仿真停止时间
stop_time = 3;
% 机体质量，单位：kg
M = 6,4;
% 重力加速度
g = 9.8;
% 仿真时间步长
simulink_step = 1e-3;
% 足端受力阈值
force_threshold = 10;
% 定义常量
L = 0.365;  %机体长度
W = 0.1;    %机体宽度
H =0.02;    %机体高度
l1 = 0.085; %腿部连杆1长度
l2= 0.2;%腿部连杆2长度
l3 = 0.2;%腿部连杆3长度
floor_x = 2;%地面长
floor_y = 2;%地面宽
floor_h= 0.05;%地面高
% 一开始离地高度
z_init = 0.5;
%腿部初始保持高度
h_init = 0.35;
damping = 100;%接触阻尼
stiff =10000;
friction_v = 0.5;%动摩擦系数
friction_p =0.7;%静摩擦系数
% VMC 力位混合控置 PD 参数
VMC_PID_P = 250;
VMC_PID_D = 0;
 
theta0_init = 0;
theta1_init = 45;
theta2_init = -75;


% 抬腿高度
hd = 0.1; % 单位：m
% 足端下探速度
Ktd = 0.1;
 
 
K_vx = 0; % 控制前进速度系数，需根据实际调试确定合适值
K_vy = 0; % 控制侧向速度系数，需根据实际调试确定合适值
K_vz = 0; % 控制高度阻尼系数，需根据实际调试确定合适值
K_wx = 0; % 控制横滚角速度系数，需根据实际调试确定合适值
K_wy = 0; % 控制伪俯仰角速度系数，需根据实际调试确定合适值
K_wz = 0; % 控制自转角速度系数，需根据实际调试确定合适值

K_h = 0;
K_h_dot = 0;
K_roll = 0;
K_pitch2 = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% 包含力传感器接触模型
fullPath = genpath('forces_lib');
addpath(fullPath)

% 包含stl模型
fullPath = genpath('unitree_stl');
addpath(fullPath)

% 设置要运行的 slx 文件路径
slxFileName = 'four_feet_control_VMC.slx'; 

% 直接调用 open 函数并将输出重定向到临时变量，避免阻塞
tmp = open(slxFileName);

% 加载模型
load_system(slxFileName);

% 运行模型
sim(slxFileName);

