clear;
clc;
 
global T beta Tf Ts M g simulink_step force_threshold L W H l1 l2 l3 floor_x floor_y floor_h z_init h_init damping stiff friction_v friction_p VMC_PID_P VMC_PID_D hd Ktd K_z K_vx K_vy K_vz K_wx K_wy K_wz;

% 步态周期 T/s
T = 1;
% 摆动相占空比 beta
beta = 0.5;
% 摆动相周期 Tf
Tf = beta * T;
% 支撑相周期 Ts
Ts = T - Tf;

% 机体质量，单位：kg
M = 12;
% 重力加速度
g = 9.8;
% 仿真时间步长
simulink_step = 1e-3;
% 足端受力阈值
force_threshold = 20;
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
z_init = 0.4;
%腿部初始保持高度
h_init = 0.35;
damping = 10;%接触阻尼
stiff =10000;%接触高度
friction_v = 0.3;%动摩擦系数
friction_p =0.7;%静摩擦系数
% VMC 力位混合控置 PD 参数
VMC_PID_P = 2000;
VMC_PID_D = 7500;
 
% 抬腿高度
hd = 0.1; % 单位：m
% 足端下探速度
Ktd = 0.1;
 
K_z = 1000; % 控制高度比例系数，需根据实际调试确定合适值
K_vx = 0; % 控制前进速度系数，需根据实际调试确定合适值
K_vy = 0; % 控制侧向速度系数，需根据实际调试确定合适值
K_vz = 0; % 控制高度阻尼系数，需根据实际调试确定合适值
K_wx = 0; % 控制横滚角速度系数，需根据实际调试确定合适值
K_wy = 0; % 控制伪俯仰角速度系数，需根据实际调试确定合适值
K_wz = 0; % 控制自转角速度系数，需根据实际调试确定合适值



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

