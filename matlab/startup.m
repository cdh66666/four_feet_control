clear;
clc;
 
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

