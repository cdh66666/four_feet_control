
% % fcn 函数功能描述：
% % 该函数根据输入的状态（status）、足端受力（force）以及足端位置（pos）信息，进行一系列计算，
% % 包括判断足端触地状态、根据状态转移规则更新整体状态、计算运动相关物理量（速度、位置误差等），
% % 最终通过 PD 控制计算四条腿对应的力（以矩阵 F 返回）。
% 
% % 输入参数：
% % - status: 包含机器人的线加速度（ax、ay、az）和角速度（wx、wy、wz）等状态信息的向量，维度为 6。
% % - status_d: 包含期望线速度（vxd、vyd、vzd）和期望角速度（rolld、pitchd、yawd）等期望状态信息的向量，维度为 6。
% % - force: 包含各个足端受力信息的向量，假设其元素顺序与后续处理逻辑相对应，长度需符合要求。
% % - pos: 包含各个足端位置信息的向量，假设其元素个数是 12，顺序与后续处理逻辑相对应，可重塑为合适矩阵形式用于计算。
% 
% % 输出参数：
% % - F: 四条腿对应的控制力矩阵，维度为 (4, 3)，每行对应一条腿的控制力（三维向量）。
% % 腿部顺序为：左前 - 左后 - 右前 - 右后，所有传入传出的参数都是按照这个顺序来的
% % 声明持久变量，用于在多次函数调用间保持状态

function F = VMC_Trot_Control(x)

global T beta Tf Ts M g simulink_step force_threshold L W H l1 l2 l3 floor_x floor_y floor_h z_init h_init damping stiff friction_v friction_p VMC_PID_P VMC_PID_D hd Ktd K_z K_vx K_vy K_vz K_wx K_wy K_wz;

%传入参数分割
status = x(1:6);
status_d = x(7:12);
force = x(13:16);
pos = x(17:28);

 

% % 定义持久变量
% persistent t state_vars error_prev_matrix S x_init_swing y_init_swing z_init_swing;
% if isempty(t)
%     % t 为计时时间
%     t = 0;
%     % S 为状态机状态
%     S = 0;
%     % 初始化状态变量矩阵，包含速度和角度信息，维度为 (6, 1)，分别对应 [vx; vy; vz; roll; pitch; yaw]
%     state_vars = zeros(6, 1);
%     % 初始化上一次的误差值矩阵，初始化为全零矩阵，维度为 (4, 3)，对应四条腿，每个腿的误差是三维向量
%     error_prev_matrix = zeros(4, 3);
%     % 初始化每条摆动腿的起始位置坐标（初始化为对应维度的全零矩阵，后续在 t == 0 时赋值）
%     p_init_swing = zeros(4, 3);
% end



[fz, onfloor_matrix, pos_matrix] = process_foot_force_and_position(force, pos, force_threshold)


F1 = [1  1  1];
F2 = [2 2 2];
F3 = [3  3 3];
F4 = [4  4 4];
F = [F1;F2;F3;F4];

 
end

% 处理足端受力及位置信息的子函数
function [fz, onfloor_matrix, pos_matrix] = process_foot_force_and_position(force, pos, force_threshold)
    % 将足端受力赋值并转换为二维矩阵形式（按列组合），方便后续矩阵运算
    fz = reshape(force([1, 2,3,4]), [], 1);

    % 通过矩阵比较判断足端受力是否大于阈值，如大于则表示触地，转换为数值矩阵（0 或 1）表示触地状态
    onfloor_matrix = double(fz > force_threshold);

    % 将足端位置赋值并重塑为合适的矩阵形式（num_legs 行 3 列，这里 num_legs = 4）
    pos_matrix = reshape(pos, 3, []).';
end