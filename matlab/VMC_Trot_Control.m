
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
global t S state_vars error_prev_matrix p_init_swing state_d stop_time status;

% %传入参数分割
status = x(1:6);
force = x(7:10);
pos = x(11:22);
 
% 提取加速度和角速度状态信息
[ax, ay, az, wx, wy, wz] = extract_status(status);
% 通过矩阵运算更新状态变量（速度和角度）
state_vars = update_state_vars(state_vars, [ax; ay; az; wx; wy; wz], simulink_step);

% 处理足端受力及位置信息 
[fz, onfloor_matrix, pos_matrix] = process_foot_force_and_position(force, pos,force_threshold);

 
% 更新计时时间 
t = t + simulink_step;
% 各足端受力总和 
total_force = sum(fz, 1);
 
% 获取状态转移规则矩阵并更新整体状态 S
[S] = update_state(S, onfloor_matrix);

% 根据状态 S 判断并计算相关变量
if S == -1
    % 四足腾空相
    [F] = handle_four_feet_airborne(pos_matrix);
else
    % 判断当前是哪组腿摆动、哪组腿支撑，并进行相应计算
      %4行3列对应四条腿及三维力向量
%       F = zeros(4,3);
    [F] = handle_leg_phases(pos_matrix);
end
  
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% 根据状态判断并处理腿部摆动相和支撑相的通用子函数
function [F] = handle_leg_phases(pos_matrix)
    global  S;
   
    % 获取当前处于摆动相和支撑相的腿的索引（根据状态转移规则确定）
    [swing_leg_indices, stance_leg_indices] = determine_leg_phases(S);

    % 分别处理摆动相和支撑相的腿
    [F_swing] = handle_swing_legs(pos_matrix,swing_leg_indices);
    [F_stance] = handle_stance_legs(pos_matrix,stance_leg_indices);
    
    F = zeros(4,3);
   
    %将摆动相腿的力按顺序赋值到F中
    F(swing_leg_indices(1), :) = F_swing(swing_leg_indices(1), :);
    F(swing_leg_indices(2), :) = F_swing(swing_leg_indices(1), :);
    %将支撑相腿的力按顺序赋值到F中
    F(stance_leg_indices(1), :) = F_stance(stance_leg_indices(1), :);
    F(stance_leg_indices(2), :) = F_stance(stance_leg_indices(1), :);

 
end


function [F_stance] = handle_stance_legs(pos_matrix,stance_leg_indices)
    global hd state_vars state_d  M g status K_wx K_wy K_wz K_roll K_pitch2 K_vx K_h K_h_dot
 
 

     % 计算机体的伪俯仰角及机体中心离地高度 
    xF = pos_matrix(stance_leg_indices(1),1);
    yF = pos_matrix(stance_leg_indices(1),2);
    zF = pos_matrix(stance_leg_indices(1),3);
    xH = pos_matrix(stance_leg_indices(2),1);
    yH = pos_matrix(stance_leg_indices(2),2);
    zH = pos_matrix(stance_leg_indices(2),3);
    pitch2 = atan2(zF - zH, xF - xH);
    h = -(zF + zH) / 2;
 
 
    % 初始化力矩阵
    F_stance = zeros(4, 3);
    %获取状态信息
    [vx, vy, vz, roll, pitch, yaw] = extract_status(state_vars);
    %提取期望机体速度和偏转角度状态信息
    [vxd, vyd, vzd, rolld, pitchd, yawd] = extract_status(state_d);
    % 提取加速度和角速度状态信息
    [ax, ay, az, wx, wy, wz] = extract_status(status);

    % 控制横滚角
    Tx = K_roll * roll + K_wx * wx;
    % 控制伪俯仰角
    Ty = K_pitch2 * pitch2 + K_wy * wy;
    % 控制机体高度
    Fz = K_h * (hd - h) + K_h_dot * vz;
    % 控制前进速度
    Fx = K_vx * (vxd - vx);
    % 控制自转角速度
    Tz = K_wz * wz;

    % 构建力和力矩向量（考虑机体重力影响，添加对应分量）
    forces_and_torques = [Fx + M * g * sin(pitch); Fz + M * g * cos(pitch); Tx; Ty; Tz; 0];

    Q = [1 0 0 1 0 0;
     0 0 1 0 0 1;
     0 -zF yF 0 -zH yH;
     zF 0 -xF zH 0 -xH;
     -yF xF 0 -yH xH 0;
     0 1 0 0 -1 0;];

    % 计算矩阵的逆，需确保矩阵可逆，这里简单使用inv函数，实际可能需更多错误处理
    Q_inv = inv(Q);

  
    % 根据作用力与反作用力关系计算各腿关机电机输出力矩（转换到各腿的力）
    F_combined = -Q_inv * forces_and_torques;

    % 将计算得到的力赋值给对应的支撑腿力矩阵
    F_stance(stance_leg_indices(1), 1) = F_combined(1,1);
    F_stance(stance_leg_indices(1), 2) = F_combined(2,1);
    F_stance(stance_leg_indices(1), 3) = F_combined(3,1);
    F_stance(stance_leg_indices(2), 1) = F_combined(4,1);
    F_stance(stance_leg_indices(2), 2) = F_combined(5,1);
    F_stance(stance_leg_indices(2), 3) = F_combined(6,1);
 
 
end


 

 
% 处理摆动相腿的子函数（通用，同时考虑传入的对应腿的数据）
function [F_swing] = handle_swing_legs(pos_matrix,swing_leg_indices)
    global error_prev_matrix  p_init_swing state_vars state_d t VMC_PID_P VMC_PID_D;
    global Ts K_vy Tf Ktd hd
    [vx, vy, vz, roll, pitch, yaw] = extract_status(state_vars);
    %提取期望机体速度和偏转角度状态信息
    [vxd, vyd, vzd, rolld, pitchd, yawd] = extract_status(state_d);

    num_swing_legs = size(pos_matrix(swing_leg_indices,:), 1);
    % 初始化期望位置矩阵
    desired_positions = pos_matrix;

    for i = 1:1:num_swing_legs
        % 仅在 t == 0 时为每条摆动腿赋初始值
        if t == 0
            p_init_swing(swing_leg_indices(i),:) = pos_matrix(swing_leg_indices(i),:);
        end
        % 使用每条腿独立的初始值
        x_init = p_init_swing(swing_leg_indices(i),1);
        y_init = p_init_swing(swing_leg_indices(i),2);
        z_init = p_init_swing(swing_leg_indices(i),3);

        % x 方向落足点距离（可根据实际情况调整每条腿的期望速度等，这里简化为相同计算方式）
        xf = vxd * Ts / 2;
        % y 方向落足点距离
        yf = vyd * Ts / 2 + K_vy * (vyd - vy);
        % 判断是否超过运行时间
        if (t > Tf)
            deta_t = t - Tf;
            phi = 2 * pi;
            Xf = xf + x_init;
            Yf = yf + y_init;
        else % 在轨迹规划时间内
            deta_t = 0;
            phi = 2 * pi * t / Tf;
            Xf = xf * (phi - sin(phi)) / (2 * pi) + x_init;
            Yf = yf * (phi - sin(phi)) / (2 * pi) + y_init;
        end

        % 考虑足端轨迹延伸下探（加入 Ktd*(deta_t) 项）
        Zf = hd * (1 - cos(phi)) / 2 + z_init - Ktd * (deta_t);

        % 填充期望位置矩阵
 
        desired_positions(swing_leg_indices(i), 1) = Xf;
        desired_positions(swing_leg_indices(i), 2) = Yf;
        desired_positions(swing_leg_indices(i), 3) = Zf;
 
    end

    % 计算位置误差矩阵，利用矩阵减法一次性计算四条腿的误差
    error_matrix = desired_positions - pos_matrix
    % 计算误差变化量矩阵，利用矩阵减法
    d_error_matrix = error_matrix - error_prev_matrix
    % 更新上一次的误差值矩阵，用于下一次计算
    error_prev_matrix = error_matrix
    % PD 控制计算力矩阵，利用矩阵乘法和加法一次性计算四条腿的力
    F_swing = VMC_PID_P * error_matrix + VMC_PID_D * d_error_matrix

end


% 根据状态确定摆动相和支撑相腿的索引的子函数
function [swing_leg_indices, stance_leg_indices] = determine_leg_phases(S)
    switch S
        case 0
            % A组摆动（左前、右后），B组着地（右前、左后）
            swing_leg_indices = [1, 3];
            stance_leg_indices = [4, 2];
        case 1
            % A组着地（左前、右后），B组摆动（右前、左后）
            swing_leg_indices = [4, 2];
            stance_leg_indices = [1, 3];
        case 2
            % A组摆动（左前、右后），B组着地（右前、左后）
            swing_leg_indices = [1, 3];
            stance_leg_indices = [4, 2];
        case 3
            % A组着地（左前、右后），B组摆动（右前、左后）
            swing_leg_indices = [4, 2];
            stance_leg_indices = [1, 3];
    end
end






% 处理四足腾空相的子函数
function [F] = handle_four_feet_airborne(pos_matrix)
    global  VMC_PID_P VMC_PID_D L W h_init error_prev_matrix
    % 四足腾空相时四条腿的初始位置矩阵，直接构建，避免逐个变量赋值
    P_INIT = [L / 2, -L / 2, -L / 2, L / 2; (W + 0.1) / 2, (W + 0.1) / 2, -(W + 0.1) / 2, -(W + 0.1) / 2; -h_init, -h_init, -h_init, -h_init].';
    % 计算位置误差矩阵，利用矩阵减法一次性计算四条腿的误差
    error_matrix = P_INIT - pos_matrix;
    % 计算误差变化量矩阵，利用矩阵减法
    d_error_matrix = error_matrix - error_prev_matrix;
    % 更新上一次的误差值矩阵，用于下一次计算
    error_prev_matrix = error_matrix;
    % PD 控制计算力矩阵，利用矩阵乘法和加法一次性计算四条腿的力
    F = VMC_PID_P * error_matrix + VMC_PID_D * d_error_matrix;
end

 

% 定义状态转移规则矩阵构建函数（增强可扩展性和代码结构清晰度）
function transition_matrix = build_transition_rules()
    transition_matrix = [
        0, 0, 0, 0, -1, -1;  % 四足腾空相，从任意状态转移到 -1（这里重复 -1 只是为了格式统一，实际判断时只要满足前面的触地状态条件即可）
        1, 1, 1, 1, -1, 0;   % 初始化并且全部触地，从 -1 状态转移到 0
        0, 1, 0, 1, 0, 2;    % 抬腿 A 组并且全部 A 组离地，从 0 状态转移到 2
        1, 1, 1, 1, 2, 1;    % A 组全部着地，从 2 状态转移到 1
        1, 0, 1, 0, 1, 3;    % 抬腿 B 组并且全部 B 组离地，从 1 状态转移到 3
        1, 1, 1, 1, 3, 0     % B 组全部着地，从 3 状态转移到 0
    ];
end


% 更新状态的子函数（包括获取状态转移规则矩阵和根据触地情况更新状态）
function [new_S]= update_state(S,onfloor_matrix)
    global t;
    transition_rules = build_transition_rules();
    new_S = S;
    % 先单独判断四足腾空相情况
    if ~any(onfloor_matrix(:))
        t =0; %摆动相时间清零
        new_S = -1;
    else
        % 遍历规则矩阵判断其他转移情况
        for i = 1:size(transition_rules, 1)
            if all(onfloor_matrix == transition_rules(i, 1:4), 'all') && S == transition_rules(i, 5)
                new_S = transition_rules(i, 6);
                if(new_S == 0 || new_S == 1 )
                    t =0;%更新状态，摆动相时间清零
                end
                break;
            end
        end
    end
end




% 更新状态变量（速度和角度）的子函数
function new_state_vars = update_state_vars(state_vars, acceleration_angular_velocity, simulink_step)
    new_state_vars = state_vars + acceleration_angular_velocity * simulink_step;
end

  

% 处理足端受力及位置信息的子函数
function [fz, onfloor_matrix, pos_matrix] = process_foot_force_and_position(force, pos,force_threshold)
    % 将足端受力赋值并转换为二维矩阵形式（按列组合），方便后续矩阵运算
    fz = reshape(force([1, 2,3,4]), [], 1);

    % 通过矩阵比较判断足端受力是否大于阈值，如大于则表示触地，转换为数值矩阵（0 或 1）表示触地状态
    onfloor_matrix = double(fz > force_threshold);

    % 将足端位置赋值并重塑为合适的矩阵形式（num_legs 行 3 列，这里 num_legs = 4）
    pos_matrix = reshape(pos, 3, []).';
end

% 提取状态信息的子函数
function [ax, ay, az, wx, wy, wz] = extract_status(status)
    ax = status(1);
    ay = status(2);
    az = status(3);
    wx = status(4);
    wy = status(5);
    wz = status(6);
end