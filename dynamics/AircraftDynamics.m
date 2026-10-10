classdef AircraftDynamics < handle
    %AircraftDynamics 舰载机动力学模型类
    %   实现舰载机的运动学/动力学模型，包括起飞、巡航、降落等阶段

    properties
        % 基本参数
        id                      % 飞机唯一标识
        position                % 当前位置 [x, y, z] (米)
        velocity                % 当前速度 [vx, vy, vz] (米/秒)
        attitude                % 姿态角 [roll, pitch, yaw] (弧度)
        attitude_rate           % 姿态角速率 (弧度/秒)

        % 物理参数
        max_velocity            % 最大速度 (m/s)
        min_velocity            % 最小速度 (m/s)
        max_acceleration        % 最大加速度 (m/s^2)
        max_angular_rate        % 最大角速率 (rad/s)
        cruise_velocity         % 巡航速度 (m/s)

        % 质量特性
        mass                    % 质量 (kg)
        wing_area               % 机翼面积 (m^2)

        % 安全参数
        min_altitude             % 最小飞行高度 (m)
        max_altitude             % 最大飞行高度 (m)
        safety_distance          % 安全间距 (m)

        % 状态
        state                   % 飞行状态: 'ground', 'takeoff', 'climbing', ...
        is_occupied             % 是否被占用
    end

    methods
        function obj = AircraftDynamics(aircraft_id, varargin)
            % AircraftDynamics 构造函数
            %   obj = AircraftDynamics(id) - 使用默认参数
            %   obj = AircraftDynamics(id, 'PropertyName', PropertyValue, ...)
            p = inputParser;
            addParameter(p, 'Position', [0, 0, 0]);
            addParameter(p, 'InitialVelocity', 50);
            addParameter(p, 'MaxVelocity', 250);
            addParameter(p, 'MinVelocity', 30);
            addParameter(p, 'MaxAcceleration', 5);
            addParameter(p, 'Mass', 15000);
            addParameter(p, 'WingArea', 25);
            addParameter(p, 'MinAltitude', 10);
            addParameter(p, 'MaxAltitude', 5000);
            addParameter(p, 'SafetyDistance', 50);

            parse(p, varargin{:});

            obj.id = aircraft_id;
            obj.position = p.Results.Position;
            obj.velocity = [p.Results.InitialVelocity, 0, 0];
            obj.attitude = [0, 0, 0];
            obj.attitude_rate = [0, 0, 0];
            obj.max_velocity = p.Results.MaxVelocity;
            obj.min_velocity = p.Results.MinVelocity;
            obj.max_acceleration = p.Results.MaxAcceleration;
            obj.max_angular_rate = 0.1;
            obj.cruise_velocity = (p.Results.MaxVelocity + p.Results.MinVelocity) / 2;
            obj.mass = p.Results.Mass;
            obj.wing_area = p.Results.WingArea;
            obj.min_altitude = p.Results.MinAltitude;
            obj.max_altitude = p.Results.MaxAltitude;
            obj.safety_distance = p.Results.SafetyDistance;
            obj.state = 'ground';
            obj.is_occupied = false;
        end

        function update(obj, dt, control_input)
            % update 更新飞机状态
            %   update(obj, dt, control_input) - 根据控制输入更新状态
            %
            %   control_input: 结构体，包含:
            %       - thrust: 推力指令 (0-1)
            %       - bank_angle: 滚转角指令 (rad)
            %       - pitch_angle: 俯仰角指令 (rad)
            %       - target_velocity: 目标速度 (可选)

            if nargin < 3
                control_input = struct();
            end

            % 默认控制输入
            if ~isfield(control_input, 'thrust')
                control_input.thrust = 0.5;
            end
            if ~isfield(control_input, 'bank_angle')
                control_input.bank_angle = 0;
            end
            if ~isfield(control_input, 'pitch_angle')
                control_input.pitch_angle = 0;
            end

            % 速度更新 (简化动力学)
            current_speed = norm(obj.velocity);
            target_speed = obj.cruise_velocity;
            if isfield(control_input, 'target_velocity')
                target_speed = control_input.target_velocity;
            end

            % 速度大小调整
            speed_error = target_speed - current_speed;
            acceleration_cmd = min(obj.max_acceleration, abs(speed_error));
            if speed_error > 0
                new_speed = current_speed + acceleration_cmd * dt;
            else
                new_speed = current_speed - acceleration_cmd * dt;
            end
            new_speed = max(obj.min_velocity, min(obj.max_velocity, new_speed));

            % 姿态更新
            roll_cmd = control_input.bank_angle;
            pitch_cmd = control_input.pitch_angle;
            roll_rate = (roll_cmd - obj.attitude(1)) / dt;
            pitch_rate = (pitch_cmd - obj.attitude(2)) / dt;
            roll_rate = max(-obj.max_angular_rate, min(obj.max_angular_rate, roll_rate));
            pitch_rate = max(-obj.max_angular_rate, min(obj.max_angular_rate, pitch_rate));

            obj.attitude(1) = obj.attitude(1) + roll_rate * dt;
            obj.attitude(2) = obj.attitude(2) + pitch_rate * dt;
            obj.attitude(3) = obj.attitude(3) + control_input.bank_angle * dt * 0.5; % 偏航协调转弯

            % 计算速度方向
            heading = obj.attitude(3);
            pitch = obj.attitude(2);
            bank = obj.attitude(1);

            % 速度向量更新
            v_horizontal = new_speed * cos(pitch);
            v_vertical = new_speed * sin(pitch);

            obj.velocity = [v_horizontal * cos(heading + bank*0.1), ...
                           v_horizontal * sin(heading + bank*0.1), ...
                           v_vertical];

            % 位置更新
            obj.position = obj.position + obj.velocity * dt;

            % 高度限制
            obj.position(3) = max(obj.min_altitude, min(obj.max_altitude, obj.position(3)));

            % 更新状态
            obj.updateState();
        end

        function updateState(obj)
            % updateState 根据当前状态更新飞行阶段
            if obj.position(3) < obj.min_altitude + 5
                obj.state = 'ground';
            elseif obj.position(3) < 100
                obj.state = 'takeoff';
            elseif abs(obj.velocity(3)) > 5
                obj.state = 'climbing';
            else
                obj.state = 'cruise';
            end
        end

        function pos = getPosition(obj)
            % getPosition 获取当前位置
            pos = obj.position;
        end

        function vel = getVelocity(obj)
            % getVelocity 获取当前速度
            vel = obj.velocity;
        end

        function dist = distanceTo(obj, other_aircraft)
            % distanceTo 计算与另一架飞机的距离
            if isa(other_aircraft, 'AircraftDynamics')
                dist = norm(obj.position - other_aircraft.position);
            else
                dist = norm(obj.position - other_aircraft);
            end
        end

        function checkCollision(obj, other_aircraft)
            % checkCollision 检查是否与另一架飞机发生碰撞
            dist = obj.distanceTo(other_aircraft);
            checkCollision = dist < obj.safety_distance;
        end

        function setPosition(obj, new_position)
            % setPosition 设置位置
            obj.position = new_position;
        end

        function setVelocity(obj, new_velocity)
            % setVelocity 设置速度
            obj.velocity = new_velocity;
        end

        function reset(obj)
            % reset 重置飞机状态
            obj.position = [0, 0, 0];
            obj.velocity = [0, 0, 0];
            obj.attitude = [0, 0, 0];
            obj.state = 'ground';
            obj.is_occupied = false;
        end
    end
end