classdef Scheduler < handle
    %Scheduler 舰载机调度器类
    %   提供外部调度接口，管理多架舰载机的任务分配和路径规划

    properties
        % 调度器配置
        config

        % 环境
        environment

        % 路径规划器
        pathfinder

        % 舰载机列表
        aircraft_list

        % 调度任务队列
        task_queue

        % 仿真时间
        current_time

        % 任务计数器
        task_counter

        % 回调函数
        on_task_start
        on_task_complete
        on_collision_warning
        on_status_update
    end

    methods
        function obj = Scheduler(env, varargin)
            % Scheduler 构造函数
            p = inputParser;
            addParameter(p, 'MaxAircraft', 20);
            addParameter(p, 'PlanningHorizon', 300);  % 规划时间范围 (秒)
            addParameter(p, 'SafetyTimeMargin', 5);  % 安全时间裕度 (秒)
            addParameter(p, 'EnableVisualization', true);

            parse(p, varargin{:});

            obj.config = p.Results;
            obj.environment = env;
            obj.pathfinder = CBS(env, 'TimeLimit', 30);
            obj.aircraft_list = containers.Map('KeyType', 'uint32', 'ValueType', 'any');
            obj.task_queue = [];
            obj.current_time = 0;
            obj.task_counter = 0;
        end

        function aircraft = registerAircraft(obj, varargin)
            % registerAircraft 注册新舰载机
            %   aircraft = registerAircraft(obj) - 使用默认参数
            %   aircraft = registerAircraft(obj, 'PropertyName', PropertyValue, ...)
            aircraft = AircraftDynamics(0, varargin{:});
            obj.aircraft_list(aircraft.id) = aircraft;
        end

        function task_id = scheduleTask(obj, start_pos, goal_pos, varargin)
            % scheduleTask 调度任务 - 外部调用接口
            %   task_id = scheduleTask(obj, start_pos, goal_pos)
            %   task_id = scheduleTask(obj, start_pos, goal_pos, 'StartTime', t)
            %   task_id = scheduleTask(obj, start_pos, goal_pos, 'AircraftId', id)
            %
            %   参数:
            %       start_pos: 起始位置 [x, y, z]
            %       goal_pos: 目标位置 [x, y, z]
            %       'StartTime': 任务开始时间 (秒)，默认为当前时间
            %       'AircraftId': 指定飞机ID，默认为自动分配
            %       'Priority': 任务优先级 (1-10)，数字越大优先级越高
            %       'MaxSpeed': 最大速度约束 (可选)
            %
            %   返回:
            %       task_id: 任务ID，用于查询任务状态

            p = inputParser;
            addParameter(p, 'StartTime', obj.current_time);
            addParameter(p, 'AircraftId', []);
            addParameter(p, 'Priority', 5);
            addParameter(p, 'MaxSpeed', []);

            parse(p, varargin{:});

            obj.task_counter = obj.task_counter + 1;
            task_id = obj.task_counter;

            % 创建任务结构
            task = struct();
            task.id = task_id;
            task.start_position = start_pos;
            task.goal_position = goal_pos;
            task.start_time = p.Results.StartTime;
            task.aircraft_id = p.Results.AircraftId;
            task.priority = p.Results.Priority;
            task.max_speed = p.Results.MaxSpeed;
            task.status = 'pending';
            task.path = [];
            task.trajectory = [];
            task.creation_time = obj.current_time;
            task.planned_time = [];
            task.error = [];

            % 如果未指定飞机，选择可用飞机
            if isempty(task.aircraft_id)
                task.aircraft_id = obj.assignAircraft();
            end

            % 添加到任务队列 (按优先级和开始时间排序)
            obj.task_queue = [obj.task_queue, task];
            priorities = [obj.task_queue.priority];
            start_times = [obj.task_queue.start_time];
            [~, idx] = sortrows([-priorities', start_times']);
            obj.task_queue = obj.task_queue(idx);

            % 检查是否可以开始规划
            if task.start_time <= obj.current_time
                obj.processTask(task_id);
            end
        end

        function task_ids = scheduleBatchTasks(obj, tasks)
            % scheduleBatchTasks 批量调度任务
            %   tasks: 结构体数组，每个包含 start_pos, goal_pos 和可选参数
            %   task_ids = scheduleBatchTasks(obj, tasks)

            task_ids = zeros(length(tasks), 1);
            for i = 1:length(tasks)
                t = tasks(i);
                if isfield(t, 'start_pos') && isfield(t, 'goal_pos')
                    args = {};
                    if isfield(t, 'start_time')
                        args = [args, 'StartTime', t.start_time];
                    end
                    if isfield(t, 'aircraft_id')
                        args = [args, 'AircraftId', t.aircraft_id];
                    end
                    if isfield(t, 'priority')
                        args = [args, 'Priority', t.priority];
                    end
                    task_ids(i) = obj.scheduleTask(t.start_pos, t.goal_pos, args{:});
                end
            end
        end

        function processTask(obj, task_id)
            % processTask 处理单个任务
            task_idx = find([obj.task_queue.id] == task_id, 1);
            if isempty(task_idx)
                return;
            end

            % 更新状态为 planning
            obj.task_queue(task_idx).status = 'planning';

            % 调用路径规划器
            starts = obj.task_queue(task_idx).start_position;
            goals = obj.task_queue(task_idx).goal_position;

            solution = obj.pathfinder.findPath(starts, goals);

            if ~isempty(solution) && isstruct(solution) && isfield(solution, 'paths') && ~isempty(solution.paths)
                obj.task_queue(task_idx).path = solution.paths{1};
                if isfield(solution, 'trajectories') && ~isempty(solution.trajectories)
                    obj.task_queue(task_idx).trajectory = solution.trajectories{1};
                end
                obj.task_queue(task_idx).status = 'planned';
                obj.task_queue(task_idx).planned_time = obj.current_time;

                % 存储轨迹到飞机对象
                if isKey(obj.aircraft_list, obj.task_queue(task_idx).aircraft_id)
                    aircraft = obj.aircraft_list(obj.task_queue(task_idx).aircraft_id);
                    aircraft.is_occupied = true;
                end

                % 执行回调
                if ~isempty(obj.on_task_complete)
                    obj.on_task_complete(obj.task_queue(task_idx));
                end
            else
                obj.task_queue(task_idx).status = 'failed';
                obj.task_queue(task_idx).error = 'No path found';
            end
        end

        function aircraft_id = assignAircraft(obj)
            % assignAircraft 分配可用飞机
            available_ids = [];
            keys = obj.aircraft_list.keys;
            for i = 1:length(keys)
                id = keys{i};
                aircraft = obj.aircraft_list(id);
                if ~aircraft.is_occupied
                    available_ids = [available_ids, id];
                end
            end

            if isempty(available_ids)
                % 创建新飞机
                obj.task_counter = obj.task_counter + 1;
                aircraft_id = obj.task_counter;
                aircraft = AircraftDynamics(aircraft_id);
                obj.aircraft_list(aircraft_id) = aircraft;
            else
                aircraft_id = available_ids(1);
            end
        end

        function status = getTaskStatus(obj, task_id)
            % getTaskStatus 获取任务状态
            task_idx = find([obj.task_queue.id] == task_id, 1);
            if isempty(task_idx)
                status = [];
                return;
            end
            status = obj.task_queue(task_idx);
        end

        function trajectory = getTrajectory(obj, task_id)
            % getTrajectory 获取任务轨迹
            status = obj.getTaskStatus(task_id);
            if isempty(status)
                trajectory = [];
            else
                trajectory = status.trajectory;
            end
        end

        function all_trajectories = getAllTrajectories(obj)
            % getAllTrajectories 获取所有已规划轨迹
            all_trajectories = [];
            for i = 1:length(obj.task_queue)
                task = obj.task_queue(i);
                if strcmp(task.status, 'planned') && ~isempty(task.trajectory)
                    traj = task.trajectory;
                    traj.task_id = task.id;
                    traj.aircraft_id = task.aircraft_id;
                    all_trajectories = [all_trajectories, traj];
                end
            end
        end

        function cancelTask(obj, task_id)
            % cancelTask 取消任务
            task_idx = find([obj.task_queue.id] == task_id, 1);
            if ~isempty(task_idx)
                task = obj.task_queue(task_idx);

                % 释放飞机
                if isKey(obj.aircraft_list, task.aircraft_id)
                    aircraft = obj.aircraft_list(task.aircraft_id);
                    aircraft.is_occupied = false;
                end

                task.status = 'cancelled';
                obj.task_queue(task_idx) = task;
            end
        end

        function advanceTime(obj, dt)
            % advanceTime 推进仿真时间
            obj.current_time = obj.current_time + dt;
            obj.environment.advanceTime(dt);

            % 检查是否有任务需要开始
            pending_tasks = obj.task_queue(strcmp({obj.task_queue.status}, 'pending'));
            for i = 1:length(pending_tasks)
                if pending_tasks(i).start_time <= obj.current_time
                    obj.processTask(pending_tasks(i).id);
                end
            end

            % 更新飞机状态
            obj.updateAircraftStates();
        end

        function updateAircraftStates(obj)
            % updateAircraftStates 更新飞机状态
            keys = obj.aircraft_list.keys;
            for i = 1:length(keys)
                id = keys{i};
                aircraft = obj.aircraft_list(id);

                if aircraft.is_occupied
                    % 查找对应任务
                    for j = 1:length(obj.task_queue)
                        task = obj.task_queue(j);
                        if task.aircraft_id == id && strcmp(task.status, 'planned')
                            % 沿轨迹移动
                            traj = task.trajectory;
                            time_idx = find(traj.time <= obj.current_time, 1, 'last');
                            if ~isempty(time_idx) && time_idx < size(traj.position, 1)
                                aircraft.position = traj.position(time_idx, :);
                                aircraft.velocity = traj.velocity(time_idx, :);
                            end
                        end
                    end
                end
            end
        end

        function collision_check = checkCollisions(obj)
            % checkCollisions 检查碰撞
            collision_check = struct();
            collision_check.collisions = [];
            collision_check.warnings = [];

            keys = obj.aircraft_list.keys;
            n = length(keys);
            for i = 1:n
                for j = i+1:n
                    id1 = keys{i};
                    id2 = keys{j};
                    a1 = obj.aircraft_list(id1);
                    a2 = obj.aircraft_list(id2);

                    dist = a1.distanceTo(a2);
                    if dist < min(a1.safety_distance, a2.safety_distance)
                        collision = struct();
                        collision.aircraft1 = id1;
                        collision.aircraft2 = id2;
                        collision.distance = dist;
                        collision_check.collisions = [collision_check.collisions, collision];
                    elseif dist < max(a1.safety_distance, a2.safety_distance) * 2
                        warning = struct();
                        warning.aircraft1 = id1;
                        warning.aircraft2 = id2;
                        warning.distance = dist;
                        warning.time = obj.current_time;
                        collision_check.warnings = [collision_check.warnings, warning];

                        % 触发回调
                        if ~isempty(obj.on_collision_warning)
                            obj.on_collision_warning(warning);
                        end
                    end
                end
            end
        end

        function summary = getSchedulerSummary(obj)
            % getSchedulerSummary 获取调度器摘要
            summary = struct();
            summary.current_time = obj.current_time;
            summary.total_aircraft = obj.aircraft_list.Count;
            summary.total_tasks = length(obj.task_queue);
            summary.pending_tasks = sum(strcmp({obj.task_queue.status}, 'pending'));
            summary.planned_tasks = sum(strcmp({obj.task_queue.status}, 'planned'));
            summary.completed_tasks = sum(strcmp({obj.task_queue.status}, 'completed'));
            summary.failed_tasks = sum(strcmp({obj.task_queue.status}, 'failed'));
            % 计算被占用的飞机数量
            keys = obj.aircraft_list.keys;
            active_count = 0;
            for i = 1:length(keys)
                if obj.aircraft_list(keys{i}).is_occupied
                    active_count = active_count + 1;
                end
            end
            summary.active_aircraft = active_count;
        end

        % ========== 外部API接口方法 ==========

        function setCallbacks(obj, callback_type, callback_func)
            % setCallbacks 设置回调函数
            %   setCallbacks(obj, 'task_start', @callback_func)
            %   setCallbacks(obj, 'task_complete', @callback_func)
            %   setCallbacks(obj, 'collision_warning', @callback_func)
            %   setCallbacks(obj, 'status_update', @callback_func)
            switch callback_type
                case 'task_start'
                    obj.on_task_start = callback_func;
                case 'task_complete'
                    obj.on_task_complete = callback_func;
                case 'collision_warning'
                    obj.on_collision_warning = callback_func;
                case 'status_update'
                    obj.on_status_update = callback_func;
            end
        end

        function trajectory_data = exportTrajectory(obj, task_id)
            % exportTrajectory 导出轨迹数据用于外部使用
            trajectory_data = struct();
            status = obj.getTaskStatus(task_id);
            if ~isempty(status) && ~isempty(status.trajectory)
                trajectory_data.agent_id = status.aircraft_id;
                trajectory_data.start_position = status.start_position;
                trajectory_data.goal_position = status.goal_position;
                trajectory_data.time = status.trajectory.time;
                trajectory_data.position = status.trajectory.position;
                trajectory_data.velocity = status.trajectory.velocity;
                trajectory_data.status = status.status;
            end
        end

        function scheduleFromJSON(obj, json_str)
            % scheduleFromJSON 从JSON格式解析并调度任务
            %   用于外部系统调用
            try
                data = jsondecode(json_str);
                tasks = data.tasks;
                obj.scheduleBatchTasks(tasks);
            catch
                error('Invalid JSON format');
            end
        end

        function json_str = toJSON(obj)
            % toJSON 导出调度器状态为JSON
            data = struct();
            data.current_time = obj.current_time;
            data.total_aircraft = obj.aircraft_list.Count;
            data.tasks = obj.task_queue;
            json_str = jsonencode(data);
        end

        function reset(obj)
            % reset 重置调度器
            obj.current_time = 0;
            obj.task_queue = [];
            obj.environment.current_time = 0;

            keys = obj.aircraft_list.keys;
            for i = 1:length(keys)
                obj.aircraft_list(keys{i}).reset();
            end
        end
    end
end