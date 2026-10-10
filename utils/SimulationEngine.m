classdef SimulationEngine < handle
    %SimulationEngine 仿真引擎类
    %   整合所有模块，提供完整的仿真运行接口

    properties
        % 组件
        environment
        scheduler
        visualizer

        % 仿真参数
        time_step
        total_time
        current_time
        is_running
        is_paused

        % 数据记录
        log
        trajectory_log
        collision_log

        % 回调函数
        on_step
        on_complete
    end

    methods
        function obj = SimulationEngine(varargin)
            % SimulationEngine 构造函数
            p = inputParser;
            addParameter(p, 'DeckLength', 300);
            addParameter(p, 'DeckWidth', 80);
            addParameter(p, 'TimeStep', 0.1);
            addParameter(p, 'TotalTime', 300);
            addParameter(p, 'GridResolution', 10);
            addParameter(p, 'ShowVisualization', true);

            parse(p, varargin{:});

            % 初始化环境
            obj.environment = CarrierEnvironment(...
                'DeckLength', p.Results.DeckLength, ...
                'DeckWidth', p.Results.DeckWidth, ...
                'GridResolution', p.Results.GridResolution);

            % 初始化调度器
            obj.scheduler = Scheduler(obj.environment);

            % 初始化可视化
            if p.Results.ShowVisualization
                obj.visualizer = Visualizer();
                obj.visualizer.initialize(obj.environment);
            end

            % 仿真参数
            obj.time_step = p.Results.TimeStep;
            obj.total_time = p.Results.TotalTime;
            obj.current_time = 0;
            obj.is_running = false;
            obj.is_paused = false;

            % 数据记录
            obj.log = [];
            obj.trajectory_log = [];
            obj.collision_log = [];
        end

        function setupScenario(obj, scenario_name)
            % setupScenario 设置场景
            switch scenario_name
                case 'takeoff_landing'
                    obj.setupTakeoffLandingScenario();
                case 'parking_management'
                    obj.setupParkingScenario();
                case 'emergency'
                    obj.setupEmergencyScenario();
                case 'multi_aircraft'
                    obj.setupMultiAircraftScenario();
                otherwise
                    error('Unknown scenario: %s', scenario_name);
            end
        end

        function setupTakeoffLandingScenario(obj)
            % setupTakeoffLandingScenario 起飞降落场景
            % 添加一些障碍物（模拟临时物体）
            obj.environment.addObstacle(struct('type', 'box', ...
                'min', [50, 30, 0], 'max', [60, 50, 5]));

            % 调度起飞任务
            parking_spots = obj.environment.parking_spots;
            takeoff_zone = obj.environment.takeoff_zone;

            % 起飞任务: 从停机位到起飞区
            obj.scheduler.scheduleTask(parking_spots(1,:), [80, 40, 0], ...
                'StartTime', 0, 'Priority', 10);

            % 降落任务: 从高空到降落区
            obj.scheduler.scheduleTask([250, 40, 200], [240, 40, 0], ...
                'StartTime', 10, 'Priority', 9);
        end

        function setupParkingScenario(obj)
            % setupParkingScenario 停机管理场景
            parking_spots = obj.environment.parking_spots;

            % 调度多个飞机到不同停机位
            for i = 1:min(6, size(parking_spots, 1))
                start_pos = parking_spots(mod(i, size(parking_spots,1))+1, :);
                goal_pos = parking_spots(i, :);
                obj.scheduler.scheduleTask(start_pos, goal_pos, ...
                    'StartTime', (i-1)*5, 'Priority', 5);
            end
        end

        function setupEmergencyScenario(obj)
            % setupEmergencyScenario 紧急情况场景
            % 所有飞机紧急起飞
            takeoff_zone = obj.environment.takeoff_zone;
            parking_spots = obj.environment.parking_spots;

            for i = 1:min(5, size(parking_spots, 1))
                goal_pos = [takeoff_zone(1) + 20, mean(takeoff_zone(2:4)), 0];
                obj.scheduler.scheduleTask(parking_spots(i,:), goal_pos, ...
                    'StartTime', 0, 'Priority', 10);
            end
        end

        function setupMultiAircraftScenario(obj)
            % setupMultiAircraftScenario 多机协同场景
            parking_spots = obj.environment.parking_spots;
            zones = obj.environment.getZones();

            % 起飞任务
            obj.scheduler.scheduleTask(parking_spots(1,:), [80, 40, 50], ...
                'StartTime', 0, 'Priority', 8);
            obj.scheduler.scheduleTask(parking_spots(2,:), [90, 40, 50], ...
                'StartTime', 2, 'Priority', 8);

            % 降落任务
            obj.scheduler.scheduleTask([200, 40, 100], [zones.landing(1)+20, 40, 0], ...
                'StartTime', 5, 'Priority', 9);
            obj.scheduler.scheduleTask([180, 50, 100], [zones.landing(1)+30, 40, 0], ...
                'StartTime', 7, 'Priority', 9);

            % 调度任务
            obj.scheduler.scheduleTask(parking_spots(3,:), parking_spots(5,:), ...
                'StartTime', 10, 'Priority', 5);
        end

        function run(obj, varargin)
            % run 运行仿真
            p = inputParser;
            addParameter(p, 'ShowProgress', true);
            addParameter(p, 'RecordVideo', false);
            addParameter(p, 'VideoFilename', 'simulation.mp4');

            parse(p, varargin{:});

            obj.is_running = true;
            obj.is_paused = false;

            video_writer = [];
            if p.Results.RecordVideo
                video_writer = obj.visualizer.initVideo(p.Results.VideoFilename);
            end

            tic;
            while obj.current_time < obj.total_time && obj.is_running
                if obj.is_paused
                    pause(0.1);
                    continue;
                end

                % 推进时间
                obj.advance();

                % 可视化更新
                if ~isempty(obj.visualizer)
                    obj.visualizer.refreshDisplay(obj.scheduler);
                end

                % 视频录制
                if ~isempty(video_writer)
                    obj.visualizer.recordFrame(video_writer);
                end

                % 进度显示
                if p.Results.ShowProgress && mod(obj.current_time, 10) < obj.time_step
                    fprintf('Time: %.1f / %.1f s (%.1f%%)\n', ...
                           obj.current_time, obj.total_time, ...
                           obj.current_time/obj.total_time*100);
                end

                % 回调
                if ~isempty(obj.on_step)
                    obj.on_step(obj.current_time);
                end

                pause(obj.time_step);
            end

            if ~isempty(video_writer)
                obj.visualizer.closeVideo(video_writer);
            end

            obj.is_running = false;

            % 完成回调
            if ~isempty(obj.on_complete)
                obj.on_complete();
            end

            fprintf('Simulation completed in %.2f seconds\n', toc);
        end

        function advance(obj)
            % advance 单步推进
            % 推进调度器
            obj.scheduler.advanceTime(obj.time_step);

            % 检查碰撞
            collision_check = obj.scheduler.checkCollisions();
            if ~isempty(collision_check.collisions)
                obj.collision_log = [obj.collision_log; collision_check.collisions];
            end

            % 记录数据
            obj.logRecord();

            % 更新时间
            obj.current_time = obj.current_time + obj.time_step;
        end

        function pause(obj)
            % pause 暂停仿真
            obj.is_paused = true;
        end

        function resume(obj)
            % resume 恢复仿真
            obj.is_paused = false;
        end

        function stop(obj)
            % stop 停止仿真
            obj.is_running = false;
        end

        function logRecord(obj)
            % logRecord 记录当前状态
            record = struct();
            record.time = obj.current_time;
            record.scheduler_state = obj.scheduler.getSchedulerSummary();
            record.aircraft_positions = [];

            keys = obj.scheduler.aircraft_list.keys;
            for i = 1:length(keys)
                aircraft = obj.scheduler.aircraft_list(keys{i});
                pos = aircraft.getPosition();
                record.aircraft_positions = [record.aircraft_positions; ...
                    [aircraft.id, pos]];
            end

            obj.log = [obj.log, record];

            % 记录轨迹
            trajectories = obj.scheduler.getAllTrajectories();
            for i = 1:length(trajectories)
                t = trajectories(i);
                obj.trajectory_log = [obj.trajectory_log, t];
            end
        end

        function results = getResults(obj)
            % getResults 获取仿真结果
            results = struct();
            results.log = obj.log;
            results.trajectory_log = obj.trajectory_log;
            results.collision_log = obj.collision_log;
            results.final_state = obj.scheduler.getSchedulerSummary();
            results.total_time = obj.current_time;
        end

        function exportResults(obj, filename)
            % exportResults 导出仿真结果
            results = obj.getResults();

            % 转换为表格格式
            log_table = struct2table([results.log.scheduler_state]);

            % 保存为MAT文件
            save(filename, 'results', 'log_table');
            fprintf('Results exported to %s\n', filename);
        end
    end
end