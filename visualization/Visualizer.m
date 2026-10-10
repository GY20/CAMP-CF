classdef Visualizer < handle
    %Visualizer 舰载机规划系统可视化类
    %   提供3D可视化、轨迹显示、动画等功能

    properties
        % 图形句柄
        figure_h
        axes_3d
        axes_2d
        grid_plot
        aircraft_handles
        trajectory_handles
        text_handles

        % 可视化配置
        config

        % 动画参数
        animation_speed
        current_frame
        is_animating

        % 配色方案
        colors
    end

    methods
        function obj = Visualizer(varargin)
            % Visualizer 构造函数
            p = inputParser;
            addParameter(p, 'FigurePosition', [100, 100, 1200, 800]);
            addParameter(p, 'AnimationSpeed', 1);
            addParameter(p, 'ShowGrid', true);
            addParameter(p, 'ShowTrajectories', true);
            addParameter(p, 'ShowLabels', true);

            parse(p, varargin{:});

            obj.config = p.Results;
            obj.animation_speed = p.Results.AnimationSpeed;
            obj.current_frame = 1;
            obj.is_animating = false;

            % 配色方案
            obj.colors = struct();
            obj.colors.aircraft = lines(20);
            obj.colors.ground = [0.7, 0.7, 0.7];
            obj.colors.takeoff_zone = [0.2, 0.8, 0.2];
            obj.colors.landing_zone = [0.8, 0.2, 0.2];
            obj.colors.parking = [0.2, 0.2, 0.8];
            obj.colors.taxiway = [0.9, 0.9, 0.5];
            obj.colors.obstacle = [0.3, 0.3, 0.3];

            obj.aircraft_handles = [];
            obj.trajectory_handles = [];
        end

        function initialize(obj, environment)
            % initialize 初始化可视化窗口
            clf('reset');

            % 创建主窗口
            obj.figure_h = figure('Name', 'Carrier Aircraft Planning System', ...
                                   'NumberTitle', 'off', ...
                                   'Position', obj.config.FigurePosition);

            % 创建3D子图
            obj.axes_3d = subplot(1, 2, 1);
            hold on;
            grid on;

            % 创建2D俯视图
            obj.axes_2d = subplot(1, 2, 2);
            hold on;
            grid on;

            % 绘制环境
            obj.plotEnvironment(environment);
        end

        function plotEnvironment(obj, env)
            % plotEnvironment 绘制环境
            % 3D视图
            axes(obj.axes_3d);

            % 绘制地面
            [X, Y] = meshgrid(0:50:env.deck_length, 0:50:env.deck_width);
            Z = zeros(size(X));
            surf(X, Y, Z, 'FaceColor', obj.colors.ground, 'FaceAlpha', 0.3, 'EdgeColor', 'none');

            % 绘制起飞区
            takeoff = env.takeoff_zone;
            rectangle('Position', [takeoff(1), takeoff(2), takeoff(3)-takeoff(1), takeoff(4)-takeoff(2)], ...
                     'FaceColor', obj.colors.takeoff_zone, 'FaceAlpha', 0.3, ...
                     'EdgeColor', obj.colors.takeoff_zone, 'LineWidth', 2);

            % 绘制降落区
            landing = env.landing_zone;
            rectangle('Position', [landing(1), landing(2), landing(3)-landing(1), landing(4)-landing(2)], ...
                     'FaceColor', obj.colors.landing_zone, 'FaceAlpha', 0.3, ...
                     'EdgeColor', obj.colors.landing_zone, 'LineWidth', 2);

            % 绘制停机位
            if ~isempty(env.parking_spots)
                scatter3(env.parking_spots(:,1), env.parking_spots(:,2), ...
                        env.parking_spots(:,3)+1, 50, 's', ...
                        'MarkerFaceColor', obj.colors.parking, ...
                        'MarkerEdgeColor', 'k');
            end

            % 绘制滑行通道
            if ~isempty(env.taxiway)
                plot3(env.taxiway(:,1), env.taxiway(:,2), env.taxiway(:,3), ...
                     'Color', obj.colors.taxiway, 'LineWidth', 3);
            end

            % 绘制障碍物
            for i = 1:length(env.obstacles)
                obj.plotObstacle(env.obstacles{i});
            end

            % 设置3D视图属性
            xlabel('X (m)');
            ylabel('Y (m)');
            zlabel('Altitude (m)');
            title('3D View');
            view(45, 30);
            axis([0 env.deck_length 0 env.deck_width 0 env.environment_bounds(6)]);
            camlight;

            % 2D俯视图
            axes(obj.axes_2d);

            rectangle('Position', [0, 0, env.deck_length, env.deck_width], ...
                     'FaceColor', obj.colors.ground, 'FaceAlpha', 0.5, ...
                     'EdgeColor', 'k', 'LineWidth', 2);

            % 起飞区
            rectangle('Position', [takeoff(1), takeoff(2), takeoff(3)-takeoff(1), takeoff(4)-takeoff(2)], ...
                     'FaceColor', obj.colors.takeoff_zone, 'FaceAlpha', 0.5, ...
                     'EdgeColor', obj.colors.takeoff_zone, 'LineWidth', 2);

            % 降落区
            rectangle('Position', [landing(1), landing(2), landing(3)-landing(1), landing(4)-landing(2)], ...
                     'FaceColor', obj.colors.landing_zone, 'FaceAlpha', 0.5, ...
                     'EdgeColor', obj.colors.landing_zone, 'LineWidth', 2);

            % 停机位
            if ~isempty(env.parking_spots)
                scatter(env.parking_spots(:,1), env.parking_spots(:,2), ...
                       50, 's', 'MarkerFaceColor', obj.colors.parking, ...
                       'MarkerEdgeColor', 'k');
            end

            % 滑行通道
            if ~isempty(env.taxiway)
                plot(env.taxiway(:,1), env.taxiway(:,2), ...
                    'Color', obj.colors.taxiway, 'LineWidth', 3);
            end

            xlabel('X (m)');
            ylabel('Y (m)');
            title('Top View (2D)');
            axis([0 env.deck_length 0 env.deck_width]);
            axis equal;
        end

        function plotObstacle(obj, obstacle)
            % plotObstacle 绘制障碍物
            if ~isfield(obstacle, 'type')
                return;
            end

            hold on;
            switch obstacle.type
                case 'sphere'
                    [X, Y, Z] = sphere(20);
                    surf(obstacle.center(1) + X*obstacle.radius, ...
                         obstacle.center(2) + Y*obstacle.radius, ...
                         obstacle.center(3) + Z*obstacle.radius, ...
                         'FaceColor', obj.colors.obstacle, 'FaceAlpha', 0.5);
                case 'box'
                    x = [obstacle.min(1), obstacle.max(1)];
                    y = [obstacle.min(2), obstacle.max(2)];
                    z = [obstacle.min(3), obstacle.max(3)];
                    [X, Y, Z] = meshgrid(x, y, z);
                    % 简化为线框盒
                    edges = obstacle.min;
                    dims = obstacle.max - obstacle.min;
                    rectangle3(edges(1), edges(2), edges(3), dims(1), dims(2), dims(3));
            end
        end

        function plotAircraft(obj, aircraft, color_idx)
            % plotAircraft 绘制飞机
            if nargin < 3
                color_idx = mod(aircraft.id, 20) + 1;
            end

            pos = aircraft.getPosition();
            color = obj.colors.aircraft(color_idx, :);

            % 3D视图 - 飞机图标
            axes(obj.axes_3d);
            [x, y, z] = sphere(10);
            surf(pos(1) + x*5, pos(2) + y*5, pos(3) + z*5, ...
                 'FaceColor', color, 'EdgeColor', 'none');

            % 速度向量
            vel = aircraft.getVelocity();
            if norm(vel) > 0
                quiver3(pos(1), pos(2), pos(3), ...
                       vel(1)*0.5, vel(2)*0.5, vel(3)*0.5, ...
                       'Color', 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);
            end

            % 2D视图 - 投影
            axes(obj.axes_2d);
            scatter(pos(1), pos(2), 100, 'o', 'MarkerFaceColor', color, ...
                   'MarkerEdgeColor', 'k', 'LineWidth', 2);

            % 添加标签
            if obj.config.ShowLabels
                text(pos(1)+5, pos(2)+5, sprintf('A%d', aircraft.id), ...
                    'FontSize', 10, 'FontWeight', 'bold');
            end
        end

        function plotTrajectory(obj, trajectory, varargin)
            % plotTrajectory 绘制轨迹
            p = inputParser;
            addParameter(p, 'Color', []);
            addParameter(p, 'LineWidth', 2);
            addParameter(p, 'ShowPoints', true);
            addParameter(p, 'ViewMode', 'both');  % '3d', '2d', 'both'

            parse(p, varargin{:});

            if isempty(p.Results.Color)
                color_idx = mod(trajectory.agent_id, 20) + 1;
                color = obj.colors.aircraft(color_idx, :);
            else
                color = p.Results.Color;
            end

            positions = trajectory.position;
            times = trajectory.time;

            switch p.Results.ViewMode
                case {'3d', 'both'}
                    axes(obj.axes_3d);
                    plot3(positions(:,1), positions(:,2), positions(:,3), ...
                         'Color', color, 'LineWidth', p.Results.LineWidth);
                    if p.Results.ShowPoints
                        scatter3(positions(:,1), positions(:,2), positions(:,3), ...
                                20, color, 'filled');
                    end

                case {'2d', 'both'}
                    axes(obj.axes_2d);
                    plot(positions(:,1), positions(:,2), ...
                        'Color', color, 'LineWidth', p.Results.LineWidth);
                    if p.Results.ShowPoints
                        scatter(positions(:,1), positions(:,2), ...
                               20, color, 'filled');
                    end
            end
        end

        function plotAllTrajectories(obj, trajectories)
            % plotAllTrajectories 绘制所有轨迹
            for i = 1:length(trajectories)
                obj.plotTrajectory(trajectories(i), 'ViewMode', 'both');
            end
            drawnow;
        end

        function updateAircraftPosition(obj, aircraft)
            % updateAircraftPosition 更新飞机位置显示
            % (用于动画)
            obj.plotAircraft(aircraft);
        end

        function h_anim = createAnimation(obj, scheduler, duration, dt)
            % createAnimation 创建动画对象
            %   h_anim = createAnimation(obj, scheduler, duration, dt)
            %   返回动画句柄，可用于播放/暂停

            h_anim = struct();
            h_anim.scheduler = scheduler;
            h_anim.duration = duration;
            h_anim.dt = dt;
            h_anim.current_time = 0;
            h_anim.is_paused = false;
            h_anim.timer = [];

            function animStep(h_anim)
                while h_anim.current_time < h_anim.duration && ~h_anim.is_paused
                    tic;

                    % 更新调度器
                    h_anim.scheduler.advanceTime(h_anim.dt);
                    h_anim.current_time = h_anim.current_time + h_anim.dt;

                    % 刷新显示
                    obj.refreshDisplay(h_anim.scheduler);
                    drawnow limitrate;

                    % 控制速度
                    pause(max(0, h_anim.dt/obj.animation_speed - toc));
                end
            end

            h_anim.stepFcn = @animStep;
        end

        function refreshDisplay(obj, scheduler)
            % refreshDisplay 刷新显示
            % 清除旧数据
            cla(obj.axes_3d);
            cla(obj.axes_2d);

            % 重新绘制环境
            obj.plotEnvironment(scheduler.environment);

            % 绘制轨迹
            trajectories = scheduler.getAllTrajectories();
            obj.plotAllTrajectories(trajectories);

            % 绘制飞机
            keys = scheduler.aircraft_list.keys;
            for i = 1:length(keys)
                aircraft = scheduler.aircraft_list(keys{i});
                if aircraft.is_occupied
                    obj.plotAircraft(aircraft);
                end
            end

            % 更新时间显示
            title(obj.axes_3d, sprintf('3D View - Time: %.1f s', scheduler.current_time));
            title(obj.axes_2d, sprintf('Top View - Time: %.1f s', scheduler.current_time));
        end

        function saveFrame(obj, filename)
            % saveFrame 保存当前帧
            frame = getframe(obj.figure_h);
            imwrite(frame.cdata, filename);
        end

        function video_writer = initVideo(obj, filename, fps)
            % initVideo 初始化视频录制
            if nargin < 3
                fps = 30;
            end
            video_writer = VideoWriter(filename, 'MPEG-4');
            video_writer.FrameRate = fps;
            video_writer.open();
        end

        function recordFrame(obj, video_writer)
            % recordFrame 录制当前帧
            frame = getframe(obj.figure_h);
            writeVideo(video_writer, frame);
        end

        function closeVideo(obj, video_writer)
            % closeVideo 关闭视频录制
            video_writer.close();
        end

        function plotConflictMap(obj, conflicts)
            % plotConflictMap 绘制冲突地图
            figure('Name', 'Conflict Analysis');
            hold on;
            grid on;

            for i = 1:length(conflicts)
                c = conflicts(i);
                scatter3(c.position(1), c.position(2), c.position(3), 200, 'r', 'x', 'LineWidth', 3);
                text(c.position(1)+5, c.position(2)+5, c.position(3)+5, ...
                    sprintf('A%d-A%d@t=%d', c.agent1, c.agent2, c.time));
            end

            xlabel('X (m)');
            ylabel('Y (m)');
            zlabel('Altitude (m)');
            title('Conflict Map');
        end

        function fig_stats = plotStatistics(obj, scheduler)
            % plotStatistics 绘制统计图表
            fig_stats = figure('Name', 'Scheduling Statistics');

            % 任务状态饼图
            subplot(2, 2, 1);
            statuses = {scheduler.task_queue.status};
            unique_statuses = unique(statuses);
            counts = zeros(length(unique_statuses), 1);
            for i = 1:length(unique_statuses)
                counts(i) = sum(strcmp(statuses, unique_statuses{i}));
            end
            pie(counts, unique_statuses);
            title('Task Status Distribution');

            % 时间线图
            subplot(2, 2, 2);
            for i = 1:length(scheduler.task_queue)
                task = scheduler.task_queue(i);
                if ~isempty(task.trajectory)
                    plot(task.trajectory.time, 'LineWidth', 2);
                    hold on;
                end
            end
            xlabel('Time Step');
            ylabel('Trajectory Index');
            title('Task Timelines');
            legend('show');

            % 飞机利用率
            subplot(2, 2, 3);
            keys = scheduler.aircraft_list.keys;
            n = length(keys);
            utilization = zeros(n, 1);
            for i = 1:n
                aircraft = scheduler.aircraft_list(keys{i});
                utilization(i) = aircraft.is_occupied;
            end
            bar(1:n, utilization);
            xlabel('Aircraft ID');
            ylabel('Status (1=Busy, 0=Free)');
            title('Aircraft Utilization');

            % 路径长度分布
            subplot(2, 2, 4);
            path_lengths = [];
            for i = 1:length(scheduler.task_queue)
                task = scheduler.task_queue(i);
                if ~isempty(task.path)
                    path_lengths(end+1) = size(task.path, 1);
                end
            end
            if ~isempty(path_lengths)
                histogram(path_lengths);
                xlabel('Path Length (steps)');
                ylabel('Count');
                title('Path Length Distribution');
            end
        end
    end
end

% 辅助函数：绘制3D矩形
function rectangle3(x, y, z, w, h, d)
    % 绘制线框矩形
    vertices = [
        x, y, z;
        x+w, y, z;
        x+w, y+h, z;
        x, y+h, z;
        x, y, z;
        x, y, z+d;
        x+w, y, z+d;
        x+w, y+h, z+d;
        x, y+h, z+d;
        x, y, z+d;
        x+w, y, z+d;
        x+w, y, z;
        x+w, y+h, z;
        x+w, y+h, z+d;
        x, y+h, z+d;
        x, y+h, z;
    ];
    plot3(vertices(:,1), vertices(:,2), vertices(:,3), 'k', 'LineWidth', 2);
end