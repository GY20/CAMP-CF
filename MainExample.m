% MainExample.m
% 舰载机路径规划系统主示例
% 演示如何使用各个模块进行舰载机调度和路径规划

%% 清理工作空间
clear;
clc;
close all;

fprintf('========================================\n');
fprintf('Carrier Aircraft Path Planning System\n');
fprintf('Using CBS (Conflict-Based Search)\n');
fprintf('========================================\n\n');

addpath("dynamics\");
addpath("pathfinding\");
addpath("scheduler\");
addpath("utils\");
addpath("visualization\");

%% 1. 初始化环境
fprintf('[Step 1] Initializing environment...\n');
environment = CarrierEnvironment('DeckLength', 300, 'DeckWidth', 80, 'GridResolution', 10);

% 添加一些障碍物（模拟舰岛上设备）
environment.addObstacle(struct('type', 'box', ...
    'min', [140, 35, 0], 'max', [160, 45, 15]));  % 舰岛
environment.addObstacle(struct('type', 'sphere', ...
    'center', [200, 50, 0], 'radius', 5));  % 雷达

fprintf('   Environment initialized: %.0f x %.0f meters\n', environment.deck_length, environment.deck_width);

%% 2. 初始化调度器
fprintf('[Step 2] Initializing scheduler...\n');
scheduler = Scheduler(environment, 'MaxAircraft', 20);
fprintf('   Scheduler ready.\n');

%% 3. 定义任务
fprintf('[Step 3] Defining tasks...\n');

% 获取可用停机位
parking_spots = environment.parking_spots;
takeoff_zone = environment.takeoff_zone;
landing_zone = environment.landing_zone;

% 定义任务列表
tasks = struct('start_pos', {}, 'goal_pos', {}, 'start_time', {}, 'priority', {});

% 任务1: 起飞 (从停机位到起飞区)
tasks(end+1) = struct('start_pos', parking_spots(1,:), ...
                       'goal_pos', [80, 40, 50], ...
                       'start_time', 0, ...
                       'priority', 10);

% 任务2: 起飞 (另一架)
tasks(end+1) = struct('start_pos', parking_spots(3,:), ...
                       'goal_pos', [90, 40, 50], ...
                       'start_time', 5, ...
                       'priority', 9);

% 任务3: 降落 (从高空到降落区)
tasks(end+1) = struct('start_pos', [250, 40, 200], ...
                       'goal_pos', [landing_zone(1)+20, 40, 0], ...
                       'start_time', 15, ...
                       'priority', 10);

% 任务4: 转移 (从一个停机位到另一个)
tasks(end+1) = struct('start_pos', parking_spots(2,:), ...
                       'goal_pos', parking_spots(5,:), ...
                       'start_time', 20, ...
                       'priority', 5);

% 任务5: 降落 (另一架)
tasks(end+1) = struct('start_pos', [220, 50, 150], ...
                       'goal_pos', [landing_zone(1)+30, 50, 0], ...
                       'start_time', 25, ...
                       'priority', 9);

fprintf('   Created %d tasks:\n', length(tasks));
for i = 1:length(tasks)
    fprintf('     Task %d: [%.0f, %.0f, %.0f] -> [%.0f, %.0f, %.0f] @ t=%.0fs\n', ...
            i, tasks(i).start_pos, tasks(i).goal_pos, tasks(i).start_time);
end

%% 4. 提交任务
fprintf('\n[Step 4] Submitting tasks to scheduler...\n');

task_ids = scheduler.scheduleBatchTasks(tasks);
fprintf('   Submitted %d tasks, IDs: ', length(task_ids));
fprintf('%d ', task_ids);
fprintf('\n');

%% 5. 等待规划完成
fprintf('\n[Step 5] Planning paths (CBS algorithm)...\n');
pause(2);  % 等待规划

fprintf('   Planning status:\n');
for i = 1:length(task_ids)
    status = scheduler.getTaskStatus(task_ids(i));
    if ~isempty(status)
        fprintf('     Task %d (Aircraft %d): %s\n', ...
                status.id, status.aircraft_id, status.status);
    end
end

%% 6. 获取轨迹
fprintf('\n[Step 6] Extracting trajectories...\n');
all_trajectories = scheduler.getAllTrajectories();

fprintf('   Found %d planned trajectories:\n', length(all_trajectories));
for i = 1:length(all_trajectories)
    traj = all_trajectories(i);
    fprintf('     Aircraft %d: %d steps, start [%.1f, %.1f, %.1f] -> end [%.1f, %.1f, %.1f]\n', ...
            traj.agent_id, length(traj.time), ...
            traj.position(1,:), traj.position(end,:));
end

%% 7. 可视化
fprintf('\n[Step 7] Creating visualization...\n');

% 创建可视化
visualizer = Visualizer();
visualizer.initialize(environment);

% 绘制轨迹
if ~isempty(all_trajectories)
    visualizer.plotAllTrajectories(all_trajectories);
end

% 绘制飞机当前位置
keys = scheduler.aircraft_list.keys;
for i = 1:length(keys)
    aircraft = scheduler.aircraft_list(keys{i});
    if aircraft.is_occupied
        visualizer.plotAircraft(aircraft);
    end
end

title('Carrier Aircraft Planning - CBS Path Finding');
legend('show');
drawnow;

%% 8. 简单动画演示
fprintf('\n[Step 8] Running animation (5 seconds)...\n');

% 初始化动画参数
animation_dt = 0.5;  % 动画时间步
simulation_time = 0:animation_dt:10;

for t = simulation_time
    % 更新调度器
    scheduler.advanceTime(animation_dt);
    
    % 刷新可视化
    visualizer.refreshDisplay(scheduler);
    pause(0.05);
end

fprintf('   Animation completed.\n');

%% 9. 导出结果
fprintf('\n[Step 9] Exporting results...\n');

% 获取调度器状态摘要
summary = scheduler.getSchedulerSummary();
fprintf('   Scheduler Summary:\n');
fprintf('     Current Time: %.1f s\n', summary.current_time);
fprintf('     Total Aircraft: %d\n', summary.total_aircraft);
fprintf('     Total Tasks: %d\n', summary.total_tasks);
fprintf('     Pending: %d, Planned: %d, Failed: %d\n', ...
        summary.pending_tasks, summary.planned_tasks, summary.failed_tasks);

% 显示CBS算法统计
fprintf('\n   CBS Algorithm Statistics:\n');
fprintf('     Expanded nodes: %d\n', scheduler.pathfinder.num_expanded);
fprintf('     Solution cost: %.1f\n', scheduler.pathfinder.solution_cost);
fprintf('     Total conflicts resolved: %d\n', length(scheduler.pathfinder.conflicts));

%% 10. 演示外部API调用
fprintf('\n[Step 10] Demonstrating external API...\n');

% JSON格式输入
json_input = '{"tasks":[{"start_pos":[10,10,0],"goal_pos":[80,40,0],"start_time":30,"priority":5}]}';
fprintf('   JSON input: %s\n', json_input);

% 解析并调度
try
    scheduler.scheduleFromJSON(json_input);
    pause(1);
    fprintf('   Task scheduled successfully via JSON API.\n');
catch
    fprintf('   JSON parsing failed (expected in offline mode).\n');
end

% 导出轨迹数据
fprintf('\n   Exporting trajectory for Task 1:\n');
trajectory_data = scheduler.exportTrajectory(task_ids(1));
if ~isempty(trajectory_data)
    fprintf('     Agent ID: %d\n', trajectory_data.agent_id);
    fprintf('     Start: [%.1f, %.1f, %.1f]\n', trajectory_data.start_position);
    fprintf('     Goal: [%.1f, %.1f, %.1f]\n', trajectory_data.goal_position);
    fprintf('     Path length: %d steps\n', length(trajectory_data.time));
end

fprintf('\n========================================\n');
fprintf('Example completed successfully!\n');
fprintf('========================================\n');