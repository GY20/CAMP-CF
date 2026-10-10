classdef CBS < handle
    %CBS 冲突基搜索算法 (Conflict-Based Search)
    %   多智能体路径规划算法，用于为多架舰载机规划无冲突路径

    properties
        % 环境引用
        environment

        % 算法参数
        max_iterations          % 最大迭代次数
        time_limit              % 时间限制 (秒)
        node_limit              % 节点限制

        % 结果
        solution                 % 路径解
        solution_cost           % 解的成本
        search_tree             % 搜索树
        num_expanded            % 扩展节点数

        % 冲突记录
        conflicts               % 记录所有冲突
    end

    methods
        function obj = CBS(env, varargin)
            % CBS 构造函数
            p = inputParser;
            addParameter(p, 'MaxIterations', 10000);
            addParameter(p, 'TimeLimit', 60);
            addParameter(p, 'NodeLimit', 100000);

            parse(p, varargin{:});

            obj.environment = env;
            obj.max_iterations = p.Results.MaxIterations;
            obj.time_limit = p.Results.TimeLimit;
            obj.node_limit = p.Results.NodeLimit;
            obj.solution = [];
            obj.solution_cost = 0;
            obj.search_tree = [];
            obj.num_expanded = 0;
            obj.conflicts = [];
        end

        function solution = findPath(obj, starts, goals, varargin)
            % findPath 为主智能体群规划路径
            %   solution = findPath(obj, starts, goals)
            %   starts: N x 3 起始位置矩阵
            %   goals: N x 3 目标位置矩阵
            %   solution: 结构体，包含每个智能体的路径

            p = inputParser;
            addParameter(p, 'Velocities', []);
            addParameter(p, 'StartTimes', []);
            parse(p, varargin{:});

            % 确保是 N x 3 矩阵
            if size(starts, 2) ~= 3
                starts = reshape(starts, 1, []);
            end
            if size(goals, 2) ~= 3
                goals = reshape(goals, 1, []);
            end

            n_agents = size(starts, 1);

            if isempty(p.Results.StartTimes)
                start_times = zeros(n_agents, 1);
            else
                start_times = p.Results.StartTimes;
            end

            % 初始化：每个智能体单独运行 A* 规划
            paths = cell(n_agents, 1);
            for i = 1:n_agents
                start_grid = obj.environment.positionToGrid(starts(i,:));
                goal_grid = obj.environment.positionToGrid(goals(i,:));
                paths{i} = obj.astar(start_grid, goal_grid);
            end

            % 合并约束树 (CT)
            root = obj.createNode(paths, [], start_times, []);

            % 开放列表 (优先级队列，按成本排序)
            open_list =containers.PriorityQueue;
            open_list.insert(root, root.cost);

            iteration = 0;
            start_time = tic;

            while ~open_list.isEmpty() && iteration < obj.max_iterations
                iteration = iteration + 1;

                % 检查时间限制
                if toc(start_time) > obj.time_limit
                    warning('CBS: Time limit reached');
                    break;
                end

                % 取出成本最低的节点
                node = open_list.pop();

                % 检测冲突
                conflicts = obj.detectConflicts(node.paths);

                if isempty(conflicts)
                    % 无冲突，找到解
                    obj.solution = node.paths;
                    obj.solution_cost = node.cost;
                    obj.conflicts = node.conflict_history;
                    solution = obj.formatSolution(node.paths, starts, goals);
                    return;
                end

                % 有冲突，创建子节点
                conflict = conflicts(1);  % 处理第一个冲突
                constraints = obj.createConstraintsFromConflict(conflict);

                % 为涉及的智能体创建新的约束节点
                for idx = 1:length(node.paths)
                    % 检查此智能体是否涉及冲突
                    if idx == conflict.agent1 || idx == conflict.agent2
                        new_constraints = [node.constraints, constraints{idx}];
                        new_paths = node.paths;

                        % 重新规划该智能体路径
                        start_grid = obj.environment.positionToGrid(starts(idx,:));
                        goal_grid = obj.environment.positionToGrid(goals(idx,:));
                        new_paths{idx} = obj.astar(start_grid, goal_grid, new_constraints);

                        % 检查新路径是否满足约束
                        if ~isempty(new_paths{idx})
                            % 计算新成本
                            new_cost = obj.calculateCost(new_paths);

                            % 创建新节点
                            new_node = obj.createNode(new_paths, new_constraints, ...
                                                     node.start_times, node);
                            new_node.cost = new_cost;

                            open_list.insert(new_node, new_cost);
                        end
                    end
                end

                obj.num_expanded = obj.num_expanded + 1;
            end

            % 无解
            solution = [];
            warning('CBS: No solution found');
        end

        function path = astar(obj, start, goal, constraints)
            % astar A*路径规划算法
            if nargin < 4
                constraints = [];
            end

            % 开放列表和关闭列表
            open_list = containers.PriorityQueue;
            closed_set = false(obj.environment.grid_size(1), ...
                             obj.environment.grid_size(2), ...
                             obj.environment.grid_size(3));

            % 创建起始节点
            start_node = struct();
            start_node.grid_idx = start;
            start_node.g_cost = 0;
            start_node.h_cost = obj.environment.heuristic(start, goal);
            start_node.f_cost = start_node.g_cost + start_node.h_cost;
            start_node.parent = [];
            start_node.time = 0;

            open_list.insert(start_node, start_node.f_cost);

            iteration = 0;
            max_iter = prod(obj.environment.grid_size) * 10;

            while ~open_list.isEmpty() && iteration < max_iter
                iteration = iteration + 1;

                current = open_list.pop();

                % 检查是否到达目标
                if isequal(current.grid_idx, goal)
                    path = obj.reconstructPath(current);
                    return;
                end

                % 标记为已访问
                if current.grid_idx(1) > 0 && current.grid_idx(1) <= obj.environment.grid_size(1) && ...
                   current.grid_idx(2) > 0 && current.grid_idx(2) <= obj.environment.grid_size(2) && ...
                   current.grid_idx(3) > 0 && current.grid_idx(3) <= obj.environment.grid_size(3)
                    closed_set(current.grid_idx(1), current.grid_idx(2), current.grid_idx(3)) = true;
                end

                % 遍历邻居
                neighbors = obj.environment.getNeighbors(current.grid_idx);
                for i = 1:size(neighbors, 1)
                    neighbor = neighbors(i,:);

                    % 检查是否在关闭列表
                    if closed_set(neighbor(1), neighbor(2), neighbor(3))
                        continue;
                    end

                    % 检查约束
                    if obj.isConstrained(neighbor, current.time + 1, constraints)
                        continue;
                    end

                    % 计算成本
                    tentative_g = current.g_cost + obj.environment.heuristic(current.grid_idx, neighbor);

                    % 创建邻居节点
                    neighbor_node = struct();
                    neighbor_node.grid_idx = neighbor;
                    neighbor_node.g_cost = tentative_g;
                    neighbor_node.h_cost = obj.environment.heuristic(neighbor, goal);
                    neighbor_node.f_cost = neighbor_node.g_cost + neighbor_node.h_cost;
                    neighbor_node.parent = current;
                    neighbor_node.time = current.time + 1;

                    open_list.insert(neighbor_node, neighbor_node.f_cost);
                end
            end

            % 未找到路径
            path = [];
        end

        function node = createNode(obj, paths, constraints, start_times, parent)
            % createNode 创建约束树节点
            node = struct();
            node.paths = paths;
            node.constraints = constraints;
            node.start_times = start_times;
            node.parent = parent;
            node.cost = obj.calculateCost(paths);
            node.conflict_history = [];
            if ~isempty(parent)
                node.conflict_history = parent.conflict_history;
            end
        end

        function conflicts = detectConflicts(obj, paths)
            % detectConflicts 检测路径间的冲突
            conflicts = [];
            n = length(paths);

            for i = 1:n
                if isempty(paths{i})
                    continue;
                end
                for j = i+1:n
                    if isempty(paths{j})
                        continue;
                    end

                    % 比较两个路径
                    max_len = max(length(paths{i}), length(paths{j}));
                    for t = 1:max_len
                        pos_i = paths{i}(min(t, end), :);
                        pos_j = paths{j}(min(t, end), :);

                        % 检查位置冲突 (同一点)
                        if isequal(pos_i, pos_j)
                            conflict.agent1 = i;
                            conflict.agent2 = j;
                            conflict.time = t;
                            conflict.type = 'vertex';
                            conflict.position = pos_i;
                            conflicts = [conflicts, conflict];
                        end

                        % 检查边冲突 (交叉经过)
                        if t > 1 && t <= length(paths{i}) && t <= length(paths{j})
                            prev_i = paths{i}(t-1, :);
                            prev_j = paths{j}(t-1, :);

                            if isequal(pos_i, prev_j) && isequal(pos_j, prev_i)
                                conflict.agent1 = i;
                                conflict.agent2 = j;
                                conflict.time = t;
                                conflict.type = 'edge';
                                conflict.position1 = prev_i;
                                conflict.position2 = pos_i;
                                conflicts = [conflicts, conflict];
                            end
                        end
                    end
                end
            end
        end

        function constraints = createConstraintsFromConflict(obj, conflict)
            % createConstraintsFromConflict 从冲突创建约束
            constraints = cell(1, max(conflict.agent1, conflict.agent2));

            % 为agent1添加约束: 不能在冲突时间占据冲突位置
            constraints{conflict.agent1} = struct();
            constraints{conflict.agent1}.type = 'vertex';
            constraints{conflict.agent1}.time = conflict.time;
            constraints{conflict.agent1}.position = conflict.position;

            % 为agent2添加约束
            constraints{conflict.agent2} = struct();
            constraints{conflict.agent2}.type = 'vertex';
            constraints{conflict.agent2}.time = conflict.time;
            constraints{conflict.agent2}.position = conflict.position;
        end

        function is_constrained = isConstrained(obj, grid_idx, time, constraints)
            % isConstrained 检查是否违反约束
            is_constrained = false;

            for i = 1:length(constraints)
                c = constraints(i);
                if c.time == time && isequal(grid_idx, c.position)
                    is_constrained = true;
                    return;
                end
            end
        end

        function cost = calculateCost(obj, paths)
            % calculateCost 计算路径总成本
            cost = 0;
            for i = 1:length(paths)
                if ~isempty(paths{i})
                    cost = cost + size(paths{i}, 1);
                end
            end
        end

        function path = reconstructPath(obj, node)
            % reconstructPath 重建路径
            path = node.grid_idx;
            current = node.parent;
            while ~isempty(current)
                path = [current.grid_idx; path];
                current = current.parent;
            end
        end

        function solution = formatSolution(obj, paths, starts, goals)
            % formatSolution 格式化输出解
            solution = struct();
            solution.paths = paths;
            solution.starts = starts;
            solution.goals = goals;
            solution.cost = obj.solution_cost;
            solution.num_agents = length(paths);
            solution.conflicts = obj.conflicts;

            % 转换为时间-位置序列
            solution.trajectories = cell(length(paths), 1);
            for i = 1:length(paths)
                if ~isempty(paths{i})
                    trajectory = obj.gridPathToTrajectory(paths{i}, i);
                    solution.trajectories{i} = trajectory;
                end
            end
        end

        function trajectory = gridPathToTrajectory(obj, grid_path, agent_id)
            % gridPathToTrajectory 将网格路径转换为轨迹
            n_steps = size(grid_path, 1);
            trajectory = struct();
            trajectory.agent_id = agent_id;
            trajectory.time = (0:n_steps-1)';
            trajectory.position = zeros(n_steps, 3);
            trajectory.velocity = zeros(n_steps, 3);

            for t = 1:n_steps
                trajectory.position(t,:) = obj.environment.gridToPosition(grid_path(t,:));
            end

            % 计算速度
            for t = 2:n_steps
                dt = 1;
                trajectory.velocity(t,:) = (trajectory.position(t,:) - ...
                                          trajectory.position(t-1,:)) / dt;
            end
        end

        function [positions, times] = getPlannedPath(obj, agent_id)
            % getPlannedPath 获取指定智能体的规划路径
            if isempty(obj.solution)
                positions = [];
                times = [];
                return;
            end

            if agent_id <= length(obj.solution.trajectories)
                traj = obj.solution.trajectories{agent_id};
                positions = traj.position;
                times = traj.time;
            else
                positions = [];
                times = [];
            end
        end
    end
end

