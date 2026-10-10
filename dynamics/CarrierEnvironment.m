classdef CarrierEnvironment < handle
    %CarrierEnvironment 舰载环境模型类
    %   定义舰载机的运行环境，包括航母甲板、停机位、起飞/降落区等

    properties
        % 环境范围 (米)
        environment_bounds

        % 甲板参数
        deck_length              % 甲板长度
        deck_width               % 甲板宽度
        deck_origin              % 甲板原点坐标

        % 功能区域
        takeoff_zone             % 起飞区 [x1, y1, x2, y2]
        landing_zone             % 降落区 [x1, y1, x2, y2]
        parking_spots            % 停机位坐标列表 (N x 3)
        taxiway                  % 滑行通道

        % 网格参数 (用于路径规划)
        grid_resolution          % 网格分辨率 (米)
        grid_size                % 网格大小 [nx, ny, nz]
        occupancy_grid           % 占用网格

        % 时间参数
        current_time             % 当前仿真时间 (秒)

        % 障碍物
        obstacles                % 障碍物列表

        % 天气条件
        wind_direction           % 风向 (弧度)
        wind_speed               % 风速 (m/s)
    end

    methods
        function obj = CarrierEnvironment(varargin)
            % CarrierEnvironment 构造函数
            p = inputParser;
            addParameter(p, 'DeckLength', 300);
            addParameter(p, 'DeckWidth', 80);
            addParameter(p, 'GridResolution', 10);
            addParameter(p, 'MaxAltitude', 500);
            addParameter(p, 'WindDirection', 0);
            addParameter(p, 'WindSpeed', 10);

            parse(p, varargin{:});

            obj.deck_length = p.Results.DeckLength;
            obj.deck_width = p.Results.DeckWidth;
            obj.deck_origin = [0, 0, 0];
            obj.grid_resolution = p.Results.GridResolution;

            % 定义功能区域
            obj.takeoff_zone = [0, 20, 100, 60];      % 起飞区
            obj.landing_zone = [200, 20, 300, 60];    % 降落区

            % 创建停机位
            obj.parking_spots = obj.createParkingSpots();

            % 创建滑行通道
            obj.taxiway = obj.createTaxiway();

            % 初始化网格
            nx = ceil(obj.deck_length / obj.grid_resolution);
            ny = ceil(obj.deck_width / obj.grid_resolution);
            nz = ceil(p.Results.MaxAltitude / obj.grid_resolution);
            obj.grid_size = [nx, ny, nz];
            obj.occupancy_grid = false(nx, ny, nz);

            obj.current_time = 0;
            obj.obstacles = [];
            obj.wind_direction = p.Results.WindDirection;
            obj.wind_speed = p.Results.WindSpeed;
            obj.environment_bounds = [0, obj.deck_length, 0, obj.deck_width, 0, p.Results.MaxAltitude];
        end

        function spots = createParkingSpots(obj)
            % createParkingSpots 创建停机位
            spots = [];
            spot_idx = 1;
            % 在甲板两侧创建停机位
            for x = 50:30:250
                % 左侧停机位
                spots(spot_idx, :) = [x, 10, 0];
                spot_idx = spot_idx + 1;
                % 右侧停机位
                spots(spot_idx, :) = [x, 70, 0];
                spot_idx = spot_idx + 1;
            end
        end

        function taxi = createTaxiway(obj)
            % createTaxiway 创建滑行通道
            % 简化为网格点列表
            taxi = [];
            % 中心滑行通道
            for x = 20:20:280
                taxi = [taxi; x, 40, 0];  % 中心线
            end
        end

        function grid_idx = positionToGrid(obj, position)
            % positionToGrid 将世界坐标转换为网格索引
            x = floor(position(1) / obj.grid_resolution) + 1;
            y = floor(position(2) / obj.grid_resolution) + 1;
            z = floor(position(3) / obj.grid_resolution) + 1;

            x = max(1, min(obj.grid_size(1), x));
            y = max(1, min(obj.grid_size(2), y));
            z = max(1, min(obj.grid_size(3), z));

            grid_idx = [x, y, z];
        end

        function world_pos = gridToPosition(obj, grid_idx)
            % gridToPosition 将网格索引转换为世界坐标
            x = (grid_idx(1) - 0.5) * obj.grid_resolution;
            y = (grid_idx(2) - 0.5) * obj.grid_resolution;
            z = (grid_idx(3) - 0.5) * obj.grid_resolution;
            world_pos = [x, y, z];
        end

        function is_valid = isValidPosition(obj, position)
            % isValidPosition 检查位置是否有效
            bounds = obj.environment_bounds;
            is_valid = position(1) >= bounds(1) && position(1) <= bounds(2) && ...
                       position(2) >= bounds(3) && position(2) <= bounds(4) && ...
                       position(3) >= bounds(5) && position(3) <= bounds(6);

            % 检查是否在障碍物内
            for i = 1:length(obj.obstacles)
                if obj.isInsideObstacle(position, obj.obstacles(i))
                    is_valid = false;
                    return;
                end
            end
        end

        function is_inside = isInsideObstacle(obj, pos, obstacle)
            % isInsideObstacle 检查位置是否在障碍物内
            if isfield(obstacle, 'type')
                switch obstacle.type
                    case 'sphere'
                        is_inside = norm(pos - obstacle.center) <= obstacle.radius;
                    case 'box'
                        is_inside = all(pos >= obstacle.min) && all(pos <= obstacle.max);
                    case 'cylinder'
                        d = norm(pos(1:2) - obstacle.center(1:2));
                        is_inside = d <= obstacle.radius && ...
                                   pos(3) >= obstacle.center(3) && ...
                                   pos(3) <= obstacle.center(3) + obstacle.height;
                    otherwise
                        is_inside = false;
                end
            else
                is_inside = false;
            end
        end

        function addObstacle(obj, obstacle)
            % addObstacle 添加障碍物
            obj.obstacles{end+1} = obstacle;
        end

        function markOccupied(obj, position, value)
            % markOccupied 标记/取消标记网格占用
            grid_idx = obj.positionToGrid(position);
            if all(grid_idx >= 1) && all(grid_idx <= obj.grid_size)
                obj.occupancy_grid(grid_idx(1), grid_idx(2), grid_idx(3)) = value;
            end
        end

        function is_occupied = checkOccupied(obj, position)
            % checkOccupied 检查网格是否被占用
            grid_idx = obj.positionToGrid(position);
            if all(grid_idx >= 1) && all(grid_idx <= obj.grid_size)
                is_occupied = obj.occupancy_grid(grid_idx(1), grid_idx(2), grid_idx(3));
            else
                is_occupied = false;
            end
        end

        function neighbors = getNeighbors(obj, grid_idx)
            % getNeighbors 获取网格邻居 (26连通)
            neighbors = [];
            dx = [-1, 0, 1];
            dy = [-1, 0, 1];
            dz = [-1, 0, 1];

            for i = 1:length(dx)
                for j = 1:length(dy)
                    for k = 1:length(dz)
                        if dx(i) == 0 && dy(j) == 0 && dz(k) == 0
                            continue;
                        end
                        new_idx = grid_idx + [dx(i), dy(j), dz(k)];
                        if all(new_idx >= 1) && all(new_idx <= obj.grid_size)
                            neighbors = [neighbors; new_idx];
                        end
                    end
                end
            end
        end

        function cost = heuristic(obj, grid_idx1, grid_idx2)
            % heuristic 启发函数 (欧几里得距离)
            cost = norm((grid_idx1 - grid_idx2) * obj.grid_resolution);
        end

        function advanceTime(obj, dt)
            % advanceTime 推进仿真时间
            obj.current_time = obj.current_time + dt;
        end

        function spots = getAvailableParkingSpots(obj)
            % getAvailableParkingSpots 获取可用停机位
            spots = obj.parking_spots;
        end

        function zones = getZones(obj)
            % getZones 获取所有功能区域
            zones.takeoff = obj.takeoff_zone;
            zones.landing = obj.landing_zone;
            zones.parking = obj.parking_spots;
            zones.taxiway = obj.taxiway;
        end
    end
end