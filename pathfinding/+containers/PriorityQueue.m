% 优先级队列实现类
classdef PriorityQueue < handle
    properties
        items
        priorities
    end

    methods
        function obj = PriorityQueue()
            obj.items = {};
            obj.priorities = [];
        end

        function insert(obj, item, priority)
            obj.items{end+1} = item;
            obj.priorities(end+1) = priority;
            % 重新排序
            [obj.priorities, idx] = sort(obj.priorities);
            obj.items = obj.items(idx);
        end

        function item = pop(obj)
            if isempty(obj.items)
                item = [];
                return;
            end
            item = obj.items{1};
            obj.items(1) = [];
            obj.priorities(1) = [];
        end

        function is_empty = isEmpty(obj)
            is_empty = isempty(obj.items);
        end

        function len = length(obj)
            len = length(obj.items);
        end
    end
end