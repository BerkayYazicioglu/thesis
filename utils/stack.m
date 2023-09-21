classdef stack < handle
    %% Stack implementation 
    % data: (optional) cell array of values to initialize the stack 
    % stack.size(): return the current size
    % stack.flush(): remove all data in the stack and return it
    % stack.isempty(): return true if the stack is empty
    % stack.push(data, idx (optional)): push a new data to stack
    % stack.pop(idx (optional)): pop the top value and return it
    % stack.peek(idx (optional)): return the top value without popping it
    % stack.replace(data, idx): replace the value at idx with data
    % stack.content(linearize (optional)): peek all current content in the stack

    properties
        data cell = {}; 
    end
    
    methods
        function self = stack(data)
            if nargin > 0
                for i = 1:length(data)
                    self.push(data{i});
                end
            end
        end
        
        function s = size(self)
            s = length(self.data);
        end
        
        function data = flush(self)
            data = self.data;
            self.data = {};
        end
        
        function b = isempty(self)            
            b = isempty(self.data);
        end
        
        function replace(self, new, idx)
            mustBeInRange(idx, 1, self.size());
            self.data{idx} = new;
        end

        function push(self, new, idx)
            if nargin < 3
                self.data{end+1} = new;
            else
                mustBeInRange(idx, 1, self.size());
                if idx == self.size()
                    self.push(new);
                elseif idx == 1
                    self.data = [new self.data];
                else
                    self.data = [self.data(1:idx-1) new self.data(idx:end)];
                end
            end
        end
        
        function value = pop(self, idx)
            if self.isempty()
                value = [];
            else
                if nargin < 2
                   value = self.data{end};
                   self.data = self.data(1:end-1);
                else
                    mustBeInRange(idx, 1, self.size());
                    value = self.data{idx};
                    self.data{idx} = [];
                    self.data = self.data(~cellfun('isempty', self.data));
                end
            end
        end
        
        function value = peek(self, idx)
            if self.isempty()
                value = [];
            else
               if nargin < 2
                   value = self.data{end};
                else
                    mustBeInRange(idx, 1, self.size());
                    value = self.data{idx};
                end
            end 
        end
    
        function data = content(self, linearize)
            data = self.data;
            if nargin == 2
                if linearize
                    data = cell2mat(data);
                end
            end
        end
    end
end