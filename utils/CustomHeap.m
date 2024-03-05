%% Custom min-heap implementation
% keys can be an array, sorted by the first element

classdef CustomHeap < handle
    properties (Access = protected)
        k; % current number of elements
        n; % capacity
        x; % heap matrix (each row is an element)
        c; % number of elements in a key
    end
    methods (Access = public)
        %% Constructor
        function this = CustomHeap(n, c, x0)
            arguments
                n double {mustBePositive}
                c double {mustBePositive}
                x0 double = [] 
            end

            this.n = n;
            this.c = c;
            this.x = nan(n, c);
            
            if ((nargin == 3) && ~isempty(x0))
                % Insert given elements
                k0 = size(x0, 1);
                if (k0 > n)
                    % Heap overflow
                    CustomHeap.OverflowError();
                else
                    this.x(1:k0, :) = x0;
                    this.setLength(k0);
                end
            else
                % Empty heap
                this.clear();
            end
            % Build the heap
            for i = floor(this.k / 2):-1:1
                this.minHeapify(i);
            end
        end

        %% Insert key
        function push(this, key)
            this.setLength(this.k + 1);
            this.x(this.k, 1) = inf;
            this.decreaseKey(this.k, key);
        end

        %% Peek
        function min = peek(this)
            if this.isEmpty()
                min = [];
            else
                min = this.x(1, :);
            end
        end
        
        %% Pop
        function min = pop(this)
            this.setLength(this.k - 1);
            min = this.x(1, :);
            this.x(1, :) = this.x(this.k + 1, :);
            this.minHeapify(1);
        end

         %% Return the sorted heap (ascending)
        function sx = sort(this)
            nk = this.k; % virtual heap size during sorting procedure
            for i = this.k:-1:2
                this.swap(1,i);
                nk = nk - 1;
                this.minHeapify(1,nk);
            end
            this.x(1:this.k, :) = flipud(this.x(1:this.k, :));
            sx = this.x(1:this.k, :);
        end
        

        %% Get count
        function k = count(this)
            k = this.k;
        end
        
        %% Get capacity
        function capacity = capacity(this)
            capacity = this.n;
        end
        
        %% Check if the heap is empty
        function bool = isEmpty(this)
            if (this.k == 0)
                bool = true;
            else
                bool = false;
            end
        end
        
        %% Check if the heap is full
        function bool = isFull(this)
            if (this.k == this.n)
                bool = true;
            else
                bool = false;
            end
        end
        
        %% Clear the heap
        function clear(this)
            this.setLength(0);
        end
    end

    %% Private methods
    methods (Access = private)
        % Maintain the min heap property at a given node
        function minHeapify(this,i,size)
            % Parse inputs
            if (nargin < 3)
                size = this.k;
            end
            
            ll = CustomHeap.left(i);
            rr = CustomHeap.right(i);
            if ((ll <= size) && (this.x(ll, 1) < this.x(i, 1)))
                smallest = ll;
            else
                smallest = i;
            end
            if ((rr <= size) && (this.x(rr, 1) < this.x(smallest, 1)))
                smallest = rr;
            end
            if (smallest ~= i)
                this.swap(i,smallest);
                this.minHeapify(smallest,size);
            end
        end

        % Decrease key 
        function decreaseKey(this,i,key)
            if (i > this.k)
                % Index overflow error
                CustomHeap.IndexOverflowError();
            elseif (key(1) > this.x(i, 1))
                % Decrease key error
                CustomHeap.DecreaseKeyError();
            end
            this.x(i, :) = key;
            while ((i > 1) && (this.x(CustomHeap.parent(i), 1) > this.x(i, 1)))
                this.swap(i,CustomHeap.parent(i));
                i = CustomHeap.parent(i);
            end
        end
    end

    %% Protected util
    methods (Access = protected)
        % Swap elements
        function swap(this,i,j)
            val = this.x(i, :);
            this.x(i, :) = this.x(j, :);
            this.x(j, :) = val;
        end
        
        % Set new length
        function setLength(this,k)
            if (k < 0)
                CustomHeap.UnderflowError();
            elseif (k > this.n)
                CustomHeap.OverflowError();
            end
            this.k = k;
        end
    end
   
    %% Static util
    methods (Access = protected, Static = true)
        % Parent node
        function p = parent(i)
            p = floor(i / 2);
        end 

        % Left child node
        function l = left(i)
            l = 2 * i;
        end
        
        % Right child node
        function r = right(i)
            r = 2 * i + 1;
        end
        
        % Overflow error
        function OverflowError()
            error('Heap overflow');
        end
        
        % Underflow error
        function UnderflowError()
            error('Heap underflow');
        end

        % Decrease key error
        function DecreaseKeyError()
            error('You can only decrease keys in MinHeap');
        end
        
        % Index overflow error
        function IndexOverflowError()
            error('MinHeap index overflow');
        end
    end
end