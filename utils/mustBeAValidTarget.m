function mustBeAValidTarget(target, self, check_step)
    n = neighbors(self.world.environment, self.node);
    n(end+1) = self.node;
    if ismember(target, n)
        if check_step
            step = abs(self.world.environment.Nodes.terrain(self.node) - ...
                       self.world.environment.Nodes.terrain(target));
            if step > self.max_step 
                error('The edge is too steep, check the path planner');
            end
        end
    else
        error('Target is not a neighbor of given node');
    end
end