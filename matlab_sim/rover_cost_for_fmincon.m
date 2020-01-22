function cost = rover_cost_for_fmincon(k,z_goal_local)

% evaluate cost and gradient
    cost = rover_cost(k(1),k(2),z_goal_local(1),z_goal_local(2)) ;
    
    % perform timeout check
    if nargin > 3 && toc(start_tic) > timeout
        error('Timed out while evaluating cost function!')
    end

end