%% Kinematic Simulation function
%
% Inputs
% - q current robot configuration
% - q_dot joints velocity
% - dt sample time
% - q_min lower joints bound
% - q_max upper joints bound
%
% Outputs
% - q new joint configuration

function [q] = KinematicSimulation(q, q_dot, dt, q_min, q_max)
    % updating q
    q = q + q_dot*dt;
    
    % saturating
    for i = 1: length(q)
        % upper bound
        if (q(i) > q_max(i))
            q(i)= q_max(i);
           
        % lower bound
        elseif (q(i) < q_min(i))
            q(i)= q_min(i);

        end
    end
end