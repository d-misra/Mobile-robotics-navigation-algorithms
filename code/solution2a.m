%Debaleena Misra - EMARO (174/EM)
%Solution to Project 2- Task (a) for Mobile Robotics Tutorials 
%Warsaw University of Technology

function [forwBackVel, leftRightVel, rotVel, finish] = solution2a(pts, contacts, position, orientation)
% The control loop callback function - the example solution for task 2A

    % declare the persistent variable that keeps the state of the Finite
    % State Machine (FSM)
    persistent state;
    if isempty(state),
        % the initial state of the FSM is 'init'
        state = 'init';
    end

    % initialize the robot control variables (returned by this function)
    finish = false;
    forwBackVel = 0;
    leftRightVel = 0;
    rotVel = 0;

    % get the laser contact points in sensor's coordinates
    points = [pts(1,contacts)' pts(2,contacts)'];
    % calculate the distances
    distances = (pts(1,contacts)'.^2 + pts(2,contacts)'.^2).^0.5;
    % get the closest point
    [min_value, min_index] = min(distances(:));

    persistent k_perp;
    persistent k_rot;
    persistent max_perp_vel;
    persistent max_para_vel;
    persistent max_rot_vel;

    % manage the states of FSM
    if strcmp(state, 'init'),
        k_perp = 10;
        k_rot = 10;
        max_perp_vel = 1;
        max_para_vel = 2;
        max_rot_vel = 1;
        state = 'move_to_wall';
        fprintf('changing FSM state to %s\n', state);
       
    elseif strcmp(state, 'move_to_wall'),
        error_perp = 1 - min_value;
        forwBackVel = k_perp * error_perp;
        if forwBackVel > max_perp_vel,
            forwBackVel = max_perp_vel;
        elseif forwBackVel < -max_perp_vel,
            forwBackVel = -max_perp_vel;
        end
        
        if abs(error_perp) <= 0.01,
            state = 'move';
            fprintf('changing FSM state to %s\n', state);
        end
    elseif strcmp(state, 'move')
        % rotational regulator
        theta = atan2(points(min_index,1),-points(min_index,2));
        rotVel = k_rot * theta;
        if rotVel > max_rot_vel,
           rotVel = max_rot_vel;
        elseif rotVel < -max_rot_vel,
           rotVel = -max_rot_vel;   
        end
        
        % perpendicular regulator
        rho_perp = k_perp * (1-min_value);
        if rho_perp > max_perp_vel,
            rho_perp = max_perp_vel;
        elseif rho_perp < -max_perp_vel,
            rho_perp = -max_perp_vel;
        end

        % parallel regulator
        % minimum distance to the left of min_value (every ray to the left)
        [min_value_left min_index_left] = min(distances(1:min_index-1));
        if (min_value_left > min_value),
            rho_para = max_para_vel;
        else
            rho_para = 0;
        end

        %Taking components in the local reference frame
         leftRightVel = (cos(theta)*rho_para)-(sin(theta)*rho_perp);
         forwBackVel = (sin(theta)*rho_para)+(cos(theta)*rho_perp) ;
         
        % convert between a rotated frame (a frame where the bot is presumed
        % to be oriented correctly, which our calculations are in)
        % to the real frame (where the bot has some orientation error (theta))
%         rotated_frame = [rho_para rho_perp ];
%         rot_matrix = [cos(theta) sin(theta); -sin(theta) cos(theta)];
%         real_frame = rotated_frame * rot_matrix;
% 
%         forwBackVel = real_frame(2);
%         leftRightVel = real_frame(1);
                
    else
        error('Unknown state %s.\n', state);
    end
end
