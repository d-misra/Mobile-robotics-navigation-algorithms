%This script was prepared by Debaleena Misra - EMARO (174/EM)
%Solution to Project 1- Task (a) for Mobile Robotics Tutorials 
%Warsaw University of Technology

function [forwBackVel, leftRightVel, rotVel, finish] = solution1a(pts, contacts, position, orientation, varargin)
% The control loop callback function - the solution for task 1A

    % get the destination point
    if length(varargin) ~= 3,
         error('Wrong number of additional arguments: %d\n', length(varargin));
    end
    dest_x = varargin{1};
    dest_y = varargin{2};
    dest_fi = varargin{3};

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

    % manage the states of FSM
    if strcmp(state, 'init'),
        state = 'move_to_destination';
        fprintf('changing FSM state to %s\n', state);
    elseif strcmp(state, 'move_to_destination'),
        flag_error = false;
        error_x = dest_x - position(1);
        error_y = dest_y - position(2);
        error_fi = dest_fi - orientation(3);
        theta = orientation(3);

        
        grf_forwBackVel = 10 * error_y;
        grf_leftRightVel = 10 * error_x;

        if grf_forwBackVel > 1,
            flag_error = true;
            grf_forwBackVel = 1;
        elseif grf_forwBackVel < -1,
            flag_error = true;
            grf_forwBackVel = -1;
        end

        if grf_leftRightVel > 1,
            flag_error = true;
            grf_leftRightVel = 1;
        elseif grf_leftRightVel < -1,
            flag_error = true;
            grf_leftRightVel = -1;
        end

        %Maintaining the error ratios so that straight line is ensured 
        if flag_error,
            if abs(error_x) > abs(error_y),
                grf_forwBackVel = grf_forwBackVel * abs(error_y / error_x);
            else abs(error_y) > abs(error_x),
                grf_leftRightVel = grf_leftRightVel * abs(error_x / error_y);
            end
        end

        grf_rotVel = 10 * error_fi;
        if grf_rotVel > 0.5,
            grf_rotVel = 0.5;
        end
        if grf_rotVel < -0.5,
            grf_rotVel = -0.5;
        end

        %Transformation to robot local frame
        grf_vel = [grf_leftRightVel grf_forwBackVel grf_rotVel]';
        rot_matrix = [cos(theta) sin(theta) 0; -sin(theta) cos(theta) 0; 0 0 1];
        lrf_vel = rot_matrix * grf_vel;

        leftRightVel = lrf_vel(1);
        forwBackVel = lrf_vel(2);
        rotVel = lrf_vel(3);

        if abs(error_x) < 0.01 && abs(error_y) < 0.01 && abs(rad2deg(error_fi)) < 0.01,
            state = 'stop';
            fprintf('changing FSM state to %s\n', state);
        end
    elseif strcmp(state, 'stop'),
        forwBackVel = 0;
        leftRightVel = 0;
        rotVel = 0;
        state = 'finish';
        fprintf('changing FSM state to %s\n', state);
    elseif strcmp(state, 'finish'),
        finish = true;
        disp('finished');
    else
        error('Unknown state %s.\n', state);
    end
end