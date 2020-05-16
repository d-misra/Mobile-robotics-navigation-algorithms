%This script was prepared by Debaleena Misra - EMARO (174/EM)
%Solution to Project 1- Task (b) for Mobile Robotics Tutorials 
%Warsaw University of Technology

function [forwBackVel, leftRightVel, rotVel, finish] = solution1b(pts, contacts, position, orientation, varargin)
% The control loop callback function - the solution for task 1B

    % get the destination point
    if length(varargin) ~= 1,
         error('Wrong number of additional arguments: %d\n', length(varargin));
    end
    radius = abs(varargin{1});

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

    persistent centre;
    persistent tan_rho; %Tangential velocity
    persistent k_p;
    persistent k_pz;
    persistent init_phi;

    % manage the states of FSM
    if strcmp(state, 'init'),
        centre = [position(1), position(2)-radius];
        k_p = 10;
        k_pz = 10;
        % using init_phi to maintain initial orientation
        init_phi = orientation(3);
        % initialise the magnitude of tangential velocity
        tan_rho = 1 * radius;
        if tan_rho > 1,
            tan_rho = 1;
        end
        state = 'circle_left';
        fprintf('changing FSM state to %s\n', state);
    elseif strcmp(state, 'circle_left'),
        % current orientation angle
        theta = orientation(3);
        % convert current position from Cartesian to polar
        % we must use the centre of the circle as the reference point
        [phi, rho] = cart2pol(position(1), position(2)+radius);

        % tangential velocity vector angle
        % tan_rho is persistent and has been initialised in 'init'
        tan_phi = wrapTo2Pi(phi + pi/2);

        % radius error (Euclidean distance)
        error_radius = sqrt( (position(1)-centre(1))^2 + (position(2)-centre(2))^2 ) - radius;

        % parallel (perhaps centripetal?) velocity
        p_phi = wrapTo2Pi(phi + pi);
        p_rho = k_p * error_radius;
        if p_rho > 1,
            p_rho = 1;
        elseif p_rho < -1,
            p_rho = -1;
        end

        % the final velocity vector in the global reference frame
        % is the sum of the two vectors
        % x-velocity magnitude in the global reference frame
        grf_leftRightVel = tan_rho*cos(tan_phi) + p_rho*cos(p_phi);
        % y-velocity magnitude in the global reference frame
        grf_forwBackVel = tan_rho*sin(tan_phi) + p_rho*sin(p_phi);

        % convert to local reference frame
        grf_vel = [grf_leftRightVel grf_forwBackVel]';
        rot_matrix = [cos(theta) sin(theta); -sin(theta) cos(theta)];
        lrf_vel = rot_matrix * grf_vel;

        leftRightVel = lrf_vel(1);
        forwBackVel = lrf_vel(2);

        % rotational velocity regulator
        error_phi = init_phi - orientation(3);
        rotVel = k_pz * error_phi;
        if rotVel > 0.5,
            rotVel = 0.5;
        elseif rotVel < -0.5,
            rotVel = -0.5;
        end
    else
        error('Unknown state %s.\n', state);
    end
end