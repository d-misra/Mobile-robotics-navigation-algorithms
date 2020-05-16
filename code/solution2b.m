%Debaleena Misra - EMARO (174/EM)
%Solution to Project 2- Task (b) for Mobile Robotics Tutorials 
%Warsaw University of Technology

function [forwBackVel, leftRightVel, rotVel, finish] = solution2b(pts, contacts, position, orientation, varargin)
% the control loop callback function - the solution for task 2B 

  % get the goal point
  if length(varargin) ~= 1,
       error('Wrong number of additional arguments: %d\n', length(varargin));
  end
  % goal position in homogeneous coordinates
  goal = [varargin{1} 1];

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

  % current robot position in homogeneous coordinates
  pos = [position(1) position(2) 1];

  % static variables
  persistent mline; % representation of the line in homogeneous coordinates
  persistent prev_dist; % calculated distance when we last left mline
  persistent second_fourth; % is the line in the second/fourth quadrant?
  persistent counter; % threshold counter to wait for the bot to move

  % regulator constants
  persistent K_P; % Kp
  persistent WALL_DISTANCE; % distance to maintain from walls
  persistent MAX_LINEAR_VEL; % max linear velocity
  persistent MAX_ROT_VEL; % max rotational velocity
  % wall-follower constants
  persistent MAX_PERP_VEL; % max perpendicular velocity
  persistent MAX_PARA_VEL; % max parallel velocity

  % states of FSM
  if (strcmp(state, 'init')),

    % initialise constants
    K_P = 10;
    WALL_DISTANCE = 0.5;
    MAX_LINEAR_VEL = 5;
    MAX_ROT_VEL = 8;
    MAX_PERP_VEL = 1;
    MAX_PARA_VEL = 4;

    % calculate the distant to goal, and check if we're already there
    prev_dist = norm(goal - pos);
    if prev_dist <= 0.1,
      state = 'stop';
      fprintf('changing FSM state to %s\n', state);
    end

    % represent the line in homogeneous coordinates
    % where mline*(pos)' = 0
    % refer http://www.petercorke.com/RTB/r9/html/homline.html
    mline = homline(pos(1), pos(2), goal(1), goal(2));

    % which quadrant(s) does this line lie on?
    % this defines which way the bot goes upon encountering a wall
    % current angle w.r.t. goal point
    curr_angle = atan2(goal(2)-pos(2), goal(1)-pos(1));
    if (curr_angle >= pi/2 && curr_angle <= pi) || (curr_angle <= 0 && curr_angle >= -pi/2),
      % second and fourth quadrant
      second_fourth = true;
    else
      % first and third quadrant
      second_fourth = false;
    end

    % reset counter to zero
    % the counter helps ignore a certain number of calls to
    % the check, to prevent the function from getting locked
    % between two states
    counter = 0;

    % begin rotating towards the goal
    state = 'rotate';
    fprintf('changing FSM state to %s\n', state);

  elseif strcmp(state, 'rotate'),

    % preventing the [-2 -3] wall distance 0.5 bug
    % where if the path of wall hug and mline are coincident
    % the bot becomes confused on which state to be in
    % the solution is to ignore this state to make it a little cleaner
    if min_value-WALL_DISTANCE <= -0.005,
      % i.e., min_value < WALL_DISTANCE
      state = 'move_along_mline';
      fprintf('changing FSM state to %s\n', state);
    end

    error_rot = atan2(goal(1)-pos(1), pos(2)-goal(2)) - orientation(3);
    if (abs(error_rot) > pi),
      error_rot = orientation(3) - atan2(goal(1)-pos(1), pos(2)-goal(2));
    end
    % oriented correctly yet?
    if abs(error_rot) <= 0.05,
      state = 'move_along_mline';
      fprintf('changing FSM state to %s\n', state);
    end

    % rotational regulator
    rotVel = K_P * error_rot;
    if rotVel > MAX_ROT_VEL,
      rotVel = MAX_ROT_VEL;
    elseif rotVel < -MAX_ROT_VEL,
      rotVel = -MAX_ROT_VEL;
    end

  elseif strcmp(state, 'move_along_mline'),

    % are we there yet?
    if norm(goal - pos) <= 0.1,
      state = 'stop';
      fprintf('changing FSM state to %s\n', state);
    end

    % are we colliding with a wall?
    if counter ~= 20,
      counter = counter + 1;
    end
    if min_value <= WALL_DISTANCE && counter == 20,
      counter = 0; % reset counter
      prev_dist = norm(goal - pos);
      state = 'move_along_wall';
      fprintf('changing FSM state to %s\n', state);
    end

    % rotational regulator
    error_rot = atan2(goal(1)-pos(1), pos(2)-goal(2)) - orientation(3);
    if (abs(error_rot) > pi),
      error_rot = orientation(3) - atan2(goal(1)-pos(1), pos(2)-goal(2));
    end
    rotVel = K_P * error_rot;
    if rotVel > MAX_ROT_VEL,
      rotVel = MAX_ROT_VEL;
    elseif rotVel < -MAX_ROT_VEL,
      rotVel = -MAX_ROT_VEL;
    end

    % linear regulator
    rho_leftRight = K_P * (goal(1)-pos(1)) + K_P * (0-mline*pos'); % speed up to move closer
    rho_forwBack  = K_P * (goal(2)-pos(2)) - K_P * (0-mline*pos'); % slow down to move closer

    % limit the output of the regulator with polar vectors
    [phi rho] = cart2pol(rho_leftRight, rho_forwBack);
    if rho > MAX_LINEAR_VEL,
      rho = MAX_LINEAR_VEL;
    end
    [rho_leftRight rho_forwBack] = pol2cart(phi, rho);

    % convert to local reference frame
    grf_vel = [rho_leftRight rho_forwBack]';
    theta = orientation(3);
    rot_matrix = [cos(theta) sin(theta); -sin(theta) cos(theta)];
    lrf_vel = rot_matrix * grf_vel;
    leftRightVel = lrf_vel(1);
    forwBackVel  = lrf_vel(2);

  elseif strcmp(state, 'move_along_wall'),

    % have we encountered mline?
    if abs(mline * pos') <= 0.05,
      if counter ~= 100,
        counter = counter + 1;
      end
      if abs( norm(goal-pos) - prev_dist ) <= 0.05 && counter == 100,
        % we've come back to where we were previously
        % we must be stuck
        error('Stuck! :(');
      elseif norm(goal-pos) - prev_dist <= -0.05,
        % i.e., norm(goal-pos) < prev_dist
        % (with a threshold, don't think we need it though;
        % the order of the if/else block should be enough)
        % depart for goal
        counter = 0; % reset counter;
        state = 'rotate';
        fprintf('changing FSM state to %s\n', state);
      end
    end

    % angle of the minimum distance contact point with the wall
    % relative to the bot
    theta = atan2(points(min_index, 1), -points(min_index, 2));

    % rotational regulator
    rotVel = K_P * theta;
    if rotVel > MAX_ROT_VEL,
      rotVel = MAX_ROT_VEL;
    elseif rotVel < -MAX_ROT_VEL;
      rotVel = -MAX_ROT_VEL;
    end

    % perpendicular regulator
    rho_perp = K_P * (WALL_DISTANCE-min_value);
    if rho_perp > MAX_PERP_VEL,
      rho_perp = MAX_PERP_VEL;
    elseif rho_perp < -MAX_PERP_VEL,
      rho_perp = -MAX_PERP_VEL;
    end

    % parallel regulator
    if second_fourth,
      % line is in the second and fourth quadrant
      [min_value_left min_index_left] = min(distances(1:min_index-1));
      if min_value_left > min_value,
        rho_para = MAX_PARA_VEL;
      else
        rho_para = 0;
      end
    else
      % line is in the first and third quadrant
      [min_value_right min_index_right] = min(distances(min_index+1:size(distances, 1)));
      if min_value_right > min_value,
        rho_para = -MAX_PARA_VEL;
      else
        rho_para = 0;
      end
    end

    % convert between a rotated frame (a frame where the bot is presumed
    % to be oriented correctly, which our calculations are in)
    % to the real frame (where the bot has some orientation error (theta))
    rotated_frame = [rho_para rho_perp];
    rot_matrix = [cos(theta) sin(theta); -sin(theta) cos(theta)];
    real_frame = rotated_frame * rot_matrix;

    leftRightVel = real_frame(1);
    forwBackVel  = real_frame(2);

  elseif strcmp(state, 'stop')
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