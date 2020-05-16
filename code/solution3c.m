function [forwBackVel, leftRightVel, rotVel, finish] = solution3c(pts, contacts, position, orientation, varargin)
% the control loop callback function - the solution for task 3C

  % get the goal point
  if length(varargin) ~= 1,
    error('Wrong number of additional arguments: %d\n', length(varargin));
  end
  % goal position
  goal = varargin{1};

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

  % current position
  pos = [position(1) position(2)];

  % static
  persistent K_P;        % Kp
  persistent MAX_LINVEL; % maximum linear velocity
  persistent MAX_ROTVEL; % maximum rotational velocity

  persistent pathvec;    % path vector trajectory to follow
  persistent index;      % current point index in pathvec

  % states of FSM
  if (strcmp(state, 'init')),

    % set constants
    K_P = 10;
    MAX_LINVEL = 1;
    MAX_ROTVEL = 1;

    % read the map image
    map = imread('vrep_env/exercise02.png');
    % inflate obstacles
    inflated_map = imerode(map, strel('square', 7));
    [sizey sizex] = size(map);
    % convert to map coordinates
    goal_x_map = ceil(sizex*((goal(1)+7.5)/15));
    goal_y_map = ceil(sizey*((goal(2)+7.5)/15));
    % is this on top of an obstacle?
    if inflated_map(goal_y_map,goal_x_map) == 0,
      error('Goal is unreachable.');
    end

    % wavefill
    wavemap = wavefill(inflated_map, goal_y_map, goal_x_map);

    % generate wavefront path
    % note pathvec_map is indexed as (x,y)
    pathvec_map = wavefrontPath(wavemap, pos, goal);

    % convert to world coordinates
    % note pathvec is indexed as (x,y)
    % pathvec is a kx2 vector that can directly be used
    % for trajectory planning
    pathvec(:,1) = (pathvec_map(:,2)./sizey).*15-7.5-(7.5/sizey);
    pathvec(:,2) = (pathvec_map(:,1)./sizex).*15-7.5-(7.5/sizex);

    % BEGIN PLOT

    % generate a wavefill map for the original (uninflated) map
    map_path = wavefill(map, goal_y_map, goal_x_map);
    map_path = double(map_path)*.15;
    imagesc([-7.5 7.5], [-7.5 7.5], map_path);
    c = colorbar;
    c.Label.String = 'Distance (meters)';
    set(gca,'ydir','normal'); 
    hold on;

    % start-goal line
    line([pos(1),goal(1)], [pos(2),goal(2)], 'color','green');

    % pathvec
    line(pathvec(:,1), pathvec(:,2), 'color','red');
    for i = 1:size(pathvec, 1),
      plot( pathvec(i,1), pathvec(i,2), ['*','yellow'] );
    end

    % start and goal points
    plot( pos(1),  pos(2),  ['*','red']   );
    plot( goal(1), goal(2), ['*','green'] );

    % END PLOT

    % use the trajectory defined by pathvec
    index = 1;
    state = 'execute_path';
    fprintf('changing FSM state to %s\n', state);

  elseif (strcmp(state, 'execute_path')),

    % linear errors
    error_x = pathvec(index,1) - pos(1);
    error_y = pathvec(index,2) - pos(2);

    % are we at the next point?
    if abs(error_x) <= 0.01 && abs(error_y) <= 0.01,
      % is it the goal?
      if abs(pathvec(index,1)-goal(1)) <= 0.1 && abs(pathvec(index,2)-goal(2)) <= 0.1,
        state = 'stop';
        fprintf('changing FSM state to %s\n', state);
      end

      % increment index otherwise
      index = index + 1;
    end

    % current orientation angle
    theta = orientation(3);

    % rotational regulator
    error_rot = atan2(pathvec(index,1)-pos(1), pos(2)-pathvec(index,2)) - orientation(3);
    if (abs(error_rot) > pi),
      error_rot = orientation(3) - atan2(pathvec(index,1)-pos(1), pos(2)-pathvec(index,2));
    end
    rotVel = K_P * error_rot;
    if rotVel > MAX_ROTVEL,
      rotVel = MAX_ROTVEL;
    elseif rotVel < -MAX_ROTVEL,
      rotVel = -MAX_ROTVEL;
    end

    % linear regulator
    rho_leftRight = K_P * error_x;
    rho_forwBack = K_P * error_y;
    % limit the output of the regulator with polar vectors
    [phi rho] = cart2pol(rho_leftRight, rho_forwBack);
    if rho > MAX_LINVEL,
      rho = MAX_LINVEL;
    end
    [rho_leftRight rho_forwBack] = pol2cart(phi, rho);

    % convert to local reference frame
    grf_vel = [rho_leftRight rho_forwBack]';
    rot_matrix = [cos(theta) sin(theta); -sin(theta) cos(theta)];
    lrf_vel = rot_matrix * grf_vel;
    leftRightVel = lrf_vel(1);
    forwBackVel  = lrf_vel(2);

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
end % solution3c

function map = wavefill(map, goal_y_map, goal_x_map)
% 4-neighbour wavefill function

  [sizey, sizex] = size(map);

  map(goal_y_map,goal_x_map) = 1; % goal location
  k = 1; % current maximum path cost

  while sum(map(:)==255) ~= 0, % while no cell is equal to 255
    for i = 1:sizey,
      for j = 1:sizex,
        if map(i,j) == k, % expand all cells with distance k

          % up/down cells
          for m = [i-1,i+1],
            if m > 0 && m <= sizey && map(m,j) == 255,
              map(m,j) = map(i,j) + 1;
            end
          end

          % left/right cells
          for n = [j-1,j+1],
            if n > 0 && n <= sizex && map(i,n) == 255,
              map(i,n) = map(i,j) + 1;
            end
          end

        end
      end % for j
    end % for i
    k = k + 1;
  end % while

end % wavefill

function pathvec = wavefrontPath(map, startpos, goal)
% 4-neighbour wavefront path generator; note pathvec is indexed as (x,y)
% pathvec is a kx2 vector, where every row is a new point along the path

  [sizey sizex] = size(map);

  i = ceil(sizey*((startpos(2)+7.5)/15)); % y-coord
  j = ceil(sizex*((startpos(1)+7.5)/15)); % x-coord

  pathvec(1,:) = [i j];
  k = 2; % number of points (plus one) in set pathvec

  while map(i,j) ~= 1, % until the goal is reached

    if     (i-1 > 0) && (i-1 <= sizey) && (map(i-1,j) ~= 0) && (map(i-1,j) < map(i,j)),
      i=i-1;
    elseif (j-1 > 0) && (j-1 <= sizex) && (map(i,j-1) ~= 0) && (map(i,j-1) < map(i,j)),
      j=j-1;
    elseif (j+1 > 0) && (j+1 <= sizex) && (map(i,j+1) ~= 0) && (map(i,j+1) < map(i,j)),
      j=j+1;
    elseif (i+1 > 0) && (i+1 <= sizey) && (map(i+1,j) ~= 0) && (map(i+1,j) < map(i,j)),
      i=i+1;
    else
        error('No path found.');
    end

    pathvec(k,:) = [i j];
    k = k + 1;

  end % while

end % wavefrontPath