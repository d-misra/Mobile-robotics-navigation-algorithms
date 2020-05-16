function [forwBackVel, leftRightVel, rotVel, finish] = solution3b(pts, contacts, delta_position, delta_orientation)
% the control loop callback function - the solution for task 3B

  % declare the persistent variable that keeps the state of the Finite
  % State Machine (FSM)
  persistent state;
  if isempty(state),
    % the initial state of the FSM is 'init'
    state = 'init';
  end

  % initialise the robot control variables (returned by this function)
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

  % BEGIN PARTICLE FILTER

  % static
  persistent map;          % map to work on
  persistent Q;            % covariance of the noise added to particle states
  persistent L;            % covariance of the likelihood function
  persistent N_PARTICLES;  % number of particles
  persistent p_state;      % particle states;  N_PARTICLES x 3 (x, y, phi)
  persistent p_weight;     % particle weights; N_PARTICLES x 1
  persistent p_estimate;   % estimated position; weighted mean of the particle population
  persistent randstream;   % mersenne twister engine

  persistent RESAM_THRESHOLD; % resampling threshold; resample if Var(p_weight) > threshold

  persistent h1;           % graphics handle for plotting states
  persistent h2;           % graphics handle for plotting estimate
  persistent h3;           % graphics handle for plotting weights

  persistent iter_count;   % just an iteration counter
  persistent resam_count;  % resampling counter

  if strcmp(state, 'init'),

    % set constants
    Q = diag([0.05, 0.05, 0.08*pi/180]); %for noise covariance later
    N_PARTICLES = 10000;
    RESAM_THRESHOLD = 0.02;

    % initialise variables
    map = imread('vrep_env/exercise02.png');
    % mersenne twister seeded with life, the universe and everything
    randstream = RandStream.create('mt19937ar', 'Seed', 42);

    % generate the initial uniformly distributed particle population
    % rand() generates uniformly distributed values in [0,1]
    % then we extend that to get the values we want in [-7.5,7.5]
    p_state = (2*randstream.rand(N_PARTICLES,3) - 1) * diag([7.5, 7.5, pi]);

    % reset weights and position estimate
    p_weight = ones(N_PARTICLES,1);
    p_estimate = [0 0];

    % plot particles and set graphics handle
    opengl hardware;
    set(gcf, 'Renderer', 'OpenGL');
    imagesc([-7.5 7.5], [-7.5 7.5], uint8(map));
    set(gca,'ydir','normal');
    hold on;
    h1 = plot(p_state(:,1), p_state(:,2), ['*','blue']);  % plot particles
    h2 = plot(p_estimate(1), p_estimate(2), ['*','red']); % plot estimate
    figure
    h3 = plot(1:N_PARTICLES, sort(p_weight));

    % reset counters
    iter_count  = 0;
    resam_count = 0;

  else

    % predict
    % a set of random vectors to be added as noise
    randvec = randstream.randn(N_PARTICLES,3);
    % Q is the covariance in the noise
    noise = randvec*Q;

    dp  = delta_position;     % change in position
    dth = delta_orientation;  % change in orientation
    th  = p_state(:,3) + dth; % new orientation

    % total change in state in the global reference frame
    % the odometry is the local reference frame, so we rotate by -theta
    delta = [dp(1)*cos(-th)+dp(2)*sin(-th) dp(1)*-sin(-th)+dp(2)*cos(-th) ones(N_PARTICLES,1)*dth];

    % add it up
    p_state = p_state + delta + noise;
    % wrap orientation angles in [-pi,pi]
    p_state(:,3) = angdiff(p_state(:,3));

    % clip into range
    % -7.49 because -7.5 returns a 0 index when converting
    % converting between world and map coordinates
    p_state(p_state(:,1) < -7.49, 1) = -7.49;
    p_state(p_state(:,1) >  7.5,  1) =  7.5;
    p_state(p_state(:,2) < -7.49, 2) = -7.49;
    p_state(p_state(:,2) >  7.5,  2) =  7.5;

    % observe
    % use the sensor model to predict the readings for
    % the LIDAR rays at -pi/4, 0 and pi/4
    predLIDAR(:,1) = predSensors(map, p_state, -pi/4, [-7.5,7.5], [-7.5,7.5]);
    predLIDAR(:,2) = predSensors(map, p_state, 0,     [-7.5,7.5], [-7.5,7.5]);
    predLIDAR(:,3) = predSensors(map, p_state, pi/4,  [-7.5,7.5], [-7.5,7.5]);

    % get the real LIDAR readings for the current position
    lidar_angles = [214 342 470];
    realLIDAR = (pts(1,lidar_angles).^2 + pts(2,lidar_angles).^2).^0.5;

    % calculate the innovation (error between predicted and real readings)
    innov = predLIDAR - realLIDAR;

    for i = 1:N_PARTICLES,
      e = ( -0.5 * innov (i,:) * innov(i,:)' );
      p_weight(i) = exp(e) + 0.05;
    end

    % find and set the weights of particles hitting obstacles to minimum
    % coordinates on the map of the position of the particle
    x_map = ceil(100*((p_state(:,2)+7.5)/15));
    y_map = ceil(100*((p_state(:,1)+7.5)/15));

    % vector of booleans of particles on obstacles
    occ = ( map(sub2ind(size(map), x_map, y_map)) == 0 );

    % reduce the weights of particles on obstacles
    p_weight(occ) = 0.05;

    % select
    iter_count = iter_count + 1;
    % only resample if the variance of the weights is greater than
    % some threshold
    % ref http://www.cs.cmu.edu/~16831-f14/notes/F11/16831_lecture04_tianyul.pdf
    % (2.1) Loss of diversity
    if var(p_weight) > RESAM_THRESHOLD,
      resam_count = resam_count + 1;
      % create a cumulative distribution of the weights
      % particles with large weights will occupy a greater percentage of
      % the y-axis
      CDF = cumsum(p_weight)/sum(p_weight);
      % uniformly and randomly choose y-values, since it's more likely
      % to correspond to better particles
      iSelect = randstream.rand(N_PARTICLES,1);
      % nearest-neighbour interpolation for the selected particles
      iNextGeneration = interp1(CDF, 1:N_PARTICLES, iSelect, 'nearest', 'extrap');
      % copy the selected particles to the next generation
      % note the number of particles remains the same due to interpolation
      p_state = p_state(iNextGeneration,:);
    end

    % weighted mean
    p_estimate(1) = sum( p_state(:,1).*p_weight ) / sum(p_weight);
    p_estimate(2) = sum( p_state(:,2).*p_weight ) / sum(p_weight);

    % print out some useful statistics
    fprintf('resampling ratio: %.4g\n', resam_count / iter_count);
    fprintf('Var(p_weight):    %.4g\n', var(p_weight));
    fprintf('min(p_weight):    %.4g\n', min(p_weight));
    fprintf('max(p_weight):    %.4g\n', max(p_weight));
    fprintf('pos estimate:     ( %.4f %.4f )\n', p_estimate);

    % plot
    set( h1, 'Xdata', p_state(:,1),  'Ydata', p_state(:,2)  );
    set( h2, 'Xdata', p_estimate(1), 'Ydata', p_estimate(2) );
    set( h3, 'Xdata', 1:N_PARTICLES, 'Ydata', sort(p_weight));
    drawnow
  
  end

  % END PARTICLE FILTER

  % everything following this is simply solution2a

  % wall-follower constants
  persistent K_P;           % Kp
  persistent MAX_PERP_VEL;  % max perpendicular velocity
  persistent MAX_PARA_VEL;  % max parallel velocity
  persistent MAX_ROT_VEL;   % max rotational velocity
  persistent WALL_DISTANCE; % distance to maintain from walls

  % manage the states of FSM
  if strcmp(state, 'init'),

    % initialise constants
    K_P = 10;
    MAX_PERP_VEL  = 1.5;
    MAX_PARA_VEL  = 8;
    MAX_ROT_VEL   = 10;
    WALL_DISTANCE = 1;

    state = 'move_to_wall';
    fprintf('changing FSM state to %s\n', state);

  elseif strcmp(state, 'move_to_wall'),

    % move forward until a wall is found
    error_perp = WALL_DISTANCE - min_value;
    forwBackVel = K_P * error_perp;
    if forwBackVel > MAX_PARA_VEL,
      forwBackVel = MAX_PARA_VEL;
    elseif forwBackVel < -MAX_PARA_VEL,
      forwBackVel = -MAX_PARA_VEL;
    end

    % are we there yet?
    if abs(error_perp) <= 0.01,
      state = 'move_along_wall';
      fprintf('changing FSM state to %s\n', state);
    end

  elseif strcmp(state, 'move_along_wall'),

    % rotational regulator
    theta = atan2(points(min_index,1), -points(min_index,2));
    rotVel = K_P * theta;
    if rotVel > MAX_ROT_VEL,
      rotVel = MAX_ROT_VEL;
    elseif rotVel < -MAX_ROT_VEL,
      rotVel = -MAX_ROT_VEL;
    end

    % perpendicular regulator
    rho_perp = K_P * (WALL_DISTANCE-min_value);
    if rho_perp > MAX_PERP_VEL,
      rho_perp = MAX_PERP_VEL;
    elseif rho_perp < -MAX_PERP_VEL;
      rho_perp = -MAX_PERP_VEL;
    end

    % parallel regulator
    % minimum distance to the left of min_value (every ray to the left)
    [min_value_left min_index_left] = min(distances(1:min_index-1));
    if min_value_left > min_value,
      rho_para = MAX_PARA_VEL;
    else
      rho_para = 0;
    end

    % convert between a rotated frame (a frame where the bot is presumed
    % to be oriented correctly, which our calculations are in)
    % to the real frame (where the bot has some orientation error (theta))
    rotated_frame = [rho_para rho_perp];
    rot_matrix = [cos(theta) sin(theta); -sin(theta) cos(theta)];
    real_frame = rotated_frame * rot_matrix;

    leftRightVel = real_frame(1);
    forwBackVel  = real_frame(2);

  else
    error('Unknown state %s.\n', state);
  end % fsm

end % solution3b

function result = predSensors(map, x, angle, dim_x, dim_y)
% Prediction of LIDAR sensor readings
% Format:
%               [x y] = predSensors(map, x, angle, dim_x, dim_y)
%
% Input:
%               map:   the map as a matrix created by imread function
%               x:     the array of state vectors, where each state
%                      vector is [x, y, fi]
%               angle: the angle of the LIDAR ray (in radians)
%               dim_x: the size of the map in the x axis direction
%               dim_y: the size of the map in the y axis direction
%
% Output:
%               result: the vector of distances from each state
%               particle to the closest obstacle in the direction of
%               particle's orientation and ray angle
%
% Usage example:
%               pfnparticles = 10000;
%               pfx = (2*pfrandstream.rand([pfnparticles,3]) - 1) * diag([7.5, 7.5, pi]);
%               dist = predSensors(map, pfx, pi/4, [-7.5,7.5], [-7.5, 7.5]);
  
  n = [cos(x(:,3)-pi/2+angle), sin(x(:,3)-pi/2+angle)];

  [x_count, ~] = size(x);
  [map_y_size, map_x_size] = size(map);
  ix = round( map_x_size*((x(:,1)-dim_x(1)) / (dim_x(2)- dim_x(1))) );
  iy = round( map_y_size*((x(:,2)-dim_y(1)) / (dim_y(2)- dim_y(1))) );

  result = zeros( [x_count, 1] ) - 1;
  
  for idx = 1:x_count
    if ix(idx) < 1 || iy(idx) < 1 || iy(idx) > map_y_size || ix(idx) > map_x_size,
      result(idx) = 100;
    elseif map(iy(idx), ix(idx)) == 0,
      result(idx) = 100;
    end
  end

  px_size = (dim_x(2) - dim_x(1)) / map_x_size;
  steps = 5 / px_size;
  for i = 1:steps
    d = (px_size * i) * n;
    ix = round(map_x_size*(x(:,1)+d(:,1)-dim_x(1)) / (dim_x(2)- dim_x(1)));
    iy = round(map_y_size*(x(:,2)+d(:,2)-dim_y(1)) / (dim_y(2)- dim_y(1)));
    for idx = 1:x_count
      if result(idx) > 0
        continue
      end
      if ix(idx) < 1 || iy(idx) < 1 || iy(idx) > map_y_size || ix(idx) > map_x_size,
        result(idx) = px_size * i;
      elseif map(iy(idx), ix(idx)) == 0,
        result(idx) = px_size * i;
      end
    end
  end

  for idx = 1:x_count
    if result(idx) < 0
      result(idx) = 5;
    end
  end
end % predSensors