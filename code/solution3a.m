function [forwBackVel, leftRightVel, rotVel, finish] = solution3a(pts, contacts, delta_position, delta_orientation)
% the control loop callback function - the solution for task 3A

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
  persistent map;         % map to work on
  persistent Q;           % covariance of the noise added to particle states
  persistent N_PARTICLES; % number of particles
  persistent p_state;     % particle states;  N_PARTICLES x 3 (x, y, phi)
  persistent p_weight;    % particle weights; N_PARTICLES x 1
  persistent p_estimate;  % estimated position; weighted mean of the particle population
  persistent randstream;  % mersenne twister engine

  persistent h1;          % graphics handle for plotting states
  persistent h2;          % graphics handle for plotting estimate

  if strcmp(state, 'init'),

    % set constants
    Q = diag([0.01, 0.01, 0.08*pi/180]); %for noise covariance later
    N_PARTICLES = 10000;

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
    h1 = plot( p_state(:,1),  p_state(:,2),  ['*','blue'] ); % plot particles
    h2 = plot( p_estimate(1), p_estimate(2), ['*','red']  ); % plot estimate

  else

    % predict
    % a set of random vectors to be added as noise
    randvec = randstream.randn(N_PARTICLES,3);
    % Q is the covariance of the noise
    noise = randvec*Q;

    dp  = delta_position;     % change in position
    dth = delta_orientation;  % change in orientation
    th  = p_state(:,3) + dth; % new orientation

    % total change in state in the global reference frame
    % the odometry is the local reference frame, so we rotate by -theta
    delta = [dp(1)*cos(-th)+dp(2)*sin(-th) dp(1)*-sin(-th)+dp(2)*cos(-th) ones(size(p_state,1),1)*dth];

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
    % coordinates on the map of the position of the particle
    x_map = ceil(100*((p_state(:,2)+7.5)/15));
    y_map = ceil(100*((p_state(:,1)+7.5)/15));

    % vector of booleans of particles on obstacles
    occ = ( map(sub2ind(size(map), x_map, y_map)) == 0 ); %takes 1 for obstacles

    % reduce the weights of particles on obstacles
    p_weight = ones(N_PARTICLES,1);
    p_weight(occ) = 0;

    % select
    % remove particles with weights equal to zero
    p_state = p_state(~occ,:);
    % how many particles did we remove?
    need = N_PARTICLES - size(p_state,1);
    % randomly select 'need' number of particles from p_state
    % to duplicate
    randsel = randstream.randi(size(p_state,1), need, 1);
    duplicates = p_state(randsel,:);
    % concatenate the duplicates into the states vector
    p_state = [p_state; duplicates];

    % weighted mean
    % weighted mean is equal to the arithmetic mean
    % since all weights are 1
    p_estimate = mean(p_state(:,1:2));
    
    % plot
    set( h1, 'Xdata', p_state(:,1),  'Ydata', p_state(:,2)  );
    set( h2, 'Xdata', p_estimate(1), 'Ydata', p_estimate(2) );
  
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
    MAX_PARA_VEL  = 7;
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

end % solution3a