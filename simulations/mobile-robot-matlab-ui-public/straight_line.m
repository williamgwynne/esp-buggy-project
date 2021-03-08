function my_alg = straight_line(my_alg, robot)
% This function implements velocity controllers for both wheels 
% and applies the desired setpoints for a specified amount of time.
%
% Mohamed Mustafa, August 2020
% -------------------------------------------------------------------------
%
% Reading data from sensors (if present on the robot)
%    my_alg('right encoder') - right encoder velocity
%    my_alg('left encoder')  - left encoder velocity
%    my_alg('reflectance')   - reflectance sensor output value
%    my_alg('reflectance raw')   - reflectance sensor raw output values
%    my_alg('sonar')         - sonar measured distance (m)
% 
% Sending controls to actuators (if present on the robot)
%    my_alg('right motor')   - sets the right motor input signal (pwm or angular velocity)
%    my_alg('left motor')    - sets the left motor input signal (pwm or angular velocity)
%    my_alg('servo motor')   - sets the servomotor angle (radians)
% -------------------------------------------------------------------------

if my_alg('is_first_time')
    %% Setup initial parameters here
    
    my_alg('dc_motor_signal_mode') = 'voltage_pwm';     % change if necessary to 'omega_setpoint'
    
    % Initialise wheel angular velocity contollers
    my_alg('wR_set') = 7;
    my_alg('wL_set') = 7;
    
    my_alg('control_right') = MotorControl();
    my_alg('control_left') = MotorControl();
    
    % Initialise vectors for saving velocity data
    my_alg('wR_all') = [];
    my_alg('wL_all') = [];
        
    % Initialise time parameters
    my_alg('t_sampling') = 0.03;
    my_alg('t_loop') = tic;
    my_alg('t_finish') = 6;
    
    %initialise kinematic variables
    my_alg('distance') = 0;
    my_alg('total_phi') = 0;
    
    %initialising PID variables
    my_alg('w2p_ratio') = 901/12500;
    my_alg('errorspeedright_prev')=0;
    my_alg('errorspeedleft_prev')=0;
    my_alg('errorspeedright_sum')=0;
    my_alg('errorspeedleft_sum')=0;
    my_alg('kp_speed')=0.3;%proportional coefficient
    my_alg('ki_speed')=0.001;
    my_alg('kd_speed')=0.006;%differential coefficient
end

%% Loop code runs here

time = toc(my_alg('tic'));      % Get time since start of session

if time < my_alg('t_finish')    % Check for algorithm finish time
    
    dt = toc(my_alg('t_loop'));
    
    if dt>my_alg('t_sampling')  % execute code when desired sampling time is reached
        my_alg('t_loop') = tic;

        linearVelocity_left = my_alg('left encoder') * 0.05; %v=wr
        linearVelocity_right = my_alg('right encoder') * 0.05; %v=wr

        averageVelocity = (linearVelocity_left + linearVelocity_right)/2;

        my_alg('distance') = (averageVelocity*dt) + my_alg('distance');
        distance = my_alg('distance');

        phi = (linearVelocity_left - linearVelocity_right)*dt / 0.18; %angle is clockwise

        my_alg('total_phi') = phi + my_alg('total_phi');
        totalphi=my_alg('total_phi');
        
        %% Add your loop code here (replace with your controller)%%%%%%%%%
        % Right wheel controller %%%%%%%%%%%%%%%%%%%%
        %uR = my_alg('control_right').Control(my_alg('wR_set'),my_alg('right encoder'),dt);
        errorspeedright = my_alg('wR_set')-my_alg('right encoder')
        my_alg('errorspeedright_sum') = errorspeedright + my_alg('errorspeedright_sum');
        uR = (my_alg('wR_set') + (errorspeedright * my_alg('kp_speed') + my_alg('ki_speed') * my_alg('errorspeedright_sum')*dt + my_alg('kd_speed') * (errorspeedright-my_alg('errorspeedright_prev'))/dt))*my_alg('w2p_ratio');
        my_alg('errorspeedright_prev') = errorspeedright;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % Left wheel controller %%%%%%%%%%%%%%%%%%%%%
        %uL = my_alg('control_left').Control(my_alg('wL_set'),my_alg('left encoder'),dt);
        errorspeedleft = my_alg('wL_set')-my_alg('left encoder');
        my_alg('errorspeedleft_sum') = errorspeedleft+my_alg('errorspeedleft_sum');
        uL = (my_alg('wL_set') + (errorspeedleft * my_alg('kp_speed') + my_alg('ki_speed') * my_alg('errorspeedleft_sum')*dt + my_alg('kd_speed') * (errorspeedleft-my_alg('errorspeedleft_prev'))/dt))*my_alg('w2p_ratio');
        my_alg('errorspeedleft_prev') = errorspeedleft;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % Apply pwm signal
        my_alg('right motor') = uR;
        my_alg('left motor') = uL;

        % Save data for ploting
        my_alg('wR_all') = [my_alg('wR_all') (my_alg('right encoder'))];
        my_alg('wL_all') = [my_alg('wL_all') (my_alg('left encoder'))];
        %% End %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   end

else
    %% Finish algorithm and plot results
    
    % Stop motors
    my_alg('right motor') = 0;
    my_alg('left motor') = 0;
    % Stop session
    my_alg('is_done') = true;
    
     % Plot saved velocities for right and left wheel
     figure(2);
     plot(my_alg('wR_all'));
     hold on
     %plot(my_alg('wL_all'));

end

return