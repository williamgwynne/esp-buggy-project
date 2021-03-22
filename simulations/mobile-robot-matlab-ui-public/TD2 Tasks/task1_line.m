function my_alg = task1_line(my_alg, robot)
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
    my_alg('pi') = 3.1415926;
    
    % Initialise wheel angular velocity contollers
    my_alg('wR_set') = 0;
    my_alg('wL_set') = 0;
    
    my_alg('control_right') = MotorControl();
    my_alg('control_left') = MotorControl();
    
    % Initialise vectors for saving velocity data
    my_alg('wR_all') = [];
    my_alg('wL_all') = [];
    my_alg('distance_all') = [];
        
    % Initialise time parameters
    my_alg('t_sampling') = 0.03;
    my_alg('t_loop') = tic;
    my_alg('t_finish') = 10;
    my_alg('dt_total') = 0;
    my_alg('dt_all') = [];
    
    %initialising PID variables for motor control
    my_alg('w2p_ratio') = 901/12500;
    my_alg('errorspeedright_prev')=0;
    my_alg('errorspeedleft_prev')=0;
    my_alg('errorspeedright_sum')=0;
    my_alg('errorspeedleft_sum')=0;
    my_alg('kp_speed')=0.3;%proportional coefficient
    my_alg('ki_speed')=0.001;
    my_alg('kd_speed')=0.012;%differential coefficient
    
    %initialising sensor readings
    my_alg('line_error') = 0;
    my_alg('line_error_all') = [];
end

%% Loop code runs here

time = toc(my_alg('tic'));      % Get time since start of session

if time < my_alg('t_finish')    % Check for algorithm finish time
    
    dt = toc(my_alg('t_loop'));
    
    if dt>my_alg('t_sampling')  % execute code when desired sampling time is reached
        my_alg('t_loop') = tic;
        
        %sensor readings
        sensorratio = 1-((my_alg('reflectance raw')-500)/2000);
        sensor_err_numerator=0;
        sensor_err_denominator=sum(sensorratio, 'all');
        for i=0:7
            sensor_err_numerator=sensor_err_numerator+sensorratio(i+1)*i;
        end
        my_alg('line_error') = (sensor_err_numerator/sensor_err_denominator) - 3.5;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %speed adjustments
        forwardspeed = 0.5;
        my_alg('wR_set') = forwardspeed/0.05; %converting to angular speed
        my_alg('wL_set') = forwardspeed/0.05;  
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % Right wheel controller %%%%%%%%%%%%%%%%%%%%
        errorspeedright = my_alg('wR_set')-my_alg('right encoder')
        my_alg('errorspeedright_sum') = errorspeedright + my_alg('errorspeedright_sum');
        uR = (my_alg('wR_set') + (errorspeedright * my_alg('kp_speed') + my_alg('ki_speed') * my_alg('errorspeedright_sum')*dt + my_alg('kd_speed') * (errorspeedright-my_alg('errorspeedright_prev'))/dt))*my_alg('w2p_ratio');
        my_alg('errorspeedright_prev') = errorspeedright;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % Left wheel controller %%%%%%%%%%%%%%%%%%%%%
        errorspeedleft = my_alg('wL_set')-my_alg('left encoder')
        my_alg('errorspeedleft_sum') = errorspeedleft+my_alg('errorspeedleft_sum');
        uL = (my_alg('wL_set') + (errorspeedleft * my_alg('kp_speed') + my_alg('ki_speed') * my_alg('errorspeedleft_sum')*dt + my_alg('kd_speed') * (errorspeedleft-my_alg('errorspeedleft_prev'))/dt))*my_alg('w2p_ratio');
        my_alg('errorspeedleft_prev') = errorspeedleft;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % Apply pwm signal
        my_alg('right motor') = uR;
        my_alg('left motor') = uL;
        
        %save data for plotting
        my_alg('line_error_all') = [my_alg('line_error_all') my_alg('line_error')];
        my_alg('dt_total') = my_alg('dt_total') + dt;
        my_alg('dt_all') = [my_alg('dt_all') my_alg('dt_total')];
        %% End %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   end

else
    %% Finish algorithm and plot results
    
    % Stop motors
    my_alg('right motor') = 0;
    my_alg('left motor') = 0;
    % Stop session
    my_alg('is_done') = true;
    
    figure(1);
    plot(my_alg('dt_all'), my_alg('line_error_all'));
    title('Line sensor readings');
    xlabel('Time');
    ylabel('Line error');
    hold on
end

return
