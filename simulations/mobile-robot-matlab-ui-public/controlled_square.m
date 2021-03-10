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
    my_alg('t_finish') = 15;
    
    %initialising PID variables for motor control
    my_alg('w2p_ratio') = 901/12500;
    my_alg('errorspeedright_prev')=0;
    my_alg('errorspeedleft_prev')=0;
    my_alg('errorspeedright_sum')=0;
    my_alg('errorspeedleft_sum')=0;
    my_alg('kp_speed')=0.3;%proportional coefficient
    my_alg('ki_speed')=0.001;
    my_alg('kd_speed')=0.012;%differential coefficient
    
    %PID coefficients for line-speed control
    my_alg('forwardspeed') = 0;
    my_alg('distance') = 0;
    my_alg('kp_distance') = 0.1; %20 %36;
    my_alg('ki_distance') = 0; %100 %0.5 %600;
    my_alg('kd_distance') = 0; %20 %4 %0.45;
    my_alg('errordistance_sum') = 0;
    my_alg('errordistance_prev') = 0;
    
    %initialising PID coefficients for turning control
    my_alg('cornercount') = 0;
    my_alg('phi') = 0;
end

%% Loop code runs here

time = toc(my_alg('tic'));      % Get time since start of session

if time < my_alg('t_finish')    % Check for algorithm finish time
    
    dt = toc(my_alg('t_loop'));
        
    if dt>my_alg('t_sampling')  % execute code when desired sampling time is reached
        my_alg('t_loop') = tic;
        if (my_alg('phi') < 0.01)
            if ((my_alg('forwardspeed') < 0.01) && (my_alg('distance')>0.99) && (my_alg('cornercount')<8)) %sets the angle to turn by
                my_alg('cornercount') = my_alg('cornercount')+1;
                corner=my_alg('cornercount') %trace, delete afterwards
                if(my_alg('cornercount')==4)
                    my_alg('phi') = my_alg('pi'); %complete a full turn at the 4th corner
                else
                    my_alg('phi') = my_alg('pi')/2;
                end
                my_alg('distance') = 0;
            else        
                %speed adjustments
                linearVelocity_left = my_alg('left encoder') * 0.05; %v=wr
                linearVelocity_right = my_alg('right encoder') * 0.05; %v=wr
                averageVelocity = (linearVelocity_left + linearVelocity_right)/2;
                my_alg('distance') = (averageVelocity*dt) + my_alg('distance');

                my_alg('errordistance') = 1 - my_alg('distance');
                my_alg('errordistance_sum') = my_alg('errordistance_sum') + my_alg('errordistance');
                my_alg('forwardspeed') = (0 + (my_alg('errordistance') * my_alg('kp_distance') + my_alg('ki_distance') * my_alg('errordistance_sum')*dt + my_alg('kd_distance') * (my_alg('errordistance')-my_alg('errordistance_prev'))/dt))/dt; %0 is steady state speed
                my_alg('errordistance_prev') = my_alg('errordistance');
                my_alg('wR_set') = my_alg('forwardspeed')/0.05; %-0.09*adjustmentangle)/0.05; %converting to angular speed
                my_alg('wL_set') = my_alg('forwardspeed')/0.05; %+0.09*adjustmentangle)/0.05; 
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            end
        else
            %controller for turning, linearity will return when phi<0.01
            rotationalspeed = 1;
            my_alg('difftheta') = (-2*rotationalspeed)/my_alg('buggy_width');
            my_alg('wR_set') = -rotationalspeed/0.05; %opposite wheel direction for clockwise motion
            my_alg('wR_set') = rotationalspeed/0.05;
        end
        
        
        % Right wheel controller %%%%%%%%%%%%%%%%%%%%
        errorspeedright = my_alg('wR_set')-my_alg('right encoder');
        my_alg('errorspeedright_sum') = errorspeedright + my_alg('errorspeedright_sum');
        uR = (my_alg('wR_set') + (errorspeedright * my_alg('kp_speed') + my_alg('ki_speed') * my_alg('errorspeedright_sum')*dt + my_alg('kd_speed') * (errorspeedright-my_alg('errorspeedright_prev'))/dt))*my_alg('w2p_ratio');
        my_alg('errorspeedright_prev') = errorspeedright;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % Left wheel controller %%%%%%%%%%%%%%%%%%%%%
        errorspeedleft = my_alg('wL_set')-my_alg('left encoder');
        my_alg('errorspeedleft_sum') = errorspeedleft+my_alg('errorspeedleft_sum');
        uL = (my_alg('wL_set') + (errorspeedleft * my_alg('kp_speed') + my_alg('ki_speed') * my_alg('errorspeedleft_sum')*dt + my_alg('kd_speed') * (errorspeedleft-my_alg('errorspeedleft_prev'))/dt))*my_alg('w2p_ratio');
        my_alg('errorspeedleft_prev') = errorspeedleft;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % Apply pwm signal
        my_alg('right motor') = uR;
        my_alg('left motor') = uL;       
        
        
        % Save data for ploting
        my_alg('wR_all') = [my_alg('wR_all') my_alg('errordistance')];
        my_alg('wL_all') = [my_alg('wL_all') my_alg('right encoder')];
        my_alg('distance_all') = [my_alg('distance_all') my_alg('distance')];
        %% End %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   end

else
    %% Finish algorithm and plot results
    
    % Stop motors
    my_alg('right motor') = 0;
    my_alg('left motor') = 0;
    % Stop session
    my_alg('is_done') = true;
    
%      % Plot saved velocities for right and left wheel
%       figure(2);
%       plot(my_alg('wR_all'));
%       hold on
      %plot(my_alg('wL_all'));

%       figure(3);
%       plot(my_alg('distance_all'));
%       hold on
end

return