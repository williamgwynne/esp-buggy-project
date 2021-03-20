function my_alg = line_follower(my_alg, robot)
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
    my_alg('dt_all') = [];
    my_alg('dt_total') = 0;
    my_alg('distance_all') = [];
        
    % Initialise time parameters
    my_alg('t_sampling') = 0.03;
    my_alg('t_loop') = tic;
    my_alg('t_finish') = 10;
    
    %initialise kinematic variables
    my_alg('distance') = 0;
    
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
    my_alg('kp_distance') = 144 %36;
    my_alg('ki_distance') = 0;
    my_alg('kd_distance') = 1.8 %0.45;
    my_alg('errordistance_sum') = 0;
    my_alg('errordistance_prev') = 0;
    
    %PID coefficients for line error
    my_alg('kp_line_err') = 1.35;
    my_alg('ki_line_err') = 4.56;
    my_alg('kd_line_err') = 0.055;
    my_alg('line_err_sum') = 0;
    my_alg('line_err_prev') = 0;
    
end

%% Loop code runs here

time = toc(my_alg('tic'));      % Get time since start of session

if time < my_alg('t_finish')    % Check for algorithm finish time
    
    dt = toc(my_alg('t_loop'));
    
    if dt>my_alg('t_sampling')  % execute code when desired sampling time is reached
        my_alg('t_loop') = tic;
        %if (my_alg('sonar') > 0.3)
            linearVelocity_left = my_alg('left encoder') * 0.05; %v=wr
            linearVelocity_right = my_alg('right encoder') * 0.05; %v=wr

            my_alg('averageVelocity') = (linearVelocity_left + linearVelocity_right)/2;
            my_alg('distance') = (my_alg('averageVelocity')*dt) + my_alg('distance');

            %deviation adjustments
            sensorratio = 1-((my_alg('reflectance raw')-500)/2000);
            sensor_err_numerator=0;
            sensor_err_denominator=sum(sensorratio, 'all');
            for i=0:7
                sensor_err_numerator=sensor_err_numerator+sensorratio(i+1)*i;
            end
            line_error = (sensor_err_numerator/sensor_err_denominator) - 3.5;
            line_error_mod = abs(line_error);
            my_alg('line_err_sum') = my_alg('line_err_sum') + line_error;
            if (line_error<0) %line is to the left
                wR_ratio = 3.5; %3.5 is mid-point of the sensor array
                wL_ratio = 3.5-(my_alg('kp_line_err')*line_error_mod + my_alg('ki_line_err')*my_alg('line_err_sum')*dt+my_alg('kd_line_err')*((line_error-my_alg('line_err_prev'))/dt));
            elseif (line_error>0) %line is to the right
                wR_ratio = 3.5-(my_alg('kp_line_err')*line_error_mod + my_alg('ki_line_err')*my_alg('line_err_sum')*dt+my_alg('kd_line_err')*((line_error-my_alg('line_err_prev'))/dt));
                wL_ratio = 3.5;
            else %line error = 0
                wR_ratio = 3.5;
                wL_ratio = 3.5;
                my_alg('line_err_sum') = 0;
            end
            if (wL_ratio<0)
                wL_ratio = 0;
            elseif (wL_ratio>3.5)
                wL_ratio = 3.5;
            elseif (wR_ratio>3.5)
                wR_ratio = 3.5;
            elseif (wR_ratio<0)
                wR_ratio = 0;
            end
            my_alg('line_err_prev') = line_error;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            %speed adjustments
            errordistance = (my_alg('sonar')-0.3); %3 metres is set distance for straight line
            my_alg('errordistance_sum') = my_alg('errordistance_sum') + errordistance;
               if (my_alg('errordistance_sum')>(my_alg('sonar')-0.3))
                   my_alg('errordistance_sum')=(my_alg('sonar')-0.3);
               elseif (my_alg('errordistance_sum')<0)
                   my_alg('errordistance_sum')=0;
               end
            forwardspeed = 0.7 ;%(0 + (errordistance * my_alg('kp_distance') + my_alg('ki_distance') * my_alg('errordistance_sum')*dt + my_alg('kd_distance') * (errordistance-my_alg('errordistance_prev'))/dt))/dt; %0 is steady state speed
            
            if (forwardspeed>0.7) %linear speed where angular velocity saturates - if need be, calculate a more accurate value
                forwardspeed=0.7;
            end
            
            my_alg('errordistance_prev') = errordistance;
             if (-0.01 < forwardspeed < 0.01)
                errorspeedright = 0;
                errorspeedleft = 0;
            end

            my_alg('wR_set') = (wR_ratio/3.5)*(forwardspeed/0.05); %converting to angular speed
            my_alg('wL_set') = (wL_ratio/3.5)*(forwardspeed/0.05);
%         else
%             my_alg('wR_set') = 0;
%             my_alg('wL_set') = 0;
%             my_alg('averageVelocity') = 0;
%             my_alg('errorspeedright_sum') = 0;
%             my_alg('errorspeedleft_sum') = 0;
%             my_alg('errorspeedright_prev') = 0;
%             my_alg('errorspeedleft_prev') = 0;
%         end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
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
        my_alg('wR_all') = [my_alg('wR_all') line_error];
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
    plot(my_alg('dt_all'), my_alg('wR_all'));
    hold on
end

return
