function my_alg = zong(my_alg, robot)
% -------------------------------------------------------------------------
% Exteroceptive sensors output in simulation
% Group21_TD2_Task1,March 2021
%
% Aim 1
% Show that the simulated line sensor output changes as 
% predicted, as the buggy is slowly moved from the background on top of the line.
%
% Aim 2
% Show that the simulated sonar output changes as 
% predicted, as the buggy is slowly moved toward the wall. 
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
    
    % Initialise motor controller
    my_alg('ErrorR') = 0;
    my_alg('ErrorL') = 0;
    my_alg('toterrorR') = 0;
    my_alg('toterrorL') = 0;
    my_alg('differrorR') = 0;
    my_alg('differrorL') = 0;
    my_alg('uR') = 0;
    my_alg('uL') = 0;
    my_alg('preErrorR') = 0;
    my_alg('preErrorL') = 0;
            
    % Initialise time parameters
    my_alg('sampling_outer') = 0.03;
    my_alg('t_inner_loop') = tic;
    my_alg('t_outer_loop') = tic;
    my_alg('t_finish') = 50;
    
    % desired wheel velocity 
    my_alg('wR_desired') = 0;
    my_alg('wL_desired') = 0;
        
    % Servo motor angle (1.57 radians = move servo to the right (towards the wall))
    my_alg('servo motor') = -1.57;
    
    % sonar_distance_parameter
    my_alg('sonar_dist') = 0;
    my_alg('dist_right_wall') = [];
    
    % line_sensor_parameter
    my_alg('line_sensor') = 0;
    my_alg('line') = [];
    
end

time = toc(my_alg('tic'));      % Get time since start of session

if time < my_alg('t_finish')    % Check for algorithm finish time
    %% Outer Loop
       dt = toc(my_alg('t_outer_loop'));
    if dt>my_alg('sampling_outer')  % Execute code when desired outer loop sampling time is reached
          my_alg('t_outer_loop') = tic;
          % PID parameters
          kp = 0.2; 
          ki = 0.15; 
          kd = 0; 
          % Error calculation 
          my_alg('ErrorR') = my_alg('wR_desired') - my_alg('right encoder'); 
          my_alg('ErrorL') = my_alg('wL_desired') - my_alg('left encoder');
          % Error accumulation
          my_alg('toterrorR') = my_alg('toterrorR') + my_alg('ErrorR');
          my_alg('toterrorL') = my_alg('toterrorL') + my_alg('ErrorL');
          % Error difference
          my_alg('differrorR') = my_alg('ErrorR') - my_alg('preErrorR');
          my_alg('differrorL') = my_alg('ErrorL') - my_alg('preErrorL');
          my_alg('uL') = (kp*my_alg('ErrorL') +  ki*my_alg('toterrorL') + kd*my_alg('differrorL'))/13.8735;
          my_alg('uR') = (kp*my_alg('ErrorR') +  ki*my_alg('toterrorR') + kd*my_alg('differrorR'))/13.8735;
          w_max = 13.8735;
          my_alg('right motor') = my_alg('uR');
          my_alg('left motor') = my_alg('uL');
          % Save previous error
          my_alg('preErrorR') = my_alg('ErrorR');
          my_alg('preErrorL') = my_alg('ErrorL');
        %% Inner Loop
            my_alg('sonar_dist') = my_alg('sonar');  % Read sonar distance from robot
            my_alg('line_sensor') = my_alg('reflectance raw');
            my_alg('dist_right_wall') = [my_alg('dist_right_wall') my_alg('sonar_dist')];
            my_alg('line') = [my_alg('line') my_alg('line_sensor')];
            if (my_alg('sonar_dist') > 0) && (my_alg('sonar_dist') < 1.35)
                my_alg('wR_desired') = 6;
                my_alg('wL_desired') = 6;   
            end
              
            if (my_alg('sonar_dist') > 1.35) && (my_alg('sonar_dist') < 1.7)
                my_alg('wL_desired') = 5;
                my_alg('wR_desired') = 4;   
            end
            
            if (my_alg('sonar_dist') > 1.7) && (my_alg('sonar_dist') < 2)
                my_alg('wL_desired') = 5;
                my_alg('wR_desired') = 5;   
            end
    end
else
    %% Finish algorithm and plot results
    % Stop motors
    my_alg('right motor') = 0;
    my_alg('left motor') = 0;
    % Set servo angle to 0
    my_alg('servo motor') = 0;
    % Stop session
    my_alg('is_done') = true;
    % Plot saved control signals for left sonar
    f1 = figure(1);
    movegui(f1,'northwest');
    plot(my_alg('dist_left_wall'));
    xlabel('Number of Sampling');
    ylabel('Distanc/m');
    legend('R_sonar');
    title('distance toward left wall');
end
return