function my_alg = example_sonar(my_alg, robot)
% This function implements two control loops:
%   - the inner loop implements angular velocity controllers for both
%   wheels;
%   - the outer loop implements a proportional controller for holding a
%   constant distance to a wall using the sonar;
%
%  Load world_wall.json in the Gui to test the example.
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
            
    % Initialise time parameters
    my_alg('sampling_inner') = 0.03;
    my_alg('sampling_outer') = 0.1;
    my_alg('t_inner_loop') = tic;
    my_alg('t_outer_loop') = tic;
    my_alg('t_finish') = 22;
    my_alg('Integral_right') = 0;
    my_alg('Integral_left') = 0;

    my_alg('Kp')= 0.01;
    my_alg('Ki')= 0.33;
    
    my_alg('Kp_wall')= 2;
    my_alg('Ki_wall')= 0.5;    
    my_alg('Kd_wall')= 2;
    
    my_alg('dist')= 0.5;
    my_alg('pre')=0;
    my_alg('Integral_w')=0;

    % Servo motor angle (1.57 radians = move servo to the left (towards the wall))
    my_alg('servo motor') = 1.5708;
    my_alg('dist_all') = [];
    my_alg('wallsense_1')=[];
    
end

%% Loop code runs here

time = toc(my_alg('tic'));      % Get time since start of session

if time < my_alg('t_finish')    % Check for algorithm finish time
    
    %% Outer Loop
    dt = toc(my_alg('t_outer_loop')); 
    
    if dt>my_alg('sampling_outer')  % Execute code when desired outer loop sampling time is reached
        my_alg('t_outer_loop') = tic;
        
            wallsense = my_alg('Kp_wall')*(my_alg('sonar')- my_alg('dist')) + my_alg('Integral_w')+my_alg('Ki_wall')*dt*(my_alg('sonar')- my_alg('dist'))+my_alg('Kd_wall')*(my_alg('sonar')- my_alg('dist')-my_alg('pre'))/dt;
            my_alg('Integral_w')=my_alg('Integral_w')+my_alg('Ki_wall')*dt*(my_alg('sonar')- my_alg('dist'));
            my_alg('pre')=my_alg('sonar')- my_alg('dist');
            
            
            my_alg('wR_set') = 7+wallsense;
            my_alg('wL_set') = 7-wallsense;

            my_alg('dist_all')= [my_alg('dist_all') my_alg('sonar')];
            my_alg('wallsense_1')= [my_alg('wallsense_1') wallsense];          
                      

        
    end
    %%
    
    %% Inner Loop
    dt = toc(my_alg('t_inner_loop'));
    
    if dt>my_alg('sampling_inner')  % Execute code when desired inner loop sampling time is reached 
        my_alg('t_inner_loop') = tic;

        %% Add your inner loop code here (replace with your controller)%%%
        % Right wheel controller %%%%%%%%%%%%%%%%%%%%
        uR = my_alg('Kp')*(my_alg('wR_set')-my_alg('right encoder')) + my_alg('Integral_right')+my_alg('Ki')*dt*(my_alg('wR_set')-my_alg('right encoder'));
        uL = my_alg('Kp')*(my_alg('wL_set')-my_alg('left encoder')) + my_alg('Integral_left')+my_alg('Ki')*dt*(my_alg('wL_set')-my_alg('left encoder'));

        my_alg('Integral_right') = my_alg('Integral_right') + my_alg('Ki')*dt*(my_alg('wR_set')-my_alg('right encoder'));
        my_alg('Integral_left') = my_alg('Integral_left') + my_alg('Ki')*dt*(my_alg('wL_set')-my_alg('left encoder'));
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % Apply pwm signal
        my_alg('right motor') = uR;
        my_alg('left motor') = uL;
        %% End %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end
    %%
else
    %% Finish algorithm and plot results
    
    % Stop motors
    my_alg('right motor') = 0;
    my_alg('left motor') = 0;
    
    % Set servo angle to 0
    my_alg('servo motor') = 0;
    
    % Stop session
    my_alg('is_done') = true;
    figure(2);    % Plot saved velocities for right and left wheel
    plot(my_alg('dist_all'));
    figure(3);
    plot(my_alg('wallsense_1'));

    
    
end

return