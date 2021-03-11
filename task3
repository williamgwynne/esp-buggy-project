function my_alg = speed(my_alg, robot)
% This function drives the robot in a curve line for 10 seconds.
% Then, it stops.
%
% guoqisheng, March 2021
% -------------------------------------------------------------------------

if my_alg('is_first_time')
     
    
    my_alg('dc_motor_signal_mode') = 'voltage_pwm';     % change if necessary to 'omega_setpoint'
    
    % Initialise vectors for saving velocity data
    my_alg('wR_all') = [];
    my_alg('wL_all') = [];
        
    % Initialise time parameters
    my_alg('t_sampling') = 0.03;
    my_alg('t_loop') = tic;

end
time = toc(my_alg('tic')); % Get time since start of session
if time < 10
        % Drive
        my_alg('right motor') = 0.8;
        my_alg('left motor') = 0.7;
        % Save data for ploting
        my_alg('wR_all') = [my_alg('wR_all') my_alg('right encoder')];
        my_alg('wL_all') = [my_alg('wL_all') my_alg('left encoder')];
else
        % Stop motors
        my_alg('right motor') = 0;
        my_alg('left motor') = 0;
        % Stop session
        my_alg('is_done') = true;
        % Plot saved velocities for right and left wheel
        figure(2);
        plot(my_alg('wR_all'));
        hold on
        plot(my_alg('wL_all'));
end
return
