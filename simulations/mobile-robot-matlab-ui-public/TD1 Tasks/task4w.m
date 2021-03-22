function my_alg = task4w(my_alg, robot)
% This function show the control signal and  the motor speed.
%
% Jiahe Wang, March 2021
% -------------------------------------------------------------------------
%
% Reading data from sensors (if present on the robot)
%    my_alg('right encoder') - right encoder velocity
%    my_alg('left encoder')  - left encoder velocity
% 
% Sending controls to actuators (if present on the robot)
%    my_alg('right motor')   - sets the right motor input signal (pwm or angular velocity)
%    my_alg('left motor')    - sets the left motor input signal (pwm or angular velocity)
% -------------------------------------------------------------------------

if my_alg('is_first_time')
    %% Setup initial parameters here
    
    my_alg('dc_motor_signal_mode') = 'voltage_pwm';     % change if necessary to 'omega_setpoint'
    
    % Initialise wheel angular velocity contollers
    my_alg('wR_set') = 7;
    my_alg('wL_set') = 6;
    
    % Initialise vectors for saving data
    my_alg('wR_all') = [];
    my_alg('wL_all') = [];
    my_alg('wR_control') = [];
    my_alg('wL_control') = [];
        
    % Initialise time parameters
    my_alg('t_sampling_inner') = 0.03;
    my_alg('t_sampling_outer') = 0.1;
    my_alg('t_loop_inner') = tic;
    my_alg('t_loop_outer') = tic;
    my_alg('t_finish') = 5;
    
    %pid
    my_alg('errorw') = 0;
    my_alg('errorl') = 0;
    my_alg('preerrorw') = 0;
    my_alg('preerrorl') = 0;
    my_alg('toterrorw') = 0;
    my_alg('toterrorl') = 0;
    my_alg('differrorw') = 0;
    my_alg('differrorl') = 0;
    my_alg('uR')= 0;
    my_alg('uL')= 0;
end

%% Loop code runs here

time = toc(my_alg('tic'));      % Get time since start of session

if time < my_alg('t_finish')    % Check for algorithm finish time
    
    dt = toc(my_alg('t_loop_outer'));
    if dt>my_alg('t_sampling_outer')  % execute code when desired sampling time is reached
        my_alg('t_loop_outer') = tic;
        
        dt = toc(my_alg('t_loop_inner'));  
        if dt>my_alg('t_sampling_inner')
            my_alg('t_loop_inner') = tic;
        %% Add your loop code here (replace with your controller)
        kp = 0.2;
        ki = 0.1;
        my_alg('errorw') = my_alg('wR_set') - my_alg('right encoder');
        my_alg('errorl') = my_alg('wL_set') - my_alg('left encoder');
        my_alg('toterrorw') = my_alg('toterrorw') + my_alg('errorw');
        my_alg('toterrorl') = my_alg('toterrorl') + my_alg('errorl');
        my_alg('uR')= (kp * my_alg('errorw') + ki * my_alg('toterrorw'))/13.8735;
        my_alg('uL')= (kp * my_alg('errorl') + ki * my_alg('toterrorl'))/13.8735;
    
        % Apply pwm signal
        my_alg('right motor') = my_alg('uR');
        my_alg('left motor') = my_alg('uL');

        % Save data for ploting
        my_alg('wR_all') = [my_alg('wR_all') my_alg('right encoder')];
        my_alg('wL_all') = [my_alg('wL_all') my_alg('left encoder')];
        
        my_alg('wR_control') = [my_alg('wR_control') my_alg('uR')];
        my_alg('wL_control') = [my_alg('wL_control') my_alg('uL')];
        
        my_alg('preerrorw') = my_alg('errorw');
        my_alg('preerrorl') = my_alg('errorl');
        end
    end

else
    %% Finish algorithm and plot results
    
    % Stop motors
    my_alg('right motor') = 0;
    my_alg('left motor') = 0;
    % Stop session
    my_alg('is_done') = true;
    
    % Plot saved velocities for right and left wheel
    figure(1);
    plot(my_alg('wR_all')*34);
    hold on
    plot(my_alg('wL_all')*34);
    title('motor speed');
    legend('right wheel','left wheel');
    xlabel('number of sampling');
    ylabel('motor speed(rpm)');
    
    figure(2);
    plot(my_alg('wR_control'));
    hold on
    plot(my_alg('wL_control'));
    title('Control signal');
    legend('right wheel','left wheel');
    xlabel('number of sampling');
    ylabel('PWM (duty cycle)');
   
end

return