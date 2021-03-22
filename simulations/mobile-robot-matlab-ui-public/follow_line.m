function my_alg = line(my_alg, robot)
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
        my_alg('w_robot') = 0.18;
        my_alg('r_wheel') = 0.05;
        my_alg('w2p_ratio') = 901/12500;

        % Set the mode of motor
        my_alg('dc_motor_signal_mode') = 'voltage_pwm';

        %Initialise wheel angular velocity
        my_alg('wR_set') = 0.5 / my_alg('r_wheel');
        my_alg('wL_set') = 0.5 / my_alg('r_wheel');

        %Initialise position
        my_alg('pose_line') = 4.5;
        my_alg('error_line') = 0;
        my_alg('error_line_sum') = 0;
        my_alg('error_line_last') = 0;

        % Initialise vectors for saving velocity data
        my_alg('wR_all') = [];
        my_alg('wL_all') = [];
        my_alg('ur_all') = [];
        my_alg('ul_all') = [];

        % Initialise time parameters
        my_alg('sampling_outer') = 0.03;
        my_alg('t_inner_loop') = tic;
        my_alg('t_outer_loop') = tic;
        my_alg('t_finish') = 20;
    end

    time = toc(my_alg('tic'));

    if time < my_alg('t_finish') % Check for algorithm finish time

        dt = toc(my_alg('t_outer_loop'));

        if dt > my_alg('sampling_outer') % execute code when desired outer loop sampling time is reached
            my_alg('t_outer_loop') = tic;

            %%read line sensors
            linesensor = my_alg('reflectance raw');
            linesensor_bool = not(idivide(int32(linesensor), int32(ones(1, 8) * 2500)));

            %%calculate position
            if (sum(linesensor_bool) ~= 0)
                my_alg('pose_line') = sum(linesensor_bool .* (1:8)) / sum(linesensor_bool);
            end

            kp = 0.3;
            ki = 0;
            kd = 5;
            my_alg('error_line') = my_alg('pose_line') -4.5;
            my_alg('error_line_sum') = my_alg('error_line_sum') + my_alg('error_line');
            error_line_differ = my_alg('error_line') - my_alg('error_line_last');

            my_alg('wR_set') = 0.5 / my_alg('r_wheel') - (kp * my_alg('error_line') + ki * my_alg('error_line_sum') + kd * error_line_differ);
            my_alg('wL_set') = 0.5 / my_alg('r_wheel') + (kp * my_alg('error_line') + ki * my_alg('error_line_sum') + kd * error_line_differ);

            ur = my_alg('wR_set') * my_alg('w2p_ratio');
            ul = my_alg('wL_set') * my_alg('w2p_ratio');

            % Apply pwm signal (range is [-1,1])
            my_alg('right motor') = ur;
            my_alg('left motor') = ul;

            my_alg('error_line_last') = my_alg('error_line');

            % Save data for ploting
            my_alg('wR_all') = [my_alg('wR_all') my_alg('right encoder')];
            my_alg('wL_all') = [my_alg('wL_all') my_alg('left encoder')];
            my_alg('ur_all') = [my_alg('ur_all') ur];
            my_alg('ul_all') = [my_alg('ul_all') ul];

        end

    else
        %% Finish algorithm and plot results

        % Stop motors
        my_alg('right motor') = 0;
        my_alg('left motor') = 0;
        % Stop session
        my_alg('is_done') = true;

        figure(2);
        plot(my_alg('wR_all'));
        hold on
        plot(my_alg('wL_all'));
        legend('Right encoder', 'Left encoder');
        xlabel('Time (t)');
        ylabel('wheel velocity');
        title('')

        figure(3);
        plot(my_alg('ur_all'));
        hold on
        plot(my_alg('ul_all'));
        legend('Right motor', 'Left motor');
        xlabel('Time (t)');
        ylabel('Duty Cycle');
        title('');

    end

    return
