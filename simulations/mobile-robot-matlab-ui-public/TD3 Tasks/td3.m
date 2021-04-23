function my_alg = td3(my_alg, robot)

    if my_alg('is_first_time')

        %% Setup initial parameters here
        % Define some constant
        my_alg('r_wheel') = 0.05; % Wheel radius
        my_alg('l_wheel') = 0.18; % Width of the robot
        my_alg('w2u') = 901/12500; % Ratio between wheel angular velocity and pwm dutycycle

        % Set the mode of motor
        my_alg('dc_motor_signal_mode') = 'voltage_pwm';

        % Initialise sonar mode and control mode
        my_alg('servo motor') = 3 * pi / 2; % Measure Right
        my_alg('control_mode') = 0; % 0:Line position control 1:Wall position control 2:Break 3:Turn Back 4:Stop
        my_alg('counter_break') = 0;
        my_alg('counter_back') = 0;
        my_alg('counter_stop') = 1;
        my_alg('t_turning') = 0;

        % Initialise controller select
        my_alg('controller_select') = 0; % 0:Without controller 1:PID controller

        % Initialise position
        my_alg('pose_line') = 4.5;
        my_alg('pose_wall') = 0;

        % Initialise line position controller
        my_alg('error_line') = 0;
        my_alg('error_line_last') = 0;
        my_alg('error_line_sum') = 0;

        % Initialise wall position
        my_alg('error_wall') = 0;
        my_alg('error_wall_last') = 0;
        my_alg('error_wall_sum') = 0;

        % Initialise wheel angular velocity
        my_alg('w_desired') = 5;

        % Initialise pwm dutycycle
        my_alg('uR') = 0;
        my_alg('uL') = 0;

        % Initialise angluar velocity controller
        my_alg('eR_last') = 0;
        my_alg('eL_last') = 0;
        my_alg('eR_sum') = 0;
        my_alg('eL_sum') = 0;

        % Initialise vectors for plot
        my_alg('plot_signal') = 1; % 0:Not plot 1:Plot
        my_alg('t_all') = 0;
        my_alg('uR_all') = 0;
        my_alg('uL_all') = 0;
        my_alg('wR_all') = 0;
        my_alg('wL_all') = 0;

        % Initialise vectors for save
        my_alg('line_all') = ones(1, 8).' * 2500;
        my_alg('servo_all') = 3 * pi / 2;
        my_alg('counter_all') = 0;

        % Initialise time parameters
        my_alg('t_sampling') = 0.03;
        my_alg('t_loop') = tic;
        my_alg('t_finish') = 25;
    end

    %% Loop code runs here

    time = toc(my_alg('tic')); % Get time since start of session

    % if time < my_alg('t_finish') % Check for algorithm finish time
    if (my_alg('control_mode') < 4)
        dt = toc(my_alg('t_loop'));

        if dt > my_alg('t_sampling') % Execute code when desired loop sampling time is reached
            my_alg('t_loop') = tic;

            %% Change Mode
            % Read Linesensor
            linesensor = my_alg('reflectance_raw');
            linesensor_bool = not(idivide(int32(linesensor), int32(ones(1, 8) * 2500)));

            % Calculate Line Position
            if (sum(linesensor_bool) ~= 0)
                my_alg('pose_line') = sum(linesensor_bool .* (1:8)) / sum(linesensor_bool);
            end

            % Calulate Wall Position
            if (my_alg('sonar') ~= inf)
                my_alg('pose_wall') = my_alg('sonar');
            end

            % Change control mode
            if (sum(linesensor_bool) ~= 0 || my_alg('pose_line') == 1 || my_alg('pose_line') == 8)
                my_alg('control_mode') = 0;
                my_alg('counter_back') = 0;

                if (my_alg('t_turning') > 2)
                    my_alg('counter_stop') = 1;
                end

                my_alg('t_turning') = 0;

            elseif (((my_alg('sonar') > 0.3 && my_alg('sonar') < 0.7) || my_alg('counter_break') >= 66) && my_alg('t_turning') == 0)
                my_alg('control_mode') = 1;
                my_alg('counter_break') = 0;
                my_alg('counter_stop') = 1;
            elseif (my_alg('sonar') ~= inf && my_alg('t_turning') == 0)
                my_alg('control_mode') = 2;
                my_alg('counter_break') = my_alg('counter_break') + 1;
                my_alg('counter_back') = 0;
            elseif (my_alg('counter_back') < 33)
                my_alg('control_mode') = 2;
                my_alg('counter_back') = my_alg('counter_back') + 1;
            elseif (my_alg('counter_stop') ~= 1)
                my_alg('control_mode') = 3;
            else
                my_alg('control_mode') = 4;
            end

            % Change sonar mode
            if (my_alg('sonar') == inf && my_alg('servo motor') ~= 3 * pi / 2)
                my_alg('servo motor') = 3 * pi / 2;
            elseif (my_alg('sonar') == inf && my_alg('servo motor') ~= pi / 2)
                my_alg('servo motor') = pi / 2;
            end

            %% Line Position
            if (my_alg('control_mode') == 0)
                kp = 0.3; % 0.3 0.5
                ki = 0;
                kd = 2.4; % 2.4 4

                my_alg('error_line') = my_alg('pose_line') -4.5;
                my_alg('error_line_sum') = my_alg('error_line_sum') + my_alg('error_line');
                error_line_differ = my_alg('error_line') - my_alg('error_line_last');

                my_alg('wR_set') = my_alg('w_desired') - (kp * my_alg('error_line') + ki * my_alg('error_line_sum') + kd * error_line_differ);
                my_alg('wL_set') = my_alg('w_desired') + (kp * my_alg('error_line') + ki * my_alg('error_line_sum') + kd * error_line_differ);

                my_alg('error_line_last') = my_alg('error_line');
            end

            %% Wall Position
            if (my_alg('control_mode') == 1)
                kp = 4.8;
                ki = 0;
                kd = 24;

                direction = (my_alg('servo motor') - pi) / (pi / 2);

                my_alg('error_wall') = my_alg('pose_wall') -0.5;
                my_alg('error_wall_sum') = my_alg('error_wall_sum') + my_alg('error_wall');
                error_wall_differ = my_alg('error_wall') - my_alg('error_wall_last');

                my_alg('wR_set') = my_alg('w_desired') - direction * (kp * my_alg('error_wall') + ki * my_alg('error_wall_sum') + kd * error_wall_differ);
                my_alg('wL_set') = my_alg('w_desired') + direction * (kp * my_alg('error_wall') + ki * my_alg('error_wall_sum') + kd * error_wall_differ);

                my_alg('error_wall_last') = my_alg('error_wall');
            end

            %% Break
            if (my_alg('control_mode') == 2)
                my_alg('wR_set') = my_alg('w_desired');
                my_alg('wL_set') = my_alg('w_desired');
            end

            %% Turn Back
            if (my_alg('control_mode') == 3)

                if (my_alg('t_turning') <= 2)
                    my_alg('wR_set') = (pi / 2) * my_alg('l_wheel') / 2 / my_alg("r_wheel");
                    my_alg('wL_set') =- (pi / 2) * my_alg('l_wheel') / 2 / my_alg("r_wheel");
                else
                    my_alg('wR_set') = my_alg('w_desired');
                    my_alg('wL_set') = my_alg('w_desired');
                end

                my_alg('t_turning') = my_alg('t_turning') + dt;
            end

            %% Angluar Velocity
            % PID controller
            if (my_alg('controller_select'))
                kp = 0.14;
                ki = 0;
                kd = 0;

                eR = my_alg('wR_set') - my_alg('right encoder');
                eL = my_alg('wL_set') - my_alg('left encoder');

                my_alg('eR_sum') = my_alg('eR_sum') + eR;
                my_alg('eL_sum') = my_alg('eL_sum') + eL;

                eR_differ = eR - my_alg('eR_last');
                eL_differ = eL - my_alg('eL_last');

                my_alg('uR') = my_alg('uR') + (kp * eR + ki * my_alg('eR_sum') + kd * eR_differ) * my_alg('w2u');
                my_alg('uL') = my_alg('uL') + (kp * eL + ki * my_alg('eL_sum') + kd * eL_differ) * my_alg('w2u');

                my_alg('eR_last') = eR;
                my_alg('eL_last') = eL;

            else
                my_alg('uR') = my_alg('wR_set') * my_alg('w2u');
                my_alg('uL') = my_alg('wL_set') * my_alg('w2u');
            end

            % Limit pwm dutycycle
            if (my_alg('uR') > 1.0)
                my_alg('uR') = 1.0;
            elseif (my_alg('uR') <- 1.0)
                my_alg('uR') = -1.0;
            end

            if (my_alg('uL') > 1.0)
                my_alg('uL') = 1.0;
            elseif (my_alg('uL') <- 1.0)
                my_alg('uL') = -1.0;
            end

            % Apply pwm signal
            my_alg('right motor') = my_alg('uR');
            my_alg('left motor') = my_alg('uL');

            %% Save data for plot
            my_alg('t_all') = [my_alg('t_all') time];
            my_alg('uR_all') = [my_alg('uR_all') my_alg('uR')];
            my_alg('uL_all') = [my_alg('uL_all') my_alg('uL')];
            my_alg('wR_all') = [my_alg('wR_all') my_alg('right encoder')];
            my_alg('wL_all') = [my_alg('wL_all') my_alg('left encoder')];

            %% Save data for save
            my_alg('line_all') = [my_alg('line_all') my_alg('reflectance_raw').'];
        end

    else
        %% Finish algorithm and plot results

        % Stop motors
        my_alg('right motor') = 0;
        my_alg('left motor') = 0;
        % Stop session
        my_alg('is_done') = true;

        % Plot
        if (my_alg('plot_signal'))
            figure(2);
            plot(my_alg('t_all'), my_alg('uR_all')); hold on
            plot(my_alg('t_all'), my_alg('uL_all'));
            legend('Right motor', 'Left motor');
            xlabel('Time (t)');
            ylabel('Duty Cycle');
            title('PWM duty cycle');

            figure(3);
            plot(my_alg('t_all'), my_alg('wR_all')); hold on
            plot(my_alg('t_all'), my_alg('wL_all'));
            legend('Right encoder', 'Left encoder');
            xlabel('Time (t)');
            ylabel('Wheel Velocity (rad/s)');
            title('Wheel Angular Velocities');
        end

        line = my_alg('line_all');
        save td2.mat line;
    end

    return
