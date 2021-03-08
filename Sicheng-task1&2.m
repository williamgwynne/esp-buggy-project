function my_alg = task1_2(my_alg, robot)
% This function applies a pwm signal to both wheels for 1 second.
% Then, it stops.
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
    my_alg('width_robot') = 0.18;
    my_alg('radius_wheel') = 0.05;
    my_alg('pi') = 3.1415926;
    my_alg('w2p_ratio') = 901/12500;
    
    
    my_alg('dc_motor_signal_mode') = 'voltage_pwm';     % change if necessary to 'omega_setpoint'
    
    % Initialise vectors for saving velocity data
    my_alg('wR_all') = [];
    my_alg('wL_all') = [];
    my_alg('ur_all') = [];
    my_alg('ul_all') = [];
        
    % Initialise time parameters
    my_alg('t_sampling') = 0.03;
    my_alg('t_loop') = tic;
    my_alg('t_finish') = 40;
end

%% Loop code runs here

time = toc(my_alg('tic'));       % Get time since start of session

if time < my_alg('t_finish')     % Check for algorithm finish time
    
    dt = toc(my_alg('t_loop'));
    
    if dt>my_alg('t_sampling')   % execute code when desired sampling time is reached
        my_alg('t_loop') = tic;
        
        %% Add your loop code here 
        if((4 <= time) && (time < 5) || (9 <= time) && (time < 10) || (14 <= time) && (time < 15))
            
            ul = ((my_alg('width_robot') * my_alg('pi')/2)/2)/ my_alg('radius_wheel') * my_alg('w2p_ratio');
            ur = -((my_alg('width_robot') * my_alg('pi')/2)/2)/ my_alg('radius_wheel') * my_alg('w2p_ratio');
            
        elseif((19 <= time) && (time < 20))
                
            ul = (my_alg('width_robot') * my_alg('pi')/2)/ my_alg('radius_wheel') * my_alg('w2p_ratio');
            ur = -(my_alg('width_robot') * my_alg('pi')/2)/ my_alg('radius_wheel') * my_alg('w2p_ratio');
         
        elseif((24 <= time) && (time < 25) || (29 <= time) && (time < 30) || (34 <= time) && (time < 35))
                
            ul = -((my_alg('width_robot') * my_alg('pi')/2)/2)/ my_alg('radius_wheel') * my_alg('w2p_ratio');
            ur = ((my_alg('width_robot') * my_alg('pi')/2)/2)/ my_alg('radius_wheel') * my_alg('w2p_ratio');   
         
        else
            ul = 1;
            ur = 1;
        
        end
              
        % Apply pwm signal (range is [-1,1])
        my_alg('right motor') = ur;
        my_alg('left motor') = ul;

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
    
    % Plot saved velocities for right and left wheel
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
