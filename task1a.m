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
    
    
    my_alg('dc_motor_signal_mode') = 'voltage_pwm';     % change if necessary to 'omega_setpoint'
    
    % Initialise wheel angular velocity contollers
    my_alg('wR_set') = 5;
    my_alg('wL_set') = 5;
    
    % Initialise speed contoller parameters
    my_alg('Integral_right') = 0;
    my_alg('Integral_left') = 0;

    my_alg('Kp')= 0.01;
    my_alg('Ki')= 0.33;
    
    % Initialise line sensor contoller parameters
    my_alg('Kp_line')= 1.87;
    my_alg('Ki_line')= 0.769;    
    my_alg('Kd_line')= 1.35;
    
    my_alg('Integral_l')=0;
    my_alg('pre')=0;
    my_alg('reflect') = [];
    my_alg('blank')=0;

    % Initialise time parameters
    my_alg('sampling_inner') = 0.003;
    my_alg('sampling_outer') = 0.05;
    
    my_alg('t_inner_loop') = tic;
    my_alg('t_outer_loop') = tic;
   
    
    % desired wheel velocity 
    my_alg('wR')=my_alg('wR_set');
    my_alg('wL')=my_alg('wL_set');
        
    my_alg('wRall')=[];
    my_alg('wLall')=[];    
    my_alg('uRall')=[];
    my_alg('uLall')=[]; 
    

end

time = toc(my_alg('tic'));      % Get time for plotting

% stops when it detects 0.47 meters of blank space
if my_alg('blank')< 0.47    
    
   % Outer Loop
    dt = toc(my_alg('t_outer_loop'));
    
    % Execute code when desired outer loop sampling time is reached
    if dt>my_alg('sampling_outer')  
        my_alg('t_outer_loop') = tic;
        
        % Start to calculate the blank distance when none of the sensors detects the line
        if my_alg('reflectance_raw')== 2500     
            my_alg('blank')=my_alg('blank')+dt*(my_alg('right encoder')*0.05+my_alg('left encoder')*0.05)/2;
            
             linesense = my_alg('Integral_l');
             my_alg('wR') = 5+linesense;
             my_alg('wL') = 5-linesense;            
             my_alg('wRall')=[my_alg('wRall') my_alg('wR')];
             my_alg('wLall')=[my_alg('wLall') my_alg('wL')];  
            
        else
            my_alg('blank')=0;
            
            % Line sensor controller
            linesense = my_alg('Kp_line')*my_alg('reflectance')+ my_alg('Integral_l')+my_alg('Ki_line')*dt*(my_alg('reflectance'))+my_alg('Kd_line')*(my_alg('reflectance')-my_alg('pre'))/dt;

            my_alg('Integral_l')=my_alg('Integral_l')+my_alg('Ki_line')*dt*my_alg('reflectance');
            my_alg('pre')=my_alg('reflectance');

            my_alg('wR') = 5+linesense;
            my_alg('wL') = 5-linesense;
            my_alg('wRall')=[my_alg('wRall') my_alg('wR')];
            my_alg('wLall')=[my_alg('wLall') my_alg('wL')];  

            my_alg('reflect')= [my_alg('reflect') my_alg('reflectance')];
            
        end
        
        
    end

    
    % Inner Loop
    dt = toc(my_alg('t_inner_loop'));
    
    if dt>my_alg('sampling_inner')  % Execute code when desired inner loop sampling time is reached 
        my_alg('t_inner_loop') = tic;

        % Right&Left wheel controller
        uR = my_alg('Kp')*(my_alg('wR')-my_alg('right encoder')) + my_alg('Integral_right')+my_alg('Ki')*dt*(my_alg('wR')-my_alg('right encoder'));
        uL = my_alg('Kp')*(my_alg('wL')-my_alg('left encoder')) + my_alg('Integral_left')+my_alg('Ki')*dt*(my_alg('wL')-my_alg('left encoder'));
            
        % update the integral term
        my_alg('Integral_right') = my_alg('Integral_right') + my_alg('Ki')*dt*(my_alg('wR')-my_alg('right encoder'));
        my_alg('Integral_left') = my_alg('Integral_left') + my_alg('Ki')*dt*(my_alg('wL')-my_alg('left encoder'));
            
        % Apply pwm signal
        my_alg('right motor') = uR;
        my_alg('left motor') = uL;
        
        my_alg('uRall')=[my_alg('uRall') my_alg('right encoder')];
        my_alg('uLall')=[my_alg('uLall') my_alg('left encoder')];  

    end

else
    
    % Stop motors
    my_alg('right motor') = 0;
    my_alg('left motor') = 0;
    
    % Plot line sensor output
    figure(1);    
    xtime = [1/length(my_alg('reflect')) * time : 1/length(my_alg('reflect')) *time:time];
    plot(xtime, my_alg('reflect'));
    legend('reflectance');
    ylabel('Reflectance');
    xlabel('Time(s)')
    title('')
    
    % Plot saved velocities for right and left wheel
    figure(2);    
    xtime = [1/length(my_alg('wRall')) * time : 1/length(my_alg('wRall')) *time:time];
    plot(xtime, my_alg('wRall'),'r');
    hold on
    xtime = [1/length(my_alg('wLall')) * time : 1/length(my_alg('wLall')) *time:time];
    plot(xtime, my_alg('wLall'),'b');
    legend('wR','wL');
    ylabel('Wheel Angular Velocity (rad/s)');
    xlabel('Time(s)')
    title('')
    
    figure(3);    
    xtime = [1/length(my_alg('uRall')) * time : 1/length(my_alg('uRall')) *time:time];
    plot(xtime, my_alg('uRall'),'r');
    hold on
    xtime = [1/length(my_alg('uLall')) * time : 1/length(my_alg('uLall')) *time:time];
    plot(xtime, my_alg('uLall'),'b');
    legend('uR','uL');
    ylabel('Speed control parameter value');
    xlabel('Time(s)')
    title('')

    
    % Stop session
    my_alg('is_done') = true;
    
end

return