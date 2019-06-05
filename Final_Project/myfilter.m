function [xhat, meas] = myfilter(calAcc, calGyr, calMag)
% FILTERTEMPLATE  Filter template
%
% This is a template function for how to collect and filter data
% sent from a smartphone live.  Calibration data for the
% accelerometer, gyroscope and magnetometer assumed available as
% structs with fields m (mean) and R (variance).
%
% The function returns xhat as an array of structs comprising t
% (timestamp), x (state), and P (state covariance) for each
% timestamp, and meas an array of structs comprising t (timestamp),
% acc (accelerometer measurements), gyr (gyroscope measurements),
% mag (magnetometer measurements), and orint (orientation quaternions
% from the phone).  Measurements not availabe are marked with NaNs.
%
% As you implement your own orientation estimate, it will be
% visualized in a simple illustration.  If the orientation estimate
% is checked in the Sensor Fusion app, it will be displayed in a
% separate view.
%
% Note that it is not necessary to provide inputs (calAcc, calGyr, calMag).

%% Setup necessary infrastructure
import('com.liu.sensordata.*');  % Used to receive data.

%% Filter settings
t0 = [];  % Initial time (initialize on first data received)
nx = 4;  % [orientation_quat bias_w bias_acc ]

% Add your filter settings here.

% Rw = eye(3)*1e-3;
% Ra = eye(3)*1e-1;
% Rm = eye(3)*1e-6;
Ra= 1.0e-03 * ...
    [0.0489    0.0132   -0.0118
     0.0132    0.0688   -0.0131
    -0.0118   -0.0131    0.2227];
Rm = [0.0273    0.0213   -0.0069
      0.0213    0.1144   -0.0266
     -0.0069   -0.0266    0.1134] * 1e-1;
Rw = 1e-5 * ...
    [0.0887    0.0173    0.0003
     0.0173    0.0930    0.0144
     0.0003    0.0144    0.1240];


g0 = [0 0 9.8]';
rangeSkipAcc = 0.2;  % accept acc meas. if  80%|g0| < |acc| < 120%|g0|
rangeSkipMag = 0.1;
m0 = [0 17.7 -45.4]';
L = norm(m0);
alpha = 0.01;

% Current filter state.
x = [1;0;0;0];
P = eye(4);

% Saved filter states.
xhat = struct(  't', zeros(1, 0),...
                'x', zeros(nx, 0),...
                'P', zeros(nx, nx, 0));

meas = struct(  't', zeros(1, 0),...
                'acc', zeros(3, 0),...
                'gyr', zeros(3, 0),...
                'mag', zeros(3, 0),...
                'orient', zeros(4, 0));
try
    %% Create data link
    server = StreamSensorDataReader(3400);
    % Makes sure to resources are returned.
    sentinel = onCleanup(@() server.stop());
    
    server.start();  % Start data reception.
    
    % Used for visualization.
    figure(1);
    subplot(1, 2, 1);
    ownView = OrientationView('Own filter', gca);  % Used for visualization.
    googleView = [];
    counter = 0;  % Used to throttle the displayed frame rate.
    
    %% Filter loop
    while server.status()  % Repeat while data is available
        % Get the next measurement set, assume all measurements
        % within the next 5 ms are concurrent (suitable for sampling
        % in 100Hz).
        data = server.getNext(5);
        
        if isnan(data(1))  % No new data received
            continue;        % Skips the rest of the look
        end
        t = data(1)/1000;  % Extract current time
        
        if isempty(t0)  % Initialize t0
            t0 = t;
        end
        
        
        gyr = data(1, 5:7)';
        if ~any(isnan(gyr))  % Gyro measurements are available.
            [x, P] = tu_qw( x, P, gyr, 0.01, Rw);
            [x, P] = mu_normalizeQ(x, P);
        else
            Rq = eye(4)*0.1;
            [x, P] = tu_qw_randwalk( x, P, Rq);
            [x, P] = mu_normalizeQ(x, P);
        end
        
        acc = data(1, 2:4)';
        if ~any(isnan(acc))  % Acc measurements are available.
            if abs(norm(acc)-norm(g0)) < norm(g0)*rangeSkipAcc
                [x, P] = mu_g(x, P, acc, Ra, g0);
                [x, P] = mu_normalizeQ(x, P);
                ownView.setAccDist(0);
            else
                % skip measurement update
                ownView.setAccDist(1);
            end
        end
        
        mag = data(1, 8:10)';
        if ~any(isnan(mag))  % Mag measurements are available.
            L = (1-alpha)*L+alpha*norm(mag);
            if abs(L-norm(mag)) < L*rangeSkipMag
                [x, P] = mu_m(x, P, mag, Rm, m0);
                [x, P] = mu_normalizeQ(x, P);
                ownView.setMagDist(0);
            else
                ownView.setMagDist(1);
            end
        end
        
        orientation = data(1, 18:21)';  % Google's orientation estimate.
        
        % Visualize result
        if rem(counter, 10) == 0
            setOrientation(ownView, x(1:4));
            title(ownView, 'OWN', 'FontSize', 16);
            if ~any(isnan(orientation))
                if isempty(googleView)
                    subplot(1, 2, 2);
                    % Used for visualization.
                    googleView = OrientationView('Google filter', gca);
                end
                setOrientation(googleView, orientation);
                title(googleView, 'GOOGLE', 'FontSize', 16);
            end
        end
        counter = counter + 1;
        
        % Save estimates
        xhat.x(:, end+1)    = x;
        xhat.P(:, :, end+1) = P;
        xhat.t(end+1)       = t - t0;
        
        meas.t(end+1)       = t - t0;
        meas.acc(:, end+1)  = acc;
        meas.gyr(:, end+1)  = gyr;
        meas.mag(:, end+1)  = mag;
        meas.orient(:, end+1) = orientation;
    end
catch e
    fprintf(['Unsuccessful connecting to client!\n' ...
        'Make sure to start streaming from the phone *after*'...
        'running this function!']);
end
end
