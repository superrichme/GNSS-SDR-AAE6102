%function [pos, el, az, dop] = leastSquarePos(satpos, obs, settinx = A \ omcgs)
function [pos, vel, el, az, dop] = leastSquarePos(satpos, satvelocity, doppler, obs, settings)
%Function calculates the Least Square Solution.
%
%[pos, el, az, dop] = leastSquarePos(satpos, obs, settings);
%
%   Inputs:
%       satpos      - Satellites positions (in ECEF system: [X; Y; Z;] -
%                   one column per satellite)
%       obs         - Observations - the pseudorange measurements to each
%                   satellite:
%                   (e.g. [20000000 21000000 .... .... .... .... ....])
%       settings    - receiver settings
%
%   Outputs:
%       pos         - receiver position and receiver clock error 
%                   (in ECEF system: [X, Y, Z, dt]) 
%       el          - Satellites elevation angles (degrees)
%       az          - Satellites azimuth angles (degrees)
%       dop         - Dilutions Of Precision ([GDOP PDOP HDOP VDOP TDOP])

%--------------------------------------------------------------------------
%                           SoftGNSS v3.0
%--------------------------------------------------------------------------
%Based on Kai Borre
%Copyright (c) by Kai Borre
%Updated by Darius Plausinaitis, Peter Rinder and Nicolaj Bertelsen
%
% CVS record:
% $Id: leastSquarePos.m,v 1.1.2.12 2006/08/22 13:45:59 dpl Exp $
%==========================================================================

%=== Initialization =======================================================
nmbOfIterations = 7;

dtr     = pi/180;
pos     = zeros(4, 1);
X       = satpos;
nmbOfSatellites = size(satpos, 2);

A       = zeros(nmbOfSatellites, 4);
omc     = zeros(nmbOfSatellites, 1);
az      = zeros(1, nmbOfSatellites);
el      = az;

%=== Iteratively find receiver position ===================================
for iter = 1:nmbOfIterations
    W = zeros(nmbOfSatellites, nmbOfSatellites);
    for i = 1:nmbOfSatellites
        if iter == 1
            %--- Initialize variables at the first iteration --------------
            Rot_X = X(:, i);
            trop = 2;
            el(i) = 0; % 初始仰角设为0，第一次迭代不计算仰角
            W(i, i) = 1; % 第一次迭代权重设为1
        else
            %--- Update equations -----------------------------------------
            rho2 = (X(1, i) - pos(1))^2 + (X(2, i) - pos(2))^2 + ...
                   (X(3, i) - pos(3))^2;
            traveltime = sqrt(rho2) / settings.c ;

            %--- Correct satellite position (do to earth rotation) --------
            Rot_X = e_r_corr(traveltime, X(:, i));

            %--- Find the elevation angel of the satellite ----------------
            [az(i), el(i), dist] = topocent(pos(1:3, :), Rot_X - pos(1:3, :));
            W(i, i) = max(0.1, sin(el(i) * dtr)^2);
            if (settings.useTropCorr == 1)
                %--- Calculate tropospheric correction --------------------
                trop = tropo(sin(el(i) * dtr), ...
                             0.0, 1013.0, 293.0, 50.0, 0.0, 0.0, 0.0);
            else
                % Do not calculate or apply the tropospheric corrections
                trop = 0;
            end
        end % if iter == 1 ... ... else 

        %--- Apply the corrections ----------------------------------------
        omc(i) = (obs(i) - norm(Rot_X - pos(1:3), 'fro') - pos(4) - trop);

        %--- Construct the A matrix ---------------------------------------
        A(i, :) =  [ (-(Rot_X(1) - pos(1))) / obs(i) ...
                     (-(Rot_X(2) - pos(2))) / obs(i) ...
                     (-(Rot_X(3) - pos(3))) / obs(i) ...
                     1 ];
    end % for i = 1:nmbOfSatellites

    % These lines allow the code to exit gracefully in case of any errors
    if rank(A) ~= 4
        pos     = zeros(1, 4);
        vel = zeros(1, 3);%============================================================================
        return
    end

    %--- Find position update ---------------------------------------------
    %x   = A \ omc;
    x = (A' * W * A) \ (A' * W * omc);
    %--- Apply position update --------------------------------------------
    pos = pos + x;
    
end % for iter = 1:nmbOfIterations

pos = pos';
%=== 使用最小二乘法计算接收机速度
%====================================================================================================
% 初始化速度输出
vel = zeros(3, 1);

% 确保多普勒和卫星速度数据可用
if ~isempty(doppler) && ~isempty(satvelocity)
    % 计算 GPS L1 信号波长 (1575.42 MHz)
    lamda = settings.c / 1575.42e6;

    % 将多普勒频率转换为伪距率 (m/s)
    rate = -lamda * doppler;
    rate = rate'; % 确保正确方向 (列向量)

    % 确保卫星速度方向正确
    satvelocity = satvelocity'; % [vx; vy; vz] 每颗卫星

    % 初始化残差伪距率向量
    b = zeros(nmbOfSatellites, 1);
    Wv = zeros(nmbOfSatellites, nmbOfSatellites);
    % 计算残差伪距率
    for i = 1:nmbOfSatellites
        b(i) = rate(i) - satvelocity(i, :) * (A(i, 1:3))';
        Wv(i, i) = max(0.1, sin(el(i) * dtr)^2);
    end

    % 假设单位协方差矩阵 C 简化计算
    %C = eye(nmbOfSatellites);

    % 使用最小二乘法求解速度
    %vel = (A(:, 1:3)' * C * A(:, 1:3)) \ (A(:, 1:3)' * C * b);
    vel = (A(:, 1:3)' * Wv * A(:, 1:3)) \ (A(:, 1:3)' * Wv * b);
end%========================================================================================================
%=== Calculate Dilution Of Precision ======================================
if nargout  == 4 || nargout == 5
    %--- Initialize output ------------------------------------------------
    dop     = zeros(1, 5);
    
    %--- Calculate DOP ----------------------------------------------------
    %Q       = inv(A'*A);
    Q = inv(A' * W * A);
    dop(1)  = sqrt(trace(Q));                       % GDOP    
    dop(2)  = sqrt(Q(1,1) + Q(2,2) + Q(3,3));       % PDOP
    dop(3)  = sqrt(Q(1,1) + Q(2,2));                % HDOP
    dop(4)  = sqrt(Q(3,3));                         % VDOP
    dop(5)  = sqrt(Q(4,4));                         % TDOP
end



% =====================EKF====================================
% function [pos, vel, el, az, dop] = leastSquarePos(satpos, satvelocity, doppler, obs, settings)
% % Function uses Extended Kalman Filter (EKF) to estimate receiver position and velocity.
% %
% % [pos, vel, el, az, dop] = leastSquarePos(satpos, satvelocity, doppler, obs, settings);
% %
% %   Inputs:
% %       satpos      - Satellites positions (in ECEF system: [X; Y; Z;] - one column per satellite)
% %       satvelocity - Satellites velocities (in ECEF system: [VX; VY; VZ;])
% %       doppler     - Doppler measurements (Hz)
% %       obs         - Pseudorange measurements (meters)
% %       settings    - Receiver settings (includes c: speed of light)
% %
% %   Outputs:
% %       pos         - Receiver position and clock error [X, Y, Z, dt]
% %       vel         - Receiver velocity [VX, VY, VZ]
% %       el          - Satellites elevation angles (degrees)
% %       az          - Satellites azimuth angles (degrees)
% %       dop         - Dilutions Of Precision [GDOP PDOP HDOP VDOP TDOP]
% 
% %=== Initialization =======================================================
% dtr = pi/180; % Degrees to radians conversion factor
% X = satpos; % Satellite positions
% nmbOfSatellites = size(satpos, 2); % Number of satellites
% az = zeros(1, nmbOfSatellites); % Azimuth angles
% el = az; % Elevation angles
% 
% % EKF state vector: [x, y, z, vx, vy, vz, dt, ddt] (position, velocity, clock bias, clock drift)
% x = zeros(8, 1); % Initial state
% P = eye(8) * 1e4; % Initial covariance (large uncertainty)
% 
% % Process noise covariance (Q)
% sigma_pos = 5; % Position noise (m)
% sigma_vel = 0.1; % Velocity noise (m/s)
% sigma_clock = 1e-2; % Clock bias noise (m)
% sigma_drift = 1e-4; % Clock drift noise (m/s)
% Q = diag([sigma_pos^2 * ones(1, 3), sigma_vel^2 * ones(1, 3), sigma_clock^2, sigma_drift^2]);
% 
% % Measurement noise covariance (R)
% sigma_pr = 5; % Pseudorange noise (m)
% sigma_dop = 0.5; % Doppler noise (m/s)
% R = diag([sigma_pr^2 * ones(1, nmbOfSatellites), sigma_dop^2 * ones(1, nmbOfSatellites)]);
% 
% % State transition matrix (F) - assuming constant velocity model
% dt = 1; % Time step (assuming 1 second between updates)
% F = eye(8);
% F(1:3, 4:6) = eye(3) * dt; % Position update with velocity
% F(7, 8) = dt; % Clock bias update with clock drift
% 
% %=== Extended Kalman Filter (EKF) to estimate position and velocity =========
% if ~isempty(doppler) && ~isempty(satvelocity)
%     % Prepare Doppler measurements
%     lambda = settings.c / 1575.42e6; % GPS L1 wavelength
%     doppler_rate = -lambda * doppler'; % Convert Doppler to pseudorange rate (m/s)
%     satvelocity = satvelocity'; % [vx; vy; vz] per satellite
% 
%     % Measurement vector: [pseudoranges; Doppler rates]
%     z = [obs'; doppler_rate];
% 
%     % EKF iteration (single step for simplicity)
%     % 1. Predict (Time Update)
%     x_pred = F * x; % State prediction
%     P_pred = F * P * F' + Q; % Covariance prediction
% 
%     % 2. Measurement Update
%     H = zeros(2 * nmbOfSatellites, 8); % Jacobian matrix
%     h = zeros(2 * nmbOfSatellites, 1); % Predicted measurements
% 
%     % Compute predicted measurements and Jacobian
%     for i = 1:nmbOfSatellites
%         % Correct satellite position for earth rotation
%         rho = sqrt((X(1, i) - x_pred(1))^2 + (X(2, i) - x_pred(2))^2 + (X(3, i) - x_pred(3))^2);
%         traveltime = rho / settings.c;
%         Rot_X = e_r_corr(traveltime, X(:, i));
% 
%         % Compute elevation and azimuth for weights
%         [az(i), el(i), dist] = topocent(x_pred(1:3), Rot_X - x_pred(1:3));
% 
%         % Tropospheric correction
%         if settings.useTropCorr == 1
%             trop = tropo(sin(el(i) * dtr), 0.0, 1013.0, 293.0, 50.0, 0.0, 0.0, 0.0);
%         else
%             trop = 0;
%         end
% 
%         % Predicted pseudorange
%         h(i) = norm(Rot_X - x_pred(1:3)) + x_pred(7) + trop;
% 
%         % Predicted Doppler (pseudorange rate)
%         los = (Rot_X - x_pred(1:3)) / norm(Rot_X - x_pred(1:3)); % Line-of-sight vector
%         h(i + nmbOfSatellites) = los' * (satvelocity(i, :)' - x_pred(4:6)) + x_pred(8);
% 
%         % Jacobian for pseudorange
%         H(i, 1:3) = -(Rot_X - x_pred(1:3))' / norm(Rot_X - x_pred(1:3)); % Partial derivative w.r.t. position
%         H(i, 7) = 1; % Partial derivative w.r.t. clock bias
% 
%         % Jacobian for Doppler
%         H(i + nmbOfSatellites, 4:6) = -los'; % Partial derivative w.r.t. velocity
%         H(i + nmbOfSatellites, 8) = 1; % Partial derivative w.r.t. clock drift
%     end
% 
%     % Innovation
%     y = z - h;
% 
%     % Innovation covariance
%     S = H * P_pred * H' + R;
% 
%     % Kalman gain
%     K = P_pred * H' / S;
% 
%     % Update state and covariance
%     x = x_pred + K * y;
%     P = (eye(8) - K * H) * P_pred;
% end
% 
% % Extract position and velocity
% pos = x(1:4)'; % [X, Y, Z, dt]
% vel = x(4:6)'; % [VX, VY, VZ]
% 
% %=== Calculate Dilution Of Precision ======================================
% if nargout == 4 || nargout == 5
%     dop = zeros(1, 5);
%     H_dop = H(1:nmbOfSatellites, 1:4); % Use position-related Jacobian for DOP
%     Q = inv(H_dop' * H_dop);
%     dop(1) = sqrt(trace(Q));                % GDOP
%     dop(2) = sqrt(Q(1,1) + Q(2,2) + Q(3,3)); % PDOP
%     dop(3) = sqrt(Q(1,1) + Q(2,2));         % HDOP
%     dop(4) = sqrt(Q(3,3));                  % VDOP
%     dop(5) = sqrt(Q(4,4));                  % TDOP
% end
