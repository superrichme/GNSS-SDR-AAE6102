% function [satPositions, satClkCorr] = satpos(transmitTime, prnList, ...
%                                              eph, settings) 
% %SATPOS Computation of satellite coordinates X,Y,Z at TRANSMITTIME for
% %given ephemeris EPH. Coordinates are computed for each satellite in the
% %list PRNLIST.
% %[satPositions, satClkCorr] = satpos(transmitTime, prnList, eph, settings);
% %
% %   Inputs:
% %       transmitTime  - transmission time
% %       prnList       - list of PRN-s to be processed
% %       eph           - ephemerides of satellites
% %       settings      - receiver settings
% %
% %   Outputs:
% %       satPositions  - position of satellites (in ECEF system [X; Y; Z;])
% %       satClkCorr    - correction of satellite clocks
% 
% %--------------------------------------------------------------------------
% %                           SoftGNSS v3.0
% %--------------------------------------------------------------------------
% %Based on Kai Borre 04-09-96
% %Copyright (c) by Kai Borre
% %Updated by Darius Plausinaitis, Peter Rinder and Nicolaj Bertelsen
% %
% % CVS record:
% % $Id: satpos.m,v 1.1.2.17 2007/01/30 09:45:12 dpl Exp $
% 
% %% Initialize constants ===================================================
% numOfSatellites = size(prnList, 2);
% 
% % GPS constatns
% 
% gpsPi          = 3.1415926535898;  % Pi used in the GPS coordinate 
%                                    % system
% 
% %--- Constants for satellite position calculation -------------------------
% Omegae_dot     = 7.2921151467e-5;  % Earth rotation rate, [rad/s]
% GM             = 3.986005e14;      % Universal gravitational constant times
%                                    % the mass of the Earth, [m^3/s^2]
% F              = -4.442807633e-10; % Constant, [sec/(meter)^(1/2)]
% 
% %% Initialize results =====================================================
% satClkCorr   = zeros(1, numOfSatellites);
% satPositions = zeros(3, numOfSatellites);
% 
% %% Process each satellite =================================================
% 
% for satNr = 1 : numOfSatellites
% 
%     prn = prnList(satNr);
% 
% %% Find initial satellite clock correction --------------------------------
% 
%     %--- Find time difference ---------------------------------------------
%     dt = check_t(transmitTime - eph(prn).t_oc);
% 
%     %--- Calculate clock correction ---------------------------------------
%     satClkCorr(satNr) = (eph(prn).a_f2 * dt + eph(prn).a_f1) * dt + ...
%                          eph(prn).a_f0 - ...
%                          eph(prn).T_GD;
% 
%     time = transmitTime - satClkCorr(satNr);
% 
% %% Find satellite's position ----------------------------------------------
% 
%     %Restore semi-major axis
%     a   = eph(prn).sqrtA * eph(prn).sqrtA;
% 
%     %Time correction
%     tk  = check_t(time - eph(prn).t_oe);
% 
%     %Initial mean motion
%     n0  = sqrt(GM / a^3);
%     %Mean motion
%     n   = n0 + eph(prn).deltan;
% 
%     %Mean anomaly
%     M   = eph(prn).M_0 + n * tk;
%     %Reduce mean anomaly to between 0 and 360 deg
%     M   = rem(M + 2*gpsPi, 2*gpsPi);
% 
%     %Initial guess of eccentric anomaly
%     E   = M;
% 
%     %--- Iteratively compute eccentric anomaly ----------------------------
%     for ii = 1:10
%         E_old   = E;
%         E       = M + eph(prn).e * sin(E);
%         dE      = rem(E - E_old, 2*gpsPi);
% 
%         if abs(dE) < 1.e-12
%             % Necessary precision is reached, exit from the loop
%             break;
%         end
%     end
% 
%     %Reduce eccentric anomaly to between 0 and 360 deg
%     E   = rem(E + 2*gpsPi, 2*gpsPi);
% 
%     %Compute relativistic correction term
%     dtr = F * eph(prn).e * eph(prn).sqrtA * sin(E);
% 
%     %Calculate the true anomaly
%     nu   = atan2(sqrt(1 - eph(prn).e^2) * sin(E), cos(E)-eph(prn).e);
% 
%     %Compute angle phi
%     phi = nu + eph(prn).omega;
%     %Reduce phi to between 0 and 360 deg
%     phi = rem(phi, 2*gpsPi);
% 
%     %Correct argument of latitude
%     u = phi + ...
%         eph(prn).C_uc * cos(2*phi) + ...
%         eph(prn).C_us * sin(2*phi);
%     %Correct radius
%     r = a * (1 - eph(prn).e*cos(E)) + ...
%         eph(prn).C_rc * cos(2*phi) + ...
%         eph(prn).C_rs * sin(2*phi);
%     %Correct inclination
%     i = eph(prn).i_0 + eph(prn).iDot * tk + ...
%         eph(prn).C_ic * cos(2*phi) + ...
%         eph(prn).C_is * sin(2*phi);
% 
%     %Compute the angle between the ascending node and the Greenwich meridian
%     Omega = eph(prn).omega_0 + (eph(prn).omegaDot - Omegae_dot)*tk - ...
%             Omegae_dot * eph(prn).t_oe;
%     %Reduce to between 0 and 360 deg
%     Omega = rem(Omega + 2*gpsPi, 2*gpsPi);
% 
%     %--- Compute satellite coordinates ------------------------------------
%     satPositions(1, satNr) = cos(u)*r * cos(Omega) - sin(u)*r * cos(i)*sin(Omega);
%     satPositions(2, satNr) = cos(u)*r * sin(Omega) + sin(u)*r * cos(i)*cos(Omega);
%     satPositions(3, satNr) = sin(u)*r * sin(i);
% 
% 
% %% Include relativistic correction in clock correction --------------------
%     satClkCorr(satNr) = (eph(prn).a_f2 * dt + eph(prn).a_f1) * dt + ...
%                          eph(prn).a_f0 - ...
%                          eph(prn).T_GD + dtr;
% 
% end % for satNr = 1 : numOfSatellites




function [satPositions, satClkCorr, satVelocities] = satpos(transmitTime, prnList, ...
                                                           eph, settings)
%SATPOS Computation of satellite coordinates X,Y,Z and velocities vx,vy,vz
%at TRANSMITTIME for given ephemeris EPH. Coordinates are computed for each
%satellite in the list PRNLIST.
%[satPositions, satClkCorr, satVelocities] = satpos(transmitTime, prnList, eph, settings);
%
%   Inputs:
%       transmitTime  - transmission time
%       prnList       - list of PRN-s to be processed
%       eph           - ephemerides of satellites
%       settings      - receiver settings
%
%   Outputs:
%       satPositions  - position of satellites (in ECEF system [X; Y; Z;])
%       satClkCorr    - correction of satellite clocks
%       satVelocities - velocities of satellites (in ECEF system [vx; vy; vz;])

%--------------------------------------------------------------------------
%                           SoftGNSS v3.0
%--------------------------------------------------------------------------
%Based on Kai Borre 04-09-96
%Copyright (c) by Kai Borre
%Updated by Darius Plausinaitis, Peter Rinder and Nicolaj Bertelsen
%
% CVS record:
% $Id: satpos.m,v 1.1.2.17 2007/01/30 09:45:12 dpl Exp $

%% Initialize constants ===================================================
numOfSatellites = size(prnList, 2);

% GPS constants
gpsPi          = 3.1415926535898;  % Pi used in the GPS coordinate system
Omegae_dot     = 7.2921151467e-5;  % Earth rotation rate, [rad/s]
GM             = 3.986005e14;      % Universal gravitational constant times the mass of the Earth, [m^3/s^2]
F              = -4.442807633e-10; % Constant, [sec/(meter)^(1/2)]

%% Initialize results =====================================================
satClkCorr   = zeros(1, numOfSatellites);
satPositions = zeros(3, numOfSatellites);
satVelocities = zeros(3, numOfSatellites); % Initialize velocity matrix

%% Process each satellite =================================================
for satNr = 1 : numOfSatellites
    
    prn = prnList(satNr);
    
    %% Find initial satellite clock correction --------------------------------
    %--- Find time difference ---------------------------------------------
    dt = check_t(transmitTime - eph(prn).t_oc);

    %--- Calculate clock correction ---------------------------------------
    satClkCorr(satNr) = (eph(prn).a_f2 * dt + eph(prn).a_f1) * dt + ...
                        eph(prn).a_f0 - eph(prn).T_GD;

    time = transmitTime - satClkCorr(satNr);

    %% Find satellite's position and velocity ----------------------------------
    % Restore semi-major axis
    a   = eph(prn).sqrtA * eph(prn).sqrtA;

    % Time correction
    tk  = check_t(time - eph(prn).t_oe);

    % Initial mean motion
    n0  = sqrt(GM / a^3);
    % Mean motion
    n   = n0 + eph(prn).deltan;

    % Mean anomaly
    M   = eph(prn).M_0 + n * tk;
    % Reduce mean anomaly to between 0 and 360 deg
    M   = rem(M + 2 * gpsPi, 2 * gpsPi);

    % Initial guess of eccentric anomaly
    E   = M;

    %--- Iteratively compute eccentric anomaly ----------------------------
    for ii = 1:10
        E_old   = E;
        E       = M + eph(prn).e * sin(E);
        dE      = rem(E - E_old, 2 * gpsPi);

        if abs(dE) < 1.e-12
            % Necessary precision is reached, exit from the loop
            break;
        end
    end
    % Reduce eccentric anomaly to between 0 and 360 deg
    E   = rem(E + 2 * gpsPi, 2 * gpsPi);

    % Compute relativistic correction term
    dtr = F * eph(prn).e * eph(prn).sqrtA * sin(E);

    % Calculate the true anomaly
    nu   = atan2(sqrt(1 - eph(prn).e^2) * sin(E), cos(E) - eph(prn).e);

    % Compute angle phi
    phi = nu + eph(prn).omega;
    % Reduce phi to between 0 and 360 deg
    phi = rem(phi, 2 * gpsPi);

    % Correct argument of latitude
    u = phi + eph(prn).C_uc * cos(2 * phi) + eph(prn).C_us * sin(2 * phi);
    % Correct radius
    r = a * (1 - eph(prn).e * cos(E)) + eph(prn).C_rc * cos(2 * phi) + eph(prn).C_rs * sin(2 * phi);
    % Correct inclination
    i = eph(prn).i_0 + eph(prn).iDot * tk + eph(prn).C_ic * cos(2 * phi) + eph(prn).C_is * sin(2 * phi);

    % Compute the angle between the ascending node and the Greenwich meridian
    Omega = eph(prn).omega_0 + (eph(prn).omegaDot - Omegae_dot) * tk - Omegae_dot * eph(prn).t_oe;
    % Reduce to between 0 and 360 deg
    Omega = rem(Omega + 2 * gpsPi, 2 * gpsPi);

    %--- Compute satellite coordinates ------------------------------------
    satPositions(1, satNr) = cos(u) * r * cos(Omega) - sin(u) * r * cos(i) * sin(Omega);
    satPositions(2, satNr) = cos(u) * r * sin(Omega) + sin(u) * r * cos(i) * cos(Omega);
    satPositions(3, satNr) = sin(u) * r * sin(i);

    %--- Compute satellite velocities -------------------------------------
    % Derivative of mean anomaly with respect to time
    dM_dt = n;

    % Derivative of eccentric anomaly
    dE_dt = dM_dt / (1 - eph(prn).e * cos(E));

    % Derivative of true anomaly
    dnu_dt = (dE_dt * sqrt(1 - eph(prn).e^2)) / (1 - eph(prn).e * cos(E));

    % Derivative of argument of latitude
    du_dt = dnu_dt + 2 * (eph(prn).C_us * cos(2 * phi) - eph(prn).C_uc * sin(2 * phi)) * dnu_dt;

    % Derivative of radius
    dr_dt = a * eph(prn).e * sin(E) * dE_dt + 2 * (eph(prn).C_rs * cos(2 * phi) - eph(prn).C_rc * sin(2 * phi)) * dnu_dt;

    % Derivative of inclination
    di_dt = eph(prn).iDot + 2 * (eph(prn).C_is * cos(2 * phi) - eph(prn).C_ic * sin(2 * phi)) * dnu_dt;

    % Derivative of Omega (considering Earth rotation)
    dOmega_dt = eph(prn).omegaDot - Omegae_dot;

    % Velocity components in orbital plane
    vx_orb = dr_dt * cos(u) - r * du_dt * sin(u);
    vy_orb = dr_dt * sin(u) + r * du_dt * cos(u);
    vz_orb = r * di_dt;

    % Rotate to ECEF frame
    satVelocities(1, satNr) = (vx_orb * cos(Omega) - vy_orb * sin(Omega) * cos(i) - vz_orb * sin(i) * sin(Omega)) * dOmega_dt;
    satVelocities(2, satNr) = (vx_orb * sin(Omega) + vy_orb * cos(Omega) * cos(i) + vz_orb * sin(i) * cos(Omega)) * dOmega_dt;
    satVelocities(3, satNr) = (vy_orb * sin(i) + vz_orb * cos(i)) * dOmega_dt;

    % Adjust for Earth's rotation (simplified, considering tk)
    satVelocities(:, satNr) = satVelocities(:, satNr) + [-Omegae_dot * satPositions(2, satNr); Omegae_dot * satPositions(1, satNr); 0];

    %% Include relativistic correction in clock correction --------------------
    satClkCorr(satNr) = (eph(prn).a_f2 * dt + eph(prn).a_f1) * dt + ...
                        eph(prn).a_f0 - eph(prn).T_GD + dtr;
                     
end % for satNr = 1 : numOfSatellites