function [navSolutions, eph] = postNavigation(trackResults, settings)
%Function calculates navigation solutions for the receiver (pseudoranges,
%positions). At the end it converts coordinates from the WGS84 system to
%the UTM, geocentric or any additional coordinate system.
%
%[navSolutions, eph] = postNavigation(trackResults, settings)
%
%   Inputs:
%       trackResults    - results from the tracking function (structure
%                       array).
%       settings        - receiver settings.
%   Outputs:
%       navSolutions    - contains measured pseudoranges, receiver
%                       clock error, receiver coordinates in several
%                       coordinate systems (at least ECEF and UTM).
%       eph             - received ephemerides of all SV (structure array).

%--------------------------------------------------------------------------
%                           SoftGNSS v3.0
% 
% Copyright (C) Darius Plausinaitis
% Written by Darius Plausinaitis with help from Kristin Larson
%--------------------------------------------------------------------------
%This program is free software; you can redistribute it and/or
%modify it under the terms of the GNU General Public License
%as published by the Free Software Foundation; either version 2
%of the License, or (at your option) any later version.
%
%This program is distributed in the hope that it will be useful,
%but WITHOUT ANY WARRANTY; without even the implied warranty of
%MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%GNU General Public License for more details.
%
%You should have received a copy of the GNU General Public License
%along with this program; if not, write to the Free Software
%Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
%USA.
%--------------------------------------------------------------------------

%CVS record:
%$Id: postNavigation.m,v 1.1.2.22 2006/08/09 17:20:11 dpl Exp $

%% Check is there enough data to obtain any navigation solution ===========
% It is necessary to have at least three subframes (number 1, 2 and 3) to
% find satellite coordinates. Then receiver position can be found too.
% The function requires all 5 subframes, because the tracking starts at
% arbitrary point. Therefore the first received subframes can be any three
% from the 5.
% One subframe length is 6 seconds, therefore we need at least 30 sec long
% record (5 * 6 = 30 sec = 30000ms). We add extra seconds for the cases,
% when tracking has started in a middle of a subframe.
%///@ Jr9910: 2013/04/25，确定处理的数据长度是否包含5个子帧以及四颗可见卫星
if (settings.msToProcess < 36000) || (sum([trackResults.status] ~= '-') < 4)
    % Show the error message and exit
    disp('Record is to short or too few satellites tracked. Exiting!');
    navSolutions = [];
    eph          = [];
    return
end

%% Find preamble start positions ==========================================
%///@ Jr9910: 2013/04/25，确定子帧起始位置并返回帧同步成功的通道
[subFrameStart, activeChnList] = findPreambles(trackResults, settings);

%% Decode ephemerides =====================================================

for channelNr = activeChnList

    %=== Convert tracking output to navigation bits =======================

    %--- Copy 5 sub-frames long record from tracking output ---------------
    navBitsSamples = trackResults(channelNr).I_P(subFrameStart(channelNr) - 20 : ...
                               subFrameStart(channelNr) + (1500 * 20) -1)';

    %--- Group every 20 vales of bits into columns ------------------------
    navBitsSamples = reshape(navBitsSamples, ...
                             20, (size(navBitsSamples, 1) / 20));

    %--- Sum all samples in the bits to get the best estimate -------------
    navBits = sum(navBitsSamples);

    %--- Now threshold and make 1 and 0 -----------------------------------
    % The expression (navBits > 0) returns an array with elements set to 1
    % if the condition is met and set to 0 if it is not met.
    navBits = (navBits > 0);

    %--- Convert from decimal to binary -----------------------------------
    % The function ephemeris expects input in binary form. In Matlab it is
    % a string array containing only "0" and "1" characters.
    navBitsBin = dec2bin(navBits);
    
    %=== Decode ephemerides and TOW of the first sub-frame
    %================%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [eph(trackResults(channelNr).PRN), TOW] = ...%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%eph(trackResults(channelNr).PRN
                            ephemeris(navBitsBin(2:1501)', navBitsBin(1));

    %--- Exclude satellite if it does not have the necessary nav data -----
    if (isempty(eph(trackResults(channelNr).PRN).IODC) || ...
        isempty(eph(trackResults(channelNr).PRN).IODE_sf2) || ...
        isempty(eph(trackResults(channelNr).PRN).IODE_sf3))

        %--- Exclude channel from the list (from further processing) ------
        activeChnList = setdiff(activeChnList, channelNr);
    end    
end

%% Check if the number of satellites is still above 3 =====================
if (isempty(activeChnList) || (size(activeChnList, 2) < 4))
    % Show error message and exit
    disp('Too few satellites with ephemeris data for postion calculations. Exiting!');
    navSolutions = [];
    eph          = [];
    return
end

%% Initialization =========================================================

% Set the satellite elevations array to INF to include all satellites for
% the first calculation of receiver position. There is no reference point
% to find the elevation angle as there is no receiver position estimate at
% this point.
satElev  = inf(1, settings.numberOfChannels);

% Save the active channel list. The list contains satellites that are
% tracked and have the required ephemeris data. In the next step the list
% will depend on each satellite's elevation angle, which will change over
% time.  
readyChnList = activeChnList;

%///@ Jr9910: 2013/04/25，软件接收机当中，发射时间认为是同一时刻，接收时间因为传输距离的不同而不同
transmitTime = TOW;

%##########################################################################
%#   Do the satellite and receiver position calculations                  #
%##########################################################################

%% Initialization of current measurement ==================================
%///@ Jr9910：2013/04/23，相当于确定要解算多少次
for currMeasNr = 1:fix((settings.msToProcess - max(subFrameStart)) / ...
                                                     settings.navSolPeriod)

    % Exclude satellites, that are belove elevation mask 
    %///@ Jr9910: 2013/04/23，提取既满足仰角条件，又成功实现电文解译的卫星
    activeChnList = intersect(find(satElev >= settings.elevationMask), ...
                              readyChnList);

    % Save list of satellites used for position calculation
    navSolutions.channel.PRN(activeChnList, currMeasNr) = ...
                                        [trackResults(activeChnList).PRN]; 

    % These two lines help the skyPlot function. The satellites excluded
    % do to elevation mask will not "jump" to possition (0,0) in the sky
    % plot.
    navSolutions.channel.el(:, currMeasNr) = ...
                                         NaN(settings.numberOfChannels, 1);
    navSolutions.channel.az(:, currMeasNr) = ...
                                         NaN(settings.numberOfChannels, 1);

%% Find pseudoranges ======================================================
%///@ Jr9910: 2013/04/25，navSolPeriod以ms为单位
navSolutions.channel.rawP(:, currMeasNr) = calculatePseudoranges(...
        trackResults, ...
        subFrameStart + settings.navSolPeriod * (currMeasNr-1), ...
        activeChnList, settings);

%% Find satellites positions and clocks corrections =======================
[satPositions, satClkCorr, satVelocities] = satpos(transmitTime, ...
                                                  [trackResults(activeChnList).PRN], ...
                                                  eph, settings);

%% Extract Doppler measurements
doppler = zeros(1, length(activeChnList));
% 计算当前测量时刻的索引（毫秒）
measTime = subFrameStart + settings.navSolPeriod * (currMeasNr-1);
for i = 1:length(activeChnList)
    chn = activeChnList(i);
    % 确保 measTime 在 trackResults 的范围内
    idx = min(max(round(measTime(chn)), 1), length(trackResults(chn).carrFreq));
    % 提取对应时间点的载波频率
    doppler(i) = trackResults(chn).carrFreq(idx) - 1575.42e6;
end

%% Find receiver position and velocity ===================================
if size(activeChnList, 2) > 3
    [xyzdt, vel, ...
     navSolutions.channel.el(activeChnList, currMeasNr), ...
     navSolutions.channel.az(activeChnList, currMeasNr), ...
     navSolutions.DOP(:, currMeasNr)] = ...
        leastSquarePos(satPositions, satVelocities, doppler, ...
                       navSolutions.channel.rawP(activeChnList, currMeasNr)' + satClkCorr * settings.c, ...
                       settings);

    % Store position and clock error
    navSolutions.X(currMeasNr)  = xyzdt(1);
    navSolutions.Y(currMeasNr)  = xyzdt(2);
    navSolutions.Z(currMeasNr)  = xyzdt(3);
    navSolutions.dt(currMeasNr) = xyzdt(4);

    % Store velocity
    navSolutions.Vx(currMeasNr) = vel(1);
    navSolutions.Vy(currMeasNr) = vel(2);
    navSolutions.Vz(currMeasNr) = vel(3);

    % Update the satellites elevations vector
    satElev = navSolutions.channel.el(:, currMeasNr)';

        %=== Correct pseudorange measurements for clocks errors ===========
        navSolutions.channel.correctedP(activeChnList, currMeasNr) = ...
                navSolutions.channel.rawP(activeChnList, currMeasNr) + ...
                satClkCorr' * settings.c + navSolutions.dt(currMeasNr);

%% Coordinate conversion ==================================================

        %=== Convert to geodetic coordinates ==============================
        [navSolutions.latitude(currMeasNr), ...
         navSolutions.longitude(currMeasNr), ...
         navSolutions.height(currMeasNr)] = cart2geo(...
                                            navSolutions.X(currMeasNr), ...
                                            navSolutions.Y(currMeasNr), ...
                                            navSolutions.Z(currMeasNr), ...
                                            5);

        %=== Convert to UTM coordinate system =============================
        navSolutions.utmZone = findUtmZone(navSolutions.latitude(currMeasNr), ...
                                           navSolutions.longitude(currMeasNr));
        
        [navSolutions.E(currMeasNr), ...
         navSolutions.N(currMeasNr), ...
         navSolutions.U(currMeasNr)] = cart2utm(xyzdt(1), xyzdt(2), ...
                                                xyzdt(3), ...
                                                navSolutions.utmZone);
        
    else % if size(activeChnList, 2) > 3 
        %--- There are not enough satellites to find 3D position ----------
        disp(['   Measurement No. ', num2str(currMeasNr), ...
                       ': Not enough information for position solution.']);

        %--- Set the missing solutions to NaN. These results will be
        %excluded automatically in all plots. For DOP it is easier to use
        %zeros. NaN values might need to be excluded from results in some
        %of further processing to obtain correct results.
        navSolutions.X(currMeasNr)           = NaN;
        navSolutions.Y(currMeasNr)           = NaN;
        navSolutions.Z(currMeasNr)           = NaN;
        navSolutions.dt(currMeasNr)          = NaN;
        navSolutions.DOP(:, currMeasNr)      = zeros(5, 1);
        navSolutions.latitude(currMeasNr)    = NaN;
        navSolutions.longitude(currMeasNr)   = NaN;
        navSolutions.height(currMeasNr)      = NaN;
        navSolutions.E(currMeasNr)           = NaN;
        navSolutions.N(currMeasNr)           = NaN;
        navSolutions.U(currMeasNr)           = NaN;

        navSolutions.channel.az(activeChnList, currMeasNr) = ...
                                             NaN(1, length(activeChnList));
        navSolutions.channel.el(activeChnList, currMeasNr) = ...
                                             NaN(1, length(activeChnList));

        % TODO: Know issue. Satellite positions are not updated if the
        % satellites are excluded do to elevation mask. Therefore rasing
        % satellites will be not included even if they will be above
        % elevation mask at some point. This would be a good place to
        % update positions of the excluded satellites.

    end % if size(activeChnList, 2) > 3

    %=== Update the transmit time ("measurement time") ====================
    %///@ Jr9910: 2013/04/25，更新发射时间，也就是测量时间
    transmitTime = transmitTime + settings.navSolPeriod / 1000;
save('eph.mat','eph');
load('eph.mat');

end %for currMeasNr...
%=== 绘制导航结果 ===================================================
% 创建时间向量 (单位: 秒)
time = (1:length(navSolutions.X)) * settings.navSolPeriod / 1000;

% 1. 绘制 2D 位置轨迹 (纬度-经度)
figure('Name', '接收机位置轨迹');
subplot(2, 1, 1);
plot(navSolutions.longitude, navSolutions.latitude, 'b-', 'LineWidth', 1.5);
xlabel('经度 (度)');
ylabel('纬度 (度)');
title('接收机 2D 位置轨迹');
grid on;

% 2. 绘制高度随时间变化
subplot(2, 1, 2);
plot(time, navSolutions.height, 'r-', 'LineWidth', 1.5);
xlabel('times (s)');
ylabel('Variations (m)');
title('Coordinates variations in UTM system');
grid on;

% 如果已添加速度估计，可继续绘制速度
if isfield(navSolutions, 'Vx')
    figure('Name', '接收机速度估计');
    subplot(3, 1, 1);
    plot(time, navSolutions.Vx, 'b-', 'LineWidth', 1.5);
    xlabel('时间 (s)');
    ylabel('Vx (m/s)');
    title('接收机速度 (ECEF 坐标)');
    grid on;

    subplot(3, 1, 2);
    plot(time, navSolutions.Vy, 'r-', 'LineWidth', 1.5);
    xlabel('时间 (s)');
    ylabel('Vy (m/s)');
    grid on;

    subplot(3, 1, 3);
    plot(time, navSolutions.Vz, 'g-', 'LineWidth', 1.5);
    xlabel('时间 (s)');
    ylabel('Vz (m/s)');
    grid on;
end

