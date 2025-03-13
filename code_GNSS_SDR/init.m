% %--------------------------------------------------------------------------
% %                           SoftGNSS v3.0
% % 
% % Copyright (C) Darius Plausinaitis and Dennis M. Akos
% % Written by Darius Plausinaitis and Dennis M. Akos
% %--------------------------------------------------------------------------
% %This program is free software; you can redistribute it and/or
% %modify it under the terms of the GNU General Public License
% %as published by the Free Software Foundation; either version 2
% %of the License, or (at your option) any later version.
% %
% %This program is distributed in the hope that it will be useful,
% %but WITHOUT ANY WARRANTY; without even the implied warranty of
% %MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% %GNU General Public License for more details.
% %
% %You should have received a copy of the GNU General Public License
% %along with this program; if not, write to the Free Software
% %Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
% %USA.
% %--------------------------------------------------------------------------
% %
% %Script initializes settings and environment of the software receiver.
% %Then the processing is started.
% 
% %--------------------------------------------------------------------------
% % CVS record:
% % $Id: init.m,v 1.14.2.21 2006/08/22 13:46:00 dpl Exp $
% 
% %% Clean up the environment first =========================================
% clear; close all; clc;
% 
% format ('compact');
% format ('long', 'g');
% 
% %--- Include folders with functions ---------------------------------------
% addpath include             % The software receiver functions
% addpath geoFunctions        % Position calculation related functions
% 
% %% Print startup ==========================================================
% fprintf(['\n',...
%     'Welcome to:  softGNSS\n\n', ...
%     'An open source GNSS SDR software project initiated by:\n\n', ...
%     '              Danish GPS Center/Aalborg University\n\n', ...
%     'The code was improved by GNSS Laboratory/University of Colorado.\n\n',...
%     'The software receiver softGNSS comes with ABSOLUTELY NO WARRANTY;\n',...
%     'for details please read license details in the file license.txt. This\n',...
%     'is free software, and  you  are  welcome  to  redistribute  it under\n',...
%     'the terms described in the license.\n\n']);
% fprintf('                   -------------------------------\n\n');
% 
% %% Initialize constants, settings =========================================
% %///@ Jr9910: 2013/04/15，从initSetting中获取设置参数，并存储在settings结构体中
% settings = initSettings();
% 
% %% Generate plot of raw data and ask if ready to start processing =========
% try
%     fprintf('Probing data (%s)...\n', settings.fileName)
%     probeData(settings);
% catch
%     % There was an error, print it and exit
%     errStruct = lasterror;
%     disp(errStruct.message);
%     disp('  (run setSettings or change settings in "initSettings.m" to reconfigure)')    
%     return;
% end
% 
% disp('  Raw IF data plotted ')
% disp('  (run setSettings or change settings in "initSettings.m" to reconfigure)')
% disp(' ');
% gnssStart = input('Enter "1" to initiate GNSS processing or "0" to exit : ');
% 
% % if (gnssStart == 1)
% %     disp(' ');
% %     %start things rolling...
% %     postProcessing
% % end
% 
% if (gnssStart == 1)
%     disp(' ');
%     % Modified: Read 11ms of data from Opensky.bin and perform acquisition
%     fid = fopen(settings.fileName, 'r');
%     if fid == -1
%         error('Cannot open file: %s. Ensure the file exists in the root directory.', settings.fileName);
%     end
% 
%     % Calculate number of samples for 11ms
%     requiredSamples = round(11e-3 * settings.samplingFreq);
%     % Read 8-bit I/Q samples (2 channels: I and Q)
%     data = fread(fid, [2, requiredSamples], 'int8')';
%     longSignal = data(:, 1) + 1i * data(:, 2); % Convert to complex signal
%     fclose(fid);
% 
%     % Perform acquisition
%     acqResults = acquisition(longSignal, settings);
% 
%     % Visualize acquisition results
%     plotAcquisition(acqResults, settings);
% 
%     disp('Acquisition and visualization completed for Opensky.bin.');
% end
% 
% Script initializes settings and environment of the software receiver.
% Modified to process Opensky.bin data for Task 1 (Acquisition).

%% Clean up the environment first =========================================
clear; close all; clc;

format ('compact');
format ('long', 'g');

%--- Include folders with functions ---------------------------------------
addpath include             % The software receiver functions
addpath geoFunctions        % Position calculation related functions

%% Print startup ==========================================================
fprintf(['\n',...
    'Welcome to:  softGNSS\n\n', ...
    'An open source GNSS SDR software project initiated by:\n\n', ...
    '              Danish GPS Center/Aalborg University\n\n', ...
    'The code was improved by GNSS Laboratory/University of Colorado.\n\n',...
    'The software receiver softGNSS comes with ABSOLUTELY NO WARRANTY;\n',...
    'for details please read license details in the file license.txt. This\n',...
    'is free software, and  you  are  welcome  to  redistribute  it under\n',...
    'the terms described in the license.\n\n']);
fprintf('                   -------------------------------\n\n');

%% Initialize constants, settings =========================================
%///@ Jr9910: 2013/04/15，从initSetting中获取设置参数，并存储在settings结构体中
settings = initSettings();

%% Generate plot of raw data and ask if ready to start processing =========
try
    fprintf('Probing data (%s)...\n', settings.fileName)
    probeData(settings);
catch
    % There was an error, print it and exit
    errStruct = lasterror;
    disp(errStruct.message);
    disp('  (run setSettings or change settings in "initSettings.m" to reconfigure)')    
    return;
end
    
disp('  Raw IF data plotted ')
disp('  (run setSettings or change settings in "initSettings.m" to reconfigure)')
disp(' ');
gnssStart = input('Enter "1" to initiate GNSS processing or "0" to exit : ');

if (gnssStart == 1)
    disp(' ');
    %start things rolling...
    postProcessing
end