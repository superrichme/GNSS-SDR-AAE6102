% function plotAcquisition(acqResults)
% %Functions plots bar plot of acquisition results (acquisition metrics). No
% %bars are shown for the satellites not included in the acquisition list (in
% %structure SETTINGS). 
% %
% %plotAcquisition(acqResults)
% %
% %   Inputs:
% %       acqResults    - Acquisition results from function acquisition.
% 
% %--------------------------------------------------------------------------
% %                           SoftGNSS v3.0
% % 
% % Copyright (C) Darius Plausinaitis
% % Written by Darius Plausinaitis
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
% 
% % CVS record:
% % $Id: plotAcquisition.m,v 1.1.2.4 2006/08/09 17:20:11 dpl Exp $
% %{
% ///@ Jr9910：2013/04/15，以直方图的形式绘制捕获结果
% %}
% %% Plot all results =======================================================
% figure(101);
% 
% hAxes = newplot();
% 
% bar(hAxes, acqResults.peakMetric);
% 
% title (hAxes, 'Acquisition results');
% xlabel(hAxes, 'PRN number (no bar - SV is not in the acquisition list)');
% ylabel(hAxes, 'Acquisition Metric');
% 
% oldAxis = axis(hAxes);
% axis  (hAxes, [0, 33, 0, oldAxis(4)]);
% set   (hAxes, 'XMinorTick', 'on');
% set   (hAxes, 'YGrid', 'on');
% 
% %% Mark acquired signals ==================================================
% 
% acquiredSignals = acqResults.peakMetric .* (acqResults.carrFreq ~= 0);
% 
% hold(hAxes, 'on');
% bar (hAxes, acquiredSignals, 'FaceColor', [0 0.8 0]);
% hold(hAxes, 'off');
% 
% legend(hAxes, 'Not acquired signals', 'Acquired signals');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function plotAcquisition(acqResults)
%Functions plots bar plot of acquisition results (acquisition metrics). No
%bars are shown for the satellites not included in the acquisition list (in
%structure SETTINGS). 
%
%plotAcquisition(acqResults)
%
%   Inputs:
%       acqResults    - Acquisition results from function acquisition.
%                      Contains peakMetric and carrFreq fields for PRN 1-32.

%--------------------------------------------------------------------------
%                           SoftGNSS v3.0
% 
% Copyright (C) Darius Plausinaitis
% Written by Darius Plausinaitis
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

% CVS record:
% $Id: plotAcquisition.m,v 1.1.2.4 2006/08/09 17:20:11 dpl Exp $
%{
///@ Jr9910：2013/04/15，以直方图的形式绘制捕获结果
% Modified for Opensky.bin data (visualization of acquisition results)
%}

%% Input validation =======================================================
% Check if acqResults contains required fields
if ~isstruct(acqResults) || ~isfield(acqResults, 'peakMetric') || ~isfield(acqResults, 'carrFreq')
    error('acqResults must be a structure with peakMetric and carrFreq fields.');
end
if length(acqResults.peakMetric) ~= 32 || length(acqResults.carrFreq) ~= 32
    error('acqResults.peakMetric and carrFreq must have length 32 (PRN 1-32).');
end

%% Plot all results =======================================================
figure(101); % Create or reuse figure 101
clf; % Clear the figure

hAxes = newplot();

% Plot bar for all acquisition metrics
bar(hAxes, 1:32, acqResults.peakMetric, 'FaceColor', [0.5 0.5 0.5]);

%title(hAxes, 'Acquisition Results for Urban.dat Data');
title(hAxes, 'Acquisition Results for Opensky.bin Data');
xlabel(hAxes, 'PRN Number (no bar - SV not in acquisition list)');
ylabel(hAxes, 'Acquisition Metric');

oldAxis = axis(hAxes);
axis(hAxes, [0, 33, 0, oldAxis(4)]); % Set x-axis to 1-32, y-axis auto
set(hAxes, 'XMinorTick', 'on');
set(hAxes, 'YGrid', 'on');

%% Mark acquired signals ==================================================
% Identify acquired signals (where carrFreq is non-zero)
acquiredSignals = acqResults.peakMetric .* (acqResults.carrFreq ~= 0);

hold(hAxes, 'on');
bar(hAxes, 1:32, acquiredSignals, 'FaceColor', [0 0.8 0]); % Green for acquired
hold(hAxes, 'off');

legend(hAxes, 'Not Acquired Signals', 'Acquired Signals', 'Location', 'best');

% Add grid for better readability
grid(hAxes, 'on');