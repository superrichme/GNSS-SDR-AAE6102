function [firstSubFrame, activeChnList] = findPreambles(trackResults, ...
                                                        settings)
% findPreambles finds the first preamble occurrence in the bit stream of
% each channel. The preamble is verified by check of the spacing between
% preambles (6sec) and parity checking of the first two words in a
% subframe. At the same time function returns list of channels, that are in
% tracking state and with valid preambles in the nav data stream.
%
%[firstSubFrame, activeChnList] = findPreambles(trackResults, settings)
%
%   Inputs:
%       trackResults    - output from the tracking function
%       settings        - Receiver settings.
%
%   Outputs:
%       firstSubframe   - the array contains positions of the first
%                       preamble in each channel. The position is ms count 
%                       since start of tracking. Corresponding value will
%                       be set to 0 if no valid preambles were detected in
%                       the channel.
%       activeChnList   - list of channels containing valid preambles

%--------------------------------------------------------------------------
%                           SoftGNSS v3.0
% 
% Copyright (C) Darius Plausinaitis, Peter Rinder and Nicolaj Bertelsen
% Written by Darius Plausinaitis, Peter Rinder and Nicolaj Bertelsen
%--------------------------------------------------------------------------
%
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
% $Id: findPreambles.m,v 1.1.2.10 2006/08/14 11:38:22 dpl Exp $

% Preamble search can be delayed to a later point in the tracking results
% to avoid noise due to tracking loop transients 
searchStartOffset = 0;

%--- Initialize the firstSubFrame array -----------------------------------
firstSubFrame = zeros(1, settings.numberOfChannels);

%--- Generate the preamble pattern ----------------------------------------
preamble_bits = [1 -1 -1 -1 1 -1 1 1];

% "Upsample" the preamble - make 20 vales per one bit. The preamble must be
% found with precision of a sample.
%///@ Jr9910: 2013/04/24，Kronecker
% Product：就是将preamble_bits每一个bit采样成20bit，此时每个采样点变成1ms
preamble_ms = kron(preamble_bits, ones(1, 20));

%--- Make a list of channels excluding not tracking channels --------------
activeChnList = find([trackResults.status] ~= '-');

%=== For all tracking channels ...
for channelNr = activeChnList

%% Correlate tracking output with preamble ================================
    % Read output from tracking. It contains the navigation bits. The start
    % of record is skiped here to avoid tracking loop transients.
    %/// Jr9910: 2013/04/24，因为并不知道帧头在何处，所以要处理所有值
    bits = trackResults(channelNr).I_P(1 + searchStartOffset : end);

    % Now threshold the output and convert it to -1 and +1 
    bits(bits > 0)  =  1;
    bits(bits <= 0) = -1;

    % Correlate tracking output with the preamble
    %///@ Jr9910: 2013/04/24，计算互相关值
    tlmXcorrResult = xcorr(bits, preamble_ms);

%% Find all starting points off all preamble like patterns ================
    clear index
    clear index2

    %///@ Jr9910: 2013/04/24，这个长度和原始的bits数据长度一致
    xcorrLength = (length(tlmXcorrResult) +  1) /2;

    %--- Find at what index/ms the preambles start ------------------------
    %///@ Jr9910:
    %2013/04/24，没有具体研究xcorr算法，但是tlmXcorrResult只有后一半有效，且与原始序列索引一致
    index = find(...
        abs(tlmXcorrResult(xcorrLength : xcorrLength * 2 - 1)) > 153)' + ...
        searchStartOffset;

%% Analyze detected preamble like patterns ================================
    %/// Jr9910: 2013/04/24，这段代码实施还是挺巧妙的，呵！呵！
    for i = 1:size(index) % For each occurrence

        %--- Find distances in time between this occurrence and the rest of
        %preambles like patterns. If the distance is 6000 milliseconds (one
        %subframe), the do further verifications by validating the parities
        %of two GPS words
        
        index2 = index - index(i);

        if (~isempty(find(index2 == 6000)))

            %=== Re-read bit vales for preamble verification ==============
            % Preamble occurrence is verified by checking the parity of
            % the first two words in the subframe. Now it is assumed that
            % bit boundaries a known. Therefore the bit values over 20ms are
            % combined to increase receiver performance for noisy signals.
            % in Total 62 bits mast be read :
            % 2 bits from previous subframe are needed for parity checking;
            % 60 bits for the first two 30bit words (TLM and HOW words).
            % The index is pointing at the start of TLM word.
            %///@ Jr9910:
            %2013/04/24，提取了62bits，前一个子帧最后两个bit和当前子帧前60bit构成奇偶校验数据，因为已经大体确定了帧头，因此可以对20ms数据进行求和，以提高抗噪能力
            bits = trackResults(channelNr).I_P(index(i)-40 : ...
                                               index(i) + 20 * 60 -1)';

            %--- Combine the 20 values of each bit ------------------------
            bits = reshape(bits, 20, (size(bits, 1) / 20));
            bits = sum(bits);

            % Now threshold and make it -1 and +1 
            bits(bits > 0)  = 1;
            bits(bits <= 0) = -1;

            %--- Check the parity of the TLM and HOW words ----------------
            if (navPartyChk(bits(1:32)) ~= 0) && ...
               (navPartyChk(bits(31:62)) ~= 0)
                % Parity was OK. Record the preamble start position. Skip
                % the rest of preamble pattern checking for this channel
                % and process next channel. 
                %///@ Jr9910:
                %2013/04/24，记录C/A码起点，起点位置是相对于跟踪起点来说的，单位为ms，因为捕获程序所获得的C/A码的初始相位都不足1ms，
                % 因此跟踪程序的起点也就是C/A码起点，就是整数ms位置的起点
                firstSubFrame(channelNr) = index(i);
                break;    
            end % if parity is OK ...
            
        end % if (~isempty(find(index2 == 6000)))
    end % for i = 1:size(index)

    % Exclude channel from the active channel list if no valid preamble was
    % detected
    if firstSubFrame(channelNr) == 0
        
        % Exclude channel from further processing. It does not contain any
        % valid preamble and therefore nothing more can be done for it.
        activeChnList = setdiff(activeChnList, channelNr);
        
        disp(['Could not find valid preambles in channel ', ...
                                                  num2str(channelNr),'!']);
    end
    
end % for channelNr = activeChnList
