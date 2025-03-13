t = 0:0.01:0.5; % 时间向量
for prn = [1, 3, 11, 18]
    % 假设 tracking 函数返回 I/Q 分量
    [I_P, Q_P] = tracking(longSignal, prn, acqResults.carrFreq(prn), acqResults.codePhase(prn));
    correlation(prn, :) = sqrt(I_P.^2 + Q_P.^2); % 相关幅度
end
plot(t, correlation(1, :), 'b', t, correlation(3, :), 'r', ...
     t, correlation(11, :), 'y', t, correlation(18, :), 'm');
legend('Channel 1 (PRN 1)', 'Channel 1 (PRN 3)', 'Channel 1 (PRN 11)', 'Channel 1 (PRN 18)');