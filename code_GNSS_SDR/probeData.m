function probeData(varargin)
%{
///@ Jr9910:
2013/04/15，这个函数完成了采样数据的时域和频域分析，时域分析主要是时域采样波形以及直方图统计，频域分析主要是功率谱密度分析
%}
%% Check the number of arguments ==========================================
%///@ Jr9910: 2013/04/15，该段代码检验函数输入参数，并将输入存储在fileNameStr和settings中
%///@ Jr9910: 2013/04/15，nargin返回函数输入参数的个数
if (nargin == 1)
    %///@ Jr9910:
    %2013/04/15，deal将右边参数按照结构或者cell复制到左边参数，结合probeData函数说明理解，
    % 函数输入只有两种settings和fileName，settings
    settings = deal(varargin{1});
    fileNameStr = settings.fileName;
elseif (nargin == 2)
    [fileNameStr, settings] = deal(varargin{1:2});
    if ~ischar(fileNameStr)
        error('File name must be a string');
    end
else
    error('Incorect number of arguments');
end

%% Generate plot of raw data ==============================================
[fid, message] = fopen(fileNameStr, 'rb');

if (fid > 0)
    % Move the starting point of processing. Can be used to start the
    % signal processing at any point in the data record (e.g. for long
    % records).
    fseek(fid, settings.skipNumberOfBytes, 'bof');

    % Find number of samples per spreading code
    %///@ Jr9910: 2013/04/15，也就是得到1ms有多少个采样点，一个码周期
    samplesPerCode = round(settings.samplingFreq / ...
        (settings.codeFreqBasis / settings.codeLength));

    %///@ Jr9910: 2013/04/15，fileType指定了使用实数采样数据还是复数采样数据
    if (settings.fileType==1)  % real mode: S0, S1, S2, S3, ...
        dataAdaptCoeff=1;
    else
        dataAdaptCoeff=2;      % complex mode I0, Q0, I1, Q1, ...
    end

    % Read 100ms of signal
   [data, count] = fread(fid, [1, dataAdaptCoeff*100*samplesPerCode], settings.dataType);


    fclose(fid);

    if (count < dataAdaptCoeff*100*samplesPerCode)
        % The file is to short
        error('Could not read enough data from the data file.');
    end

    %--- Initialization ---------------------------------------------------
    figure(100);
    clf(100);

    timeScale = 0 : 1/settings.samplingFreq : 5e-3;

    %--- Time domain plot -------------------------------------------------
    if (settings.fileType==1)

        %///@ Jr9910: 2013/04/15，乘以1000，时间轴单位变成ms，除50是否有什么特殊的含义？
        % 只是绘前1/50ms的采样点
        subplot(2, 2, 3);
        plot(1000 * timeScale(1:round(samplesPerCode/50)), ...
            data(1:round(samplesPerCode/50)));

        axis tight;    grid on;
        title ('Time domain plot');
        xlabel('Time (ms)'); ylabel('Amplitude');
    else

        data=data(1:2:end) + i .* data(2:2:end);
        subplot(3, 2, 4);
        plot(1000 * timeScale(1:round(samplesPerCode/50)), ...
            real(data(1:round(samplesPerCode/50))));

        axis tight;    grid on;
        title ('Time domain plot (I)');
        xlabel('Time (ms)'); ylabel('Amplitude');

        subplot(3, 2, 3);
        plot(1000 * timeScale(1:round(samplesPerCode/50)), ...
            imag(data(1:round(samplesPerCode/50))));

        axis tight;    grid on;
        title ('Time domain plot (Q)');
        xlabel('Time (ms)'); ylabel('Amplitude');

    end


    %--- Frequency domain plot --------------------------------------------

    if (settings.fileType==1) %Real Data
        subplot(2,2,1:2);
        pwelch(data, 32758, 2048, 16368, settings.samplingFreq/1e6)
    else % I/Q Data
        subplot(3,2,1:2);
        [sigspec,freqv]=pwelch(data, 32758, 2048, 16368, settings.samplingFreq,'twosided');
        plot(([-(freqv(length(freqv)/2:-1:1));freqv(1:length(freqv)/2)])/1e6, ...
            10*log10([sigspec(length(freqv)/2+1:end);
            sigspec(1:length(freqv)/2)]));
    end

    axis tight;
    grid on;
    title ('Frequency domain plot');
    xlabel('Frequency (MHz)'); ylabel('Magnitude');

    %--- Histogram --------------------------------------------------------

    if (settings.fileType == 1)
        subplot(2, 2, 4);
        hist(data, -128:128)

        dmax = max(abs(data)) + 1;
        axis tight;     adata = axis;
        axis([-dmax dmax adata(3) adata(4)]);
        grid on;        title ('Histogram');
        xlabel('Bin');  ylabel('Number in bin');
    else
        subplot(3, 2, 6);
        hist(real(data), -128:128)
        dmax = max(abs(data)) + 1;
        axis tight;     adata = axis;
        axis([-dmax dmax adata(3) adata(4)]);
        grid on;        title ('Histogram (I)');
        xlabel('Bin');  ylabel('Number in bin');

        subplot(3, 2, 5);
        hist(imag(data), -128:128)
        dmax = max(abs(data)) + 1;
        axis tight;     adata = axis;
        axis([-dmax dmax adata(3) adata(4)]);
        grid on;        title ('Histogram (Q)');
        xlabel('Bin');  ylabel('Number in bin');

    end
else
    %=== Error while opening the data file ================================
    error('Unable to read file %s: %s.', fileNameStr, message);
end % if (fid > 0)

