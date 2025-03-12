AAE6102-Assignment1
================
Task 1 – Acquisition 
Process the IF data using a GNSS SDR and generate initial acquisition outputs. 
---------------
This algorithm in SoftGNSS is a cold-start acquisition method for GPS signals based on frequency-domain correlation, utilizing a three-step approach consisting of initialization, coarse and fine acquisition stages.

### Step 1: Initialization
Calculate the number of samples per C/A code (samplesPerCode) based on sampling frequency, code frequency, and code length. Extract two 1-ms signal segments (signal1, signal2) for coarse acquisition, and generate a DC-removed signal (signal0DC) for fine acquisition. Compute sampling period (ts) and phase points (phasePoints). Define frequency search range (500 Hz steps, settings.acqSearchBand) and initialize frequency bins (frqBins). Generate local C/A code table (caCodesTable) and initialize result arrays (results, acqResults). 

### Step 2: Coarse Acquisition
For each PRN, compute the frequency-domain C/A code conjugate (caCodeFreqDom) via FFT. For each frequency bin, generate local carrier (sigCarr), remove carrier from signal1 and signal2 to get I/Q components, and perform FFT to get IQfreqDom1 and IQfreqDom2. Build the correlation function by multiplying with caCodeFreqDom, then IFFT to obtain correlation power (acqRes1, acqRes2). Select the higher power segment for results. Identify the peak, frequency, and code phase, compute the peak ratio, and proceed to fine acquisition if above threshold.

### Step 3: Fine Acquisition
For each detected PRN, generate a 10-ms C/A code sequence (longCaCode), remove code modulation from signal0DC using the coarse code phase to get the carrier component (xCarrier). Perform high-resolution FFT (8x nearest power of 2) on xCarrier, compute frequency bins (fftFreqBins), and estimate precise carrier frequency (acqResults.carrFreq) based on the maximum magnitude. Retain the coarse code phase (acqResults.codePhase) as the final estimate. 

The core of the algorithm is to efficiently search all possible code phases and frequency offsets using frequency-domain correlation, enhancing reliability with peak ratio analysis, and outputting the carrier frequency and code phase for each PRN.

### Result

![image](https://github.com/superrichme/yiweixu.github.io/blob/main/task1.png)

Fig. 1 displays the acquisition results for both the Open-sky dataset and the Urban dataset. For the Open-sky dataset, satellites 16, 22, 26, 27, and 31 were successfully acquired. Meanwhile, the Urban dataset yielded successful acquisitions for satellites 1, 3, 11, and 18.


Task 2 – Tracking
Adapt the tracking loop (DLL) to produce correlation plots and analyze the tracking performance. Discuss the impact of urban interference on the correlation peaks. (Multiple correlators must be implemented for plotting the correlation function)
---------------
### Step 1: Tracking Process and Components
Downconvert the signal using initial frequency and phase from acquisition, compute multi-branch correlations (Early, Prompt, Late) to get I/Q components: $`I_E,Q_E,I_P,Q_P,I_L,Q_L`$. Use FFT for efficiency $`I_P+jQ_P=\mathrm{FFT}^{-1}\left( \mathrm{FFT(signal)}\cdot \mathrm{conj(PRN}_{\mathrm{Prompt}}) \right) `$. Calculate DLL discriminator for code phase error: $`\mathrm{DLL} \mathrm{Discriminator}=({\sqrt{I_{E}^{2}+Q_{E}^{2}}-\sqrt{I_{L}^{2}+Q_{L}^{2}}})/({\sqrt{I_{E}^{2}+Q_{E}^{2}}+\sqrt{I_{L}^{2}+Q_{L}^{2}}})`$.

### Step 2: Feedback Control and Real-Time Adjustment
Smooth DLL/PLL discriminator outputs with a low-pass filter (e.g., IIR: $`y\left[ n \right] =\alpha x\left[ n \right] +\left( 1-\alpha \right) y\left[ n-1 \right] `$), adjust local carrier frequency and PRN code phase via NCO. Update at 1ms intervals to match GPS C/A code cycles. Store results in trackResults for navigation, ensuring real-time synchronization with the received signal.

### Step 3: Design Considerations and Applications
Optimize loop bandwidth and damping ratio to balance noise suppression and dynamic response. Output carrier frequency, code phase, and navigation bits from track Results. Use multi-correlator plots to analyze tracking performance, especially urban interference effects on correlation peaks.

### Result
* Open-sky
![image](https://github.com/superrichme/yiweixu.github.io/blob/main/task1.png)
