AAE6102-Assignment1
================
Task 1 – Acquisition 
Process the IF data using a GNSS SDR and generate initial acquisition outputs. 
---------------
This algorithm in SoftGNSS is a cold-start acquisition method for GPS signals based on frequency-domain correlation, utilizing a three-step approach consisting of initialization, coarse and fine acquisition stages.

### Step 1: Initialization
Calculate the number of samples per C/A code (`samplesPerCode`) based on sampling frequency, code frequency, and code length. Extract two 1-ms signal segments (`signal1, signal2`) for coarse acquisition, and generate a DC-removed signal (`signal0DC`) for fine acquisition. Compute sampling period (`ts`) and phase points (`phasePoints`). Define frequency search range (`500 Hz steps, settings.acqSearchBand`) and initialize frequency bins (`frqBins`). Generate local C/A code table (`caCodesTable`) and initialize result arrays (`results, acqResults`). 

### Step 2: Coarse Acquisition
For each PRN, compute the frequency-domain C/A code conjugate (`caCodeFreqDom`) via FFT. For each frequency bin, generate local carrier (`sigCarr`), remove carrier from signal1 and signal2 to get I/Q components, and perform FFT to get `IQfreqDom1` and `IQfreqDom2`. Build the correlation function by multiplying with `caCodeFreqDom`, then IFFT to obtain correlation power (`acqRes1, acqRes2`). Select the higher power segment for results. Identify the peak, frequency, and code phase, compute the peak ratio, and proceed to fine acquisition if above threshold.

### Step 3: Fine Acquisition
For each detected PRN, generate a 10-ms C/A code sequence (`longCaCode`), remove code modulation from signal0DC using the coarse code phase to get the carrier component (`xCarrier`). Perform high-resolution FFT (`8x nearest power of 2`) on xCarrier, compute frequency bins (fftFreqBins), and estimate precise carrier frequency (`acqResults.carrFreq`) based on the maximum magnitude. Retain the coarse code phase (`acqResults.codePhase`) as the final estimate. 

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
Smooth DLL/PLL discriminator outputs with a low-pass filter (e.g., IIR: $`y\left[ n \right] =\alpha x\left[ n \right] +\left( 1-\alpha \right) y\left[ n-1 \right] `$), adjust local carrier frequency and PRN code phase via NCO. Update at 1ms intervals to match GPS C/A code cycles. Store results in `trackResults` for navigation, ensuring real-time synchronization with the received signal.

### Step 3: Design Considerations and Applications
Optimize loop bandwidth and damping ratio to balance htmlnoise suppression and dynamic response. Output carrier frequency, code phase, and navigation bits from track Results. Use multi-correlator plots to analyze tracking performance, especially urban interference effects on correlation peaks.

### Result
* Open-sky
![image](https://github.com/superrichme/yiweixu.github.io/blob/main/task2_1.png)

### Results analysis
#### (a) PRN 16
Correlation output shows stable filtered DLL at 6000-8000, with raw discriminator and filtered PLL fluctuating. Raw DLL amplitude has noise, but filtered DLL stabilizes at 0.2-0.4, indicating effective noise reduction. PRN 16 tracking remains stable, with filtered discriminators maintaining lock well.

#### (b) PRN 26
Filtered DLL correlation stays stable at 6000-8000, raw discriminator varies more. Filtered PLL tracks closely with slight deviations. Raw DLL amplitude fluctuates (±0.5), filtered DLL stabilizes at 0.2-0.3, showing good noise suppression. PRN 26 tracking matches PRN 16, with effective signal stabilization.

#### (c) PRN 31
Filtered DLL correlation baseline is lower at 4000-6000 compared to PRN 16 and 26. Raw discriminator and filtered PLL oscillate more. Raw DLL amplitude varies (±0.6), filtered DLL stabilizes at 0.1-0.3 with drops. PRN 31 tracking appears less stable, but filtered DLL performs reasonably.

#### (d) PRN 22
Filtered DLL correlation stays at 5000-7000, raw discriminator shows spikes. Filtered PLL tracks smoothly with some divergence. Raw DLL amplitude fluctuates (±0.5), filtered DLL stabilizes at 0.2-0.4 with dips. PRN 22 tracking remains moderately stable, with filtering mitigating noise effectively.

#### (e) PRN 27
Filtered DLL correlation holds at 4000-6000, similar to PRN 31. Raw discriminator and filtered PLL vary notably. Raw DLL amplitude fluctuates (±0.6), filtered DLL stabilizes at 0.1-0.3 with end instability. PRN 27 tracking ranks least stable, but filtering offers some improvement.

SoftGNSS performs well in Open-sky scenario, with PRN 16 and 26 showing top stability and signal strength. Filtering (PLL and DLL) reduces noise and stabilizes outputs. PRN 31, 22, and 27 show weaker performance due to signal or environmental issues, but the system maintains lock.

### Result
* Urban
![image](https://github.com/superrichme/yiweixu.github.io/blob/main/task2_2.png)

### Results analysis
The tracking results for PRN 1, 3, 11, and 18 in the urban SoftGNSS dataset illustrate a stable filtered DLL baseline, though correlation strength varies across PRNs. The raw PLL and DLL discriminators exhibit significant noise, while filtering stabilizes outputs to some extent. Inconsistent correlation and occasional signal instability occur, particularly toward the end.

These phenomena result from urban multipath effects and signal obstructions, which weaken satellite signals and introduce noise. Lower correlation baselines (e.g., PRN 1 and 11) stem from signal attenuation by buildings, while higher values (e.g., PRN 3 and 18) reflect better visibility. Intermittent signal loss arises from blockages and reflections.

### Comparison with Open-Sky Results
Urban tracking underperforms compared to open-sky results, where PRN 16 and 26 maintained stable 6000-8000 correlations with minimal noise due to unobstructed signals. Urban multipath and obstructions cause wider amplitude fluctuations and inconsistent correlations (5000-15000), reducing filtering effectiveness. Open-sky’s clear line-of-sight contrasts with urban challenges, explaining the reliability gap. The fundamental reason for the difference lies in the unobstructed line-of-sight signal propagation in open-sky environments, while urban multipath and blockages disrupt signal integrity.

Task 3 – Navigation data decoding
Decode the navigation message and extract key parameters, such as ephemeris data, for at least one satellite.
-------------------
### Step 1: Extract Navigation Bit Samples
Principle: Extracts 1500 bit samples (covering 5 subframes, each 300 bits, 1 bit = 20 samples) from trackResults.I_P. subFrameStart defines the subframe start, with an extra 20 samples for alignment.
Purpose: Obtains a sufficient bit sequence (30 seconds, 5 subframes) to ensure complete navigation data.

### Step 2: Reshape and Sum Samples for Bit Estimation
Principle: Reshapes samples into a 20-row matrix (each row of 20 samples = 1 bit), sums columns to estimate bits. Summing enhances signal robustness by reducing noise.
Purpose: Integrates multiple samples (20) into a single bit value, improving decoding reliability.

![image](https://github.com/superrichme/yiweixu.github.io/blob/main/task3.png)
A comparison of navigation message decoding results between open sky and urban datasets reveals that the open sky data maintains a stable bit amplitude of ±5000, with continuous signals and minimal fluctuations, attributed to a high signal-to-noise ratio (SNR), absence of multipath interference, and stable tracking (Filtered PLL/DLL ±50/±5), ensuring reliable decoding. In contrast, the urban data exhibits significant amplitude variations (±5×10⁴) with frequent drops (e.g., at 30s, 50s, 70s), due to multipath effects, signal blockages reducing SNR, and unstable tracking (PRN 18 Filtered PLL/DLL ±150/±6), leading to decoding interruptions and increased noise. These differences highlight the urban environment's challenges to signal integrity, impacting tracking and decoding stability. 

### Step 3: Thresholding to Binary Bits
Principle: Applies a threshold (greater than $`0 = 1, ≤ 0 = 0`$) to convert summed values to binary bits (0 or 1).
Purpose: Transforms continuous values into discrete binary format, matching navigation message structure.

### Step 4: Convert to Binary String Format
Principle: Uses `dec2bin` to transform binary array into a string array (containing only "0" and "1" characters), matching ephemeris function input.
Purpose: Adapts input format for the decoding function, ensuring accurate processing.

### Step 5: Decode Ephemeris and TOW
Principle: `ephemeris` parses 1500 bits (5 subframes) to extract ephemeris parameters (e.g., 
,e) and TOW from Subframe 1. `navBitsBin(2:1501)` provides data, `navBitsBin(1)` aids validation.
Purpose: Generates ephemeris structure (`eph`) and TOW for subsequent positioning.

In summary, the decoding process in `postNavigation.m` starts by extracting navigation bit samples, reshapes and sums them, thresholds to binary, converts to string format, decodes ephemeris and TOW using `ephemeris`, and validates data integrity, providing reliable input for positioning.
