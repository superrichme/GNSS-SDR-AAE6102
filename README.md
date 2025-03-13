AAE6102-Assignment1
================
Task 1 – Acquisition.  
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

![image](https://github.com/superrichme/GNSS-SDR-AAE6102/blob/main/task1.png)

This figure displays the acquisition results for both the Open-sky dataset and the Urban dataset. For the Open-sky dataset, satellites 16, 22, 26, 27, and 31 were successfully acquired. Meanwhile, the Urban dataset yielded successful acquisitions for satellites 1, 3, 11, and 18.

![image](https://github.com/superrichme/GNSS-SDR-AAE6102/blob/main/task1_1.png)

The GNSS-SDR satellite acquisition process is illustrated through frequency and time domain plots for Open-sky and Urban scenarios. In the Open-sky case, the frequency domain plot shows a distinct peak around 0 MHz with a magnitude of approximately 80, indicating a strong Doppler shift detection. The time domain plots for Q and I channels exhibit clear periodic signals with amplitudes around ±2, and the histograms reveal concentrated bins near 0, suggesting low noise and successful acquisition. Conversely, the Urban scenario displays a noisier frequency domain with a weaker peak magnitude of around 60, reflecting signal degradation due to obstructions. The time domain plots for Q and I channels show increased noise with amplitudes fluctuating between ±5, and the histograms exhibit wider distributions, indicating higher variability and multipath effects. These differences highlight the challenges of satellite acquisition in urban environments, where signal interference and blockages degrade GNSS-SDR performance compared to the clearer Open-sky conditions.

Task 2 – Tracking.    
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

### Results
* Urban
![image](https://github.com/superrichme/yiweixu.github.io/blob/main/task2_2.png?raw=true)

### Result analysis
The tracking results for PRN 1, 3, 11, and 18 in the urban SoftGNSS dataset illustrate a stable filtered DLL baseline, though correlation strength varies across PRNs. The raw PLL and DLL discriminators exhibit significant noise, while filtering stabilizes outputs to some extent. Inconsistent correlation and occasional signal instability occur, particularly toward the end.

These phenomena result from urban multipath effects and signal obstructions, which weaken satellite signals and introduce noise. Lower correlation baselines (e.g., PRN 1 and 11) stem from signal attenuation by buildings, while higher values (e.g., PRN 3 and 18) reflect better visibility. Intermittent signal loss arises from blockages and reflections.

### Comparison with Open-Sky Results
Urban tracking underperforms compared to open-sky results, where PRN 16 and 26 maintained stable 6000-8000 correlations with minimal noise due to unobstructed signals. Urban multipath and obstructions cause wider amplitude fluctuations and inconsistent correlations (5000-15000), reducing filtering effectiveness. Open-sky’s clear line-of-sight contrasts with urban challenges, explaining the reliability gap. The fundamental reason for the difference lies in the unobstructed line-of-sight signal propagation in open-sky environments, while urban multipath and blockages disrupt signal integrity.

Task 3 – Navigation data decoding.    
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
Principle: `ephemeris` parses 1500 bits (5 subframes) to extract ephemeris parameters (`e.g., e`) and TOW from Subframe 1. `navBitsBin(2:1501)` provides data, `navBitsBin(1)` aids validation.
Purpose: Generates ephemeris structure (`eph`) and TOW for subsequent positioning.

In summary, the decoding process in `postNavigation.m` starts by extracting navigation bit samples, reshapes and sums them, thresholds to binary, converts to string format, decodes ephemeris and TOW using `ephemeris`, and validates data integrity, providing reliable input for positioning.

### Results
* Open-sky
  
| Ephemeris data | PRN-16       | PRN-22       | PRN-26       | PRN-31       | Meaning                            |
|----------------|--------------|--------------|--------------|--------------|------------------------------------|
| C_ic           | -1.0058e-07  | -1.0058e-07  | -2.0489e-08  | -1.1362e-07  | Cosine-harmonic-correction-to-inclination (rad) |
| omega_0        | -1.6743      | 1.2727       | -1.8129      | -2.7873      | Right ascension at reference time (rad) |
| C_is           | 1.3597e-07   | -9.3132e-08  | 8.9407e-08   | -5.0291e-08  | Sine-harmonic-correction-to-inclination (rad) |
| i_0            | 0.9716       | 0.9365       | 0.9399       | 0.9559       | Inclination at reference time (rad) |
| C_rc           | 237.6875     | 266.3438     | 234.1875     | 240.1563     | Cosine-harmonic-correction-to-orbit-radius (m) |
| omega          | 0.6796       | -0.8879      | 0.2957       | 0.3116       | Argument of perigee (rad)         |
| omegaDot       | -8.0128e-09  | -8.6686e-09  | -8.3114e-09  | -7.9950e-09  | Rate of right ascension (rad/s)   |
| IODE_sf3       | 9            | 22           | 113          | 83           | Issue of Data, Ephemeris (Subframe 3) |
| IDot           | -4.8931e-10  | -3.0358e-11  | -4.1752e-10  | -3.2144e-11  | Rate of inclination (rad/s)       |
| WeekNumber     | 1155         | 1155         | 1155         | 1155         | GPS-week number                   |
| T_GD           | -1.0245e-08 | -1.7695e-08  |6.9849e-09  | -1.3039e-08 | Satellite clock correction (s)    |
| IODC           | 234          | 218          | 15           | 228          | Issue of Data, Clock              |
| t_oc           | 396000       | 396000       | 396000       | 396000       | Clock reference time (s)          |
| a_f1           | -6.3665e-12  | 9.2086e-12  | 3.9790e-12  | -1.9327e-12  | Clock drift linear term (s/s)     |
| a_f0           | -4.0693e-04  | -4.8947e-04  | 1.4479e-04  | -1.4490e-04  | Clock bias (s)                    |
| IODE_sf2       | 9            | 22           | 113          | 83           | Issue of Data, Ephemeris (Subframe 2) |
| C_rs           | 23.3438      | -99.8125     | 21.2500      | 30.7188      | Sine harmonic correction to orbit radius (m) |
| delta_n        | 4.2466e-09   | 5.2831e-09   | 5.0513e-09   | 4.8073e-09   | Mean motion correction (rad/s)    |
| M_0            | 0.7181       | -1.2610      | 1.7356       | 2.8245       | Mean anomaly at reference time (rad) |
| C_uc           | 1.3895e-06   | -5.1558e-06  | 1.1530e-06   | 1.4603e-06   | Cosine-harmonic-correction-to-latitude (rad) |
| e              | 0.0123       | 0.0067       | 0.0063       | 0.0103       | Eccentricity                      |
| C_us           | 7.6871e-06   | 5.1651e-06   | 7.0408e-06   | 7.2289e-06   | Sine-harmonic-correction-to-latitude (rad) |
| sqrt_A          | 5.1538e+03   | 5.1537e+03   | 5.1536e+03   | 5.1536e+03   | Square-root-of-semi-major-axis (m^(1/2)) |
| t_oe           | 396000       | 396000       | 396000       | 396000       | Ephemeris reference time (s)       |

* Urban
  
| Ephemeris data | PRN-1       | PRN-3       | PRN-11      | PRN-18      | Meaning                            |
|----------------|-------------|-------------|-------------|-------------|------------------------------------|
| C_ic           | -7.4506e-08 | 1.1176e-08  | -3.1665e-07 | 2.5332e-07  | Cosine-harmonic-correction-to-inclination (rad) |
| omega_0        | -3.1060     | -2.0642     | 2.7258      | 3.1218      | Right ascension at reference time (rad) |
| C_is           | 1.6019e-07  | 5.2154e-08  | -1.3225e-07 | 3.5390e-08  | Sine-harmonic-correction-to-inclination (rad) |
| i_0            | 0.9761      | 0.9629      | 0.9008      | 0.9546      | Inclination at reference time (rad) |
| C_rc           | 287.4688    | 160.3125    | 324.4063    | 280.1563    | Cosine-harmonic-correction-to-orbit-radius (m) |
| omega          | 0.7115      | 0.5950      | 1.8915      | 1.3930      | Argument of perigee (rad)         |
| omegaDot       | -8.1696e-09 | -7.8325e-09 | -9.3043e-09 | -8.6107e-09 | Rate of right ascension (rad/s)   |
| IODE_sf3       | 72          | 72          | 83          | 56          | Issue of Data, Ephemeris (Subframe 3) |
| IDot           | -1.8108e-10 | 4.8109e-10  | -1.2858e-11 | -1.6179e-10 | Rate of inclination (rad/s)       |
| weekNumber     | 1032        | 1032        | 1032        | 1032        | GPS-week number                   |
| T_GD           | 5.5879e-09  | 1.8626e-09  | -1.2573e-08 | 5.5879e-09  | Satellite clock correction (s)    |
| IODC           | 12          | 4           | 229         | 244         | Issue of Data, Clock              |
| t_oc           | 453600      | 453600      | 453600      | 453600      | Clock reference time (s)          |
| a_f1           | -9.4360-12 | -1.1369e-12 | 8.5265e-12  | 3.1832e-12  | Clock drift linear term (s/s)     |
| a_f0           | -1.2087     | 1.8633e-04  | -5.9009e-04 | 9.8655e-05  | Clock bias (s)                    |
| IODE_sf2       | 72          | 72          | 83          | 56          | Issue of Data, Ephemeris (Subframe 2) |
| C_rs           | -120.7188   | -62.0938    | -67.1250    | -113.8750   | Sine-harmonic-correction-to-orbit-radius (m) | 
| M_0            | 0.5179      | -0.4304     | -0.1989     | 0.2598      | Mean anomaly at reference time (rad) |
| C_uc           | 8.6149e-06  | 3.9066e-06  | 3.6042e-06  | 6.1095e-06  | Cosine-harmonic-correction-to-latitude (rad) |
| e              | 0.0089      | 0.0022      | 0.0166      | 0.0154      | Eccentricity                      |
| C_us           | 5.3031e-06  | 3.9066e-06  | 1.5123e-06  | 5.1148e-06  | Sine-harmonic-correction-to-latitude (rad) |
| sqrtA          | 5153.7e+03  | 5153.8e+03  | 5153.7e+03  | 5153.7e+03  | Square-root-of-semi-major-axis (m^(1/2)) |
| t_oe           | 453600      | 453600      | 453600      | 453600      | Ephemeris reference time (s)       |

Task 4 – Position and velocity estimation.    
Using the pseudorange measurements obtained from tracking, implement the Weighted Least Squares (WLS) algorithm to compute user’s position and velocity. Plot the user position and velocity, compare it to the provided ground truth values, and comment on the impact of multipath effects on the WLS solution.
---------------------------------

### Position estimation
The WLS method for computing receiver position and velocity in this GNSS algorithm minimizes the weighted residuals between observed and predicted measurements, accounting for varying observation quality. For position, pseudorange observations $`\rho_{\text{obs}}`$ are modeled as $`\rho_{\text{obs}} = \|\mathbf{r}_{\text{sat}} - \mathbf{r}_{\text{rec}}\| + c \cdot dt + \epsilon`$, where $`\mathbf{r}_{\text{sat}}`$ and $`\mathbf{r}_{\text{rec}}`$ are satellite and receiver positions, $`dt`$ is clock bias, and $`\epsilon`$ is noise. The WLS solution iteratively updates the state $`\mathbf{x} = [x, y, z, dt]^T`$ by solving $`\mathbf{x} = (A^T W A)^{-1} A^T W \mathbf{b}`$, where $`A`$ is the geometry matrix, $`W`$ is a diagonal weight matrix (e.g., $`W_{ii} = \sin^2(\text{el}_i)`$), and $`\mathbf{b}`$ is the residual vector.

```matlab
......
for iter = 1:nmbOfIterations
    W = zeros(nmbOfSatellites, x`nmbOfSatellites); 
    for i = 1:nmbOfSatellites
        if iter == 1
            Rot_X = X(:, i);
            trop = 2;
            el(i) = 0;
            W(i, i) = 1; 
        else
            rho2 = (X(1, i) - pos(1))^2 + (X(2, i) - pos(2))^2 + (X(3, i) - pos(3))^2;
            traveltime = sqrt(rho2) / settings.c;
            Rot_X = e_r_corr(traveltime, X(:, i)); 
            [az(i), el(i), dist] = topocent(pos(1:3, :), Rot_X - pos(1:3, :)); 
            W(i, i) = max(0.1, sin(el(i) * dtr)^2); 
            if (settings.useTropCorr == 1)
                trop = tropo(sin(el(i) * dtr), 0.0, 1013.0, 293.0, 50.0, 0.0, 0.0, 0.0);
            else
                trop = 0;
            end
        end
        omc(i) = (obs(i) - norm(Rot_X - pos(1:3), 'fro') - pos(4) - trop);
        A(i, :) = [ (-(Rot_X(1) - pos(1))) / obs(i) ...
                    (-(Rot_X(2) - pos(2))) / obs(i) ...
                    (-(Rot_X(3) - pos(3))) / obs(i) ...
                    1 ];
    end
    x = (A' * W * A) \ (A' * W * omc);
    pos = pos + x; 
end
pos = pos';
......
```
### Results
![image](https://github.com/superrichme/yiweixu.github.io/blob/main/task4.png)

### Result analysis

The left figure depicts the positioning variations in the East, North, and Up directions over 160 seconds in Open-Sky scenario. The variations fluctuate between approximately -50 m and +40 m, with most deviations within ±30 m across all components. In an OpenSky environment, the absence of reflective surfaces minimizes multipath effects, reducing phase estimation errors in the DLL and PLL loops. The WLS method benefits from stronger satellite signals and better geometry, as obstructions are minimal. Additionally, atmospheric delays are more predictable and easier to correct in open areas. The consistent SNR across satellites ensures more balanced weights in WLS, leading to errors that are more reasonable—likely within ±30 m as shown—compared to the urban case. This aligns with expectations, as Open-Sky scenarios typically yield higher accuracy and reliability in GNSS positioning. 

This indicates a relatively stable performance with errors significantly lower than the urban scenario, where errors reached 100 m within the first 80 seconds before failing due to multipath-induced loss of lock. As shown in right figure, in urban environments, the WLS positioning achieves ENU errors within 100 meters for the first 80 seconds, but fails after ring-road loss of lock. This is primarily due to the traditional DLL and PLL lacking multipath suppression. Multipath effects, prevalent in cities with reflective surfaces, introduce phase estimation errors, destabilizing the PLL and causing loss of lock after 80 seconds. Without multipath mitigation, such as correlator-based techniques or advanced filtering, the pseudorange and Doppler measurements degrade, amplifying residuals in the WLS solution and leading to divergence. 

### Velocity estimation
For velocity, pseudorange rates $`\dot{\rho}`$ derived from Doppler measurements are used: $`\dot{\rho} = -\mathbf{v}_{\text{sat}} \cdot \mathbf{u} + \mathbf{v}_{\text{rec}} \cdot \mathbf{u} + \epsilon_v`$, where $`\mathbf{v}_{\text{sat}}`$ and $`\mathbf{v}_{\text{rec}}`$ are satellite and receiver velocities, and $`\mathbf{u}`$ is the line-of-sight vector. The velocity $`\mathbf{v}_{\text{rec}} = [v_x, v_y, v_z]^T`$ is solved via $`\mathbf{v}_{\text{rec}} = (A_v^T W_v A_v)^{-1} A_v^T W_v \mathbf{b}_v`$, with $`A_v`$ as the velocity design matrix and $`W_v`$ mirroring $`W`$. Weights enhance accuracy by prioritizing high-quality observations.

```matlab
...
vel = zeros(3, 1); 
if ~isempty(doppler) && ~isempty(satvelocity)
       lamda = settings.c / 1575.42e6;
       rate = -lamda * doppler; 
       rate = rate'; 
       satvelocity = satvelocity'; 
       b = zeros(nmbOfSatellites, 1); 
       Wv = zeros(nmbOfSatellites, nmbOfSatellites); 
       for i = 1:nmbOfSatellites
               b(i) = rate(i) - satvelocity(i, :) * (A(i, 1:3))';
               Wv(i, i) = max(0.1, sin(el(i) * dtr)^2);
    end

       A_v = A(:, 1:3);     
       vel = (A_v' * Wv * A_v) \ (A_v' * Wv * b);
end
...
```
### Results
![image](https://github.com/superrichme/yiweixu.github.io/blob/main/task4_2.png)

### Result analysis

The velocity estimation through Weighted Least Squares (WLS) in the UTM system plots reveals horizontal velocity errors over time across two scenarios: OpenSky (left) and Urban (right). In the OpenSky scenario, errors fluctuate between -50 m/s and +50 m/s, which reflects stable satellite visibility and minimal multipath effects, ensuring consistent WLS performance. Conversely, the Urban scenario displays larger errors, spanning from -500 m/s to +500 m/s, with notable peaks after 70 seconds. Obstructed satellite signals, multipath interference, and a reduced number of visible satellites in urban canyons likely cause this, diminishing the accuracy of the WLS solution. The WLS weighting, often based on elevation angles, faces challenges in Urban environments where low-elevation satellites prevail, thus amplifying errors. Moreover, signal reflections in urban areas introduce biases into Doppler measurements, which further affect velocity estimates. The OpenSky scenario, with its superior geometry and signal quality, yields lower errors.


Task 5 – Kalman filter-based positioning.   
Develop an Extended Kalman Filter (EKF) using pseudorange and Doppler measurements to estimate user position and velocity.
-------------------
