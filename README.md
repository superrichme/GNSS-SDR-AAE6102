# AAE6102-Assignment1
================
Task 1 – Acquisition 
Process the IF data using a GNSS SDR and generate initial acquisition outputs. 
---------------
This algorithm in SoftGNSS is a cold-start acquisition method for GPS signals based on frequency-domain correlation, utilizing a three-step approach consisting of initialization, coarse and fine acquisition stages.

Step 1: Initialization
Calculate the number of samples per C/A code (samplesPerCode) based on sampling frequency, code frequency, and code length. Extract two 1-ms signal segments (signal1, signal2) for coarse acquisition, and generate a DC-removed signal (signal0DC) for fine acquisition. Compute sampling period (ts) and phase points (phasePoints). Define frequency search range (500 Hz steps, settings.acqSearchBand) and initialize frequency bins (frqBins). Generate local C/A code table (caCodesTable) and initialize result arrays (results, acqResults). 

Step 2: Coarse Acquisition
For each PRN, compute the frequency-domain C/A code conjugate (caCodeFreqDom) via FFT. For each frequency bin, generate local carrier (sigCarr), remove carrier from signal1 and signal2 to get I/Q components, and perform FFT to get IQfreqDom1 and IQfreqDom2. Build the correlation function by multiplying with caCodeFreqDom, then IFFT to obtain correlation power (acqRes1, acqRes2). Select the higher power segment for results. Identify the peak, frequency, and code phase, compute the peak ratio, and proceed to fine acquisition if above threshold.

Step 3: Fine Acquisition
For each detected PRN, generate a 10-ms C/A code sequence (longCaCode), remove code modulation from signal0DC using the coarse code phase to get the carrier component (xCarrier). Perform high-resolution FFT (8x nearest power of 2) on xCarrier, compute frequency bins (fftFreqBins), and estimate precise carrier frequency (acqResults.carrFreq) based on the maximum magnitude. Retain the coarse code phase (acqResults.codePhase) as the final estimate. 

The core of the algorithm is to efficiently search all possible code phases and frequency offsets using frequency-domain correlation, enhancing reliability with peak ratio analysis, and outputting the carrier frequency and code phase for each PRN.
