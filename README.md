# STM32_Motion-Analysis-Using-FFT
Real-time FFT-based motion detection on STM32F4 (Cortex-M4) using CMSIS-DSP. Accelerometer data from MPU6050 is processed through a full DSP pipeline (DC removal, Hanning window, RFFT, RMS gating, band-limited peak detection, overlap processing) to classify standing, walking, and running based on dominant frequency analysis.
