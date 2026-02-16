# 🚀 FFT-Based Motion Detection on STM32F4

Real-time motion classification using frequency-domain analysis on an STM32F4 (Cortex-M4) microcontroller.

Instead of traditional threshold-based motion detection, this project implements a full DSP pipeline using **CMSIS-DSP** to analyze accelerometer signals in the frequency domain.

---

## 📌 Overview
This project captures accelerometer data from the **MPU-6050** over I2C and processes it using an optimized FFT pipeline to classify motion states:

* 🧍 **Standing** (Idle)
* 🚶 **Walking** (Low-frequency rhythmic motion)
* 🏃 **Running** (High-frequency rhythmic motion)

The system leverages the hardware **FPU** (Floating Point Unit) and **DSP instructions** available in the Cortex-M4 for efficient real-time signal processing.

## 🧠 Why Frequency-Domain?
Traditional motion detection often uses simple logic: `if(acceleration > threshold) → motion`. 

**The FFT Approach Advantages:**
* **Periodic Pattern Detection:** Differentiates between a random bump and a rhythmic gait.
* **Noise Sensitivity:** Filters out high-frequency mechanical vibrations.
* **Robustness:** Enables actual activity classification rather than just "moving/not moving."

---

## ⚙️ System Architecture

### 🔹 Hardware
* **MCU:** STM32F4 Series (Cortex-M4)
* **Sensor:** MPU-6050 (3-Axis Accelerometer) via I2C
* **Clock:** Configured for maximum performance (e.g., 168MHz)

### 🔹 Sampling & Buffering
* **Sampling Rate:** 200–500 Hz
* **Buffer Size ($N$):** 256 samples
* **Overlap:** 50% (128 samples shifted) to improve temporal resolution.

---

## 🔬 DSP Processing Pipeline

### 1. Acceleration Magnitude
To make detection orientation-independent, we calculate the magnitude:
$$|A| = \sqrt{A_x^2 + A_y^2 + A_z^2}$$

### 2. DC Removal & Windowing
* **DC Removal:** Subtract the mean to remove the $1g$ gravity component.
* **Hanning Window:** Applied to reduce spectral leakage before the FFT.
    ```c
    float window = 0.5f * (1.0f - arm_cos_f32((2.0f * PI * i) / (FFT_SIZE - 1)));
    ```

### 3. FFT Execution (CMSIS-DSP)
We use the **Real FFT (RFFT)** function. Since accelerometer data is real-valued, RFFT is twice as fast and uses half the memory of a Complex FFT (CFFT).
```c
arm_rfft_fast_f32(&fft_instance, fft_input, fft_output, 0);
arm_cmplx_mag_f32(fft_output, fft_mag, FFT_SIZE/2);
