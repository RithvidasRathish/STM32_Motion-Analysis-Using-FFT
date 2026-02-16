# 🚀 FFT-Based Motion Detection on STM32F4

Real-time motion classification using frequency-domain analysis on an STM32F4 (Cortex-M4) microcontroller.

Instead of traditional threshold-based motion detection, this project implements a full DSP pipeline using CMSIS-DSP to analyze accelerometer signals in the frequency domain.

---

## 📌 Overview

This project captures accelerometer data from the MPU-6050 over I2C and processes it using an optimized FFT pipeline to classify motion states such as:

- 🧍 Standing  
- 🚶 Walking  
- 🏃 Running  

The system leverages hardware FPU and DSP instructions available in the Cortex-M4 for efficient real-time signal processing.

