# Vibration Immune Smart Weighing Scale

An embedded systems project that uses an STM32 microcontroller, a load cell, an HX711 load-cell amplifier, and an OLED display to measure weight. Digital Signal Processing (DSP) techniques, including FIR filtering and FFT-based frequency analysis, are used to reduce measurement fluctuations and analyze vibration components.

## 📌 Project Overview

Electronic weighing scales can produce unstable readings when exposed to mechanical vibrations, movement, and electrical noise. These disturbances can affect the reliability of weight measurements.

This project aims to improve measurement stability by combining load-cell-based weight measurement with digital signal processing. The STM32 microcontroller acquires digital readings from the HX711, processes the measurements, analyzes frequency components, and displays weight information on an OLED display.

## 🎯 Objectives

- Develop a digital weighing scale using an STM32 microcontroller.
- Measure weight using a load cell and HX711 module.
- Reduce unwanted fluctuations using a Finite Impulse Response (FIR) low-pass filter.
- Analyze vibration-related frequency components using the Fast Fourier Transform (FFT).
- Implement weight calibration and stability detection.
- Display measurement information on an SSD1306 OLED.
- Evaluate filtering performance using MATLAB simulation.

## 🛠️ Hardware Components

| Component | Purpose |
|---|---|
| STM32 NUCLEO-F411RE | Main controller and DSP processing unit |
| Load cell | Converts applied force into a small electrical signal |
| HX711 module | Amplifies and digitizes the load-cell signal |
| SSD1306 OLED (128 × 64) | Displays weight and measurement information |
| Connecting wires | Connect the hardware components |
| Computer | Firmware development, debugging, and serial monitoring |

## 💻 Software and Tools

- **STM32CubeIDE** — Firmware development, compilation, and debugging
- **STM32CubeMX** — Microcontroller peripheral configuration
- **MATLAB** — DSP simulation and performance evaluation
- **C programming language** — STM32 firmware implementation
- **UART Serial Monitor** — Debugging and monitoring measurement data

## 🔌 Hardware Connections

The following table shows the pin assignments used in the project configuration.

### HX711 to STM32

| HX711 Pin | STM32 Pin |
|---|---|
| DT / DOUT | PA0 |
| SCK / CLK | PA1 |
| VCC | Compatible supply voltage |
| GND | GND |

### SSD1306 OLED to STM32

| OLED Pin | STM32 Pin |
|---|---|
| SDA | PB9 |
| SCL | PB8 |
| VCC | 3.3 V |
| GND | GND |

The OLED communicates with the STM32 through the I²C1 peripheral at a configured clock frequency of 100 kHz.

### UART Serial Communication

| USART2 Signal | STM32 Pin |
|---|---|
| TX | PA2 |
| RX | PA3 |

The serial connection can be used to monitor debugging messages and measurement information through the ST-LINK Virtual COM Port. The configured serial settings are 115200 baud, 8 data bits, no parity, and 1 stop bit.

**Hardware note:** Verify the HX711 module's supply-voltage requirements and logic compatibility before connecting it to the STM32. All connected modules must share a common ground.

## ⚙️ Working Principle

The system operates through the following stages:

1. **Weight sensing:** The load cell generates an electrical signal in response to the applied load.
2. **Signal amplification and conversion:** The HX711 amplifies the load-cell signal and converts it into digital measurement data.
3. **Data acquisition:** The STM32 reads the digital measurements from the HX711.
4. **Digital filtering:** An FIR low-pass filter processes the sampled signal to attenuate unwanted fluctuations.
5. **Frequency analysis:** FFT processing analyzes the frequency components of the measured signal to help identify dominant vibration frequencies.
6. **Weight calibration:** Sensor readings are converted into weight units using calibration information.
7. **Stability detection:** The firmware evaluates whether the weight readings have become sufficiently stable.
8. **Display:** The OLED presents the relevant measurement information.

### System Block Diagram

```text
        Applied Weight
              |
              v
          Load Cell
              |
              v
        HX711 Module
   (Amplification and ADC)
              |
              v
       STM32F411RE MCU
              |
       +------+------+
       |             |
       v             v
   FIR Filtering   FFT Analysis
       |             |
       v             v
 Weight Processing  Frequency Analysis
       |
       v
 Stability Detection
       |
       v
    OLED Display
```

The diagram illustrates the main processing functions. The exact execution sequence and data flow depend on the firmware implementation.

## 📊 Digital Signal Processing

### 1. FIR Low-Pass Filtering

A Finite Impulse Response (FIR) filter combines the current and previous input samples using filter coefficients to calculate each output sample.

The general equation is:

\[
y[n] = \sum_{k=0}^{N} b_k x[n-k]
\]

Where:

- \(y[n]\) is the filtered output.
- \(x[n-k]\) represents the input samples.
- \(b_k\) represents the FIR filter coefficients.
- \(N\) is the filter order.

The low-pass filter attenuates higher-frequency components to help reduce rapid fluctuations in the weight signal.

### 2. FFT-Based Frequency Analysis

The Fast Fourier Transform (FFT) efficiently calculates the Discrete Fourier Transform (DFT), allowing the signal to be examined in the frequency domain.

FFT analysis can help identify dominant frequency components that may be associated with vibration.

The STM32 firmware uses a 256-point FFT and a Hann window for frequency analysis.

### 3. Sampling Parameters

The nominal HX711 sampling rate used in the project is 80 samples per second.

| Parameter | Value |
|---|---:|
| Nominal sampling frequency | 80 Hz |
| FFT size | 256 samples |
| Observation duration for 256 samples at 80 SPS | 3.2 seconds |
| Frequency-bin spacing at 80 SPS | 0.3125 Hz |
| Nyquist frequency at 80 SPS | 40 Hz |
| FFT analysis window | Hann window |

The frequency-bin spacing is calculated using:

\[
\Delta f = \frac{F_s}{N}
\]

For \(F_s = 80\) Hz and \(N = 256\):

\[
\Delta f = \frac{80}{256} = 0.3125\text{ Hz}
\]

The firmware measures the actual sampling rate during tare and uses the measured value for filter and FFT frequency calculations. Therefore, the values above are based on the nominal 80 SPS rate.

## 🧪 MATLAB Simulation

MATLAB is used to simulate the digital signal processing approach and evaluate the effect of FIR filtering on the measured weight signal.

### Simulation Parameters

| Parameter | Value |
|---|---:|
| Sampling frequency | 80 Hz |
| FIR filter order | 40 |
| Number of FIR coefficients | 41 |
| Low-pass cutoff frequency | 5 Hz |
| FFT size | 256 samples |
| Reference weight | 2 kg |
| Simulated vibration frequency | 20 Hz |
| FIR group delay | 20 samples (0.25 seconds) |

### Simulation Results

| Performance Metric | Simulated Result |
|---|---:|
| Raw signal RMS error | 0.036239 kg |
| Filtered signal RMS error | 0.003285 kg |
| Approximate RMS error reduction | 90.93% |

The simulated RMS error reduction is calculated as:

\[
\text{Error Reduction}(\%) =
\frac{RMSE_{\text{raw}}-RMSE_{\text{filtered}}}
{RMSE_{\text{raw}}}\times100
\]

Using the simulated values:

\[
\text{Error Reduction} \approx 90.93\%
\]

These results indicate that FIR filtering reduced the RMS error under the simulated test conditions.

**Important:** The values in this section are MATLAB simulation results. They should not be interpreted as measured performance of the physical STM32 device.

The MATLAB simulation uses a 5 Hz FIR cutoff frequency, whereas the STM32 firmware is configured with a 3 Hz FIR cutoff frequency. The two configurations should be evaluated separately.

## 🚀 Getting Started

### Prerequisites

- STM32 NUCLEO-F411RE development board
- Compatible load cell and HX711 module
- SSD1306 I²C OLED display
- STM32CubeIDE
- USB connection for programming and debugging

### Setup Steps

1. Clone or download this repository.
2. Open STM32CubeIDE.
3. Import the STM32 project into the workspace.
4. Verify the pin assignments and peripheral configuration.
5. Connect the load cell, HX711, and OLED according to the connection tables.
6. Build the project and resolve any compilation errors.
7. Connect the NUCLEO-F411RE board to the computer through USB.
8. Flash the firmware onto the microcontroller.
9. Open a serial terminal if UART debugging is enabled.
10. Apply a known weight and perform the required calibration before evaluating measurements.

**Note:** These steps describe the general setup procedure. Exact build instructions may vary depending on the source files and project structure included in the repository.

## 📈 Expected Outcome

The project is designed to:

- Measure weight using a load cell and HX711.
- Process measurements using an STM32 microcontroller.
- Reduce unwanted fluctuations through FIR filtering.
- Analyze dominant signal frequencies using FFT.
- Detect sufficiently stable weight readings.
- Display measurement information on an OLED screen.

Actual performance depends on calibration, sensor characteristics, mechanical mounting, vibration conditions, filter settings, and firmware implementation.

## ⚠️ Limitations

- FIR filtering introduces processing delay.
- Excessive filtering may slow the response to genuine weight changes.
- FFT frequency resolution depends on the sampling rate and FFT size.
- Accurate weight measurement requires proper calibration and mechanical setup.

## 🔮 Future Improvements

- Test the system under controlled physical vibration conditions.
- Compare raw and filtered measurements using experimental RMS error.
- Tune FIR filter parameters for different operating conditions.
- Improve stable-weight detection and response time.
- Develop and validate vibration-source classification using real measurements.
- Add data logging and graphical visualization of weight and vibration signals.
- Evaluate performance with different loads and vibration frequencies.

## 📚 Key Concepts

This project demonstrates the practical application of:

- Embedded systems
- STM32 microcontroller programming
- Load-cell interfacing
- HX711 analog-to-digital conversion
- Digital signal processing
- FIR digital filtering
- Convolution
- Fast Fourier Transform (FFT)
- Sampling and frequency resolution
- Weight calibration
- Stable-weight detection
- I²C and UART communication

## 👨‍💻 Project Summary

The **Vibration Immune Smart Weighing Scale** combines sensor interfacing, embedded firmware, and digital signal processing to investigate more stable weight measurement under vibration. The STM32F411RE handles data acquisition and processing, the FIR filter reduces unwanted signal fluctuations, FFT analysis examines frequency components, and the OLED provides measurement information.

MATLAB simulation is used to evaluate the filtering approach, while further controlled hardware testing is required to establish real-world accuracy and vibration immunity.
