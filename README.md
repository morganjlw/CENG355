# CENG355: PWM Signal Generation and Monitoring System

![](https://github.com/morganjlw/CENG355/blob/master/STM32F0.jpg)

## Summary
STM32F0 based project to control a PWM signal frequency using an optocoupler output that is regulated by a measured potentiometer input. The microcontroller will also be used to measure the frequnecy of the generated PWM signal. The measured frequnecy and the corresponding POT resistance are displayed on the LCD. Project written in C.

## Objective
The objective of the project is to develop an embedded system for monitoring and controlling a pulse-width modulated (PWM) signal that will be generated by an external clock signal (NE555 IC). An external optocoupler (4N35 IC), driven by the microcontroller on the STMF0xx Discovery board, will be used to control the frequency of the PWM signal.

## System Specifications
Measure POT resistance using ADC and PWM using STM32F0 and output POT resistance and analog clock signal frequency to the LCD screen. This is achieved by using the STM32F051R8T6 MCU mounted on the STM32F0 Discovery board and taking advantage of the built-in analog-to-digital converter (ADC), digital-to-analog converter (DAC), and serial peripheral interface (SPI). The DAC will be used to drive the 4N35 optocoupler to adjust the signal frequency of the NE555 timer, based on the potentiometer voltage read by the ADC, and the SPI will be used to communicate with the LCD.

The analog voltage signal coming from the potentiometer on the PBMCUSLK board will be measured by polling the ADC. Using ADC voltage measurements, a corresponding potentiometer resistance value is calculated. The ADC measurement is normalized for the lower and the upper limits of the measured voltage.

The digital value obtained from the ADC is used to adjust the frequency of the PWM signal generated by the NE555 timer. For that purpose, the DAC is used to convert that digital value to an analog voltage signal driving the 4N35 optocoupler.
To display the signal frequency and the potentiometer resistance, the configured SPI is used to drive the LCD on the PBMCUSLK board. The LCD is a 4-bit, 2-by-8 character display with no ability to directly write to the LCD pins. The LCD pins are controlled through the 8-bit 74HC595 shift register on the PBMCUSLK board. The 74HC595 shift register receives 8-bit words from the SPI via the serial MOSI port using the correctly timed latch clock LCK and the serial shift register clock SCK. Each 8-bit word sent via SPI includes LCD data bits D3-D0, register-select bit RS, and enable bit EN.

CMSIS-defined software library functions are used to configure and control STM32F0 I/O.

## System Design
### Hardware Design
![](https://github.com/morganjlw/CENG355/blob/master/Hardware%20Schematic.JPG)
### Software Design
![](https://github.com/morganjlw/CENG355/blob/master/SoftwareDiagram.JPG)
The system executes initializations of the timers, the delay function (that incorporates TIM3), the GPIO, ADC, DAC, EXTI, and LCD. Following the initialization of the system peripherals, the potentiometer analysis algorithm is implemented using polling to update potentiometer measurements. Frequency analysis is toggled by the EXTI interrupt and used to measure and update frequency. Inside the same polling function as potentiometer analysis, delay is toggled before the updated potentiometer and frequency values are written to the using the LCD Write function. Following the delay function execution (inside the same polling loop), the updated frequency and potentiometer values are displayed using the LCD Write function.
### Potentiometer Analysis
Potentiometer resistance is measured by the STMF0’s ADC that receives an analog input a voltage from the PBMCUSLK’s potentiometer. The ADC is enabled, mapped to GPIO, configured to continuous conversion and Overrun mode, and must be calibrated before it can perform conversions (Lines 428-452). Interpreting the potentiometer’s input voltage signal requires an ADC conversion initiation (Line 156). Once a conversion is completed (Line 159 and 161), the ADC interprets this input voltage signal as a 12-bit right-aligned value and stores it in the ADC1 DR register. This value is pass to the DAC’s DHR12R1 to be output. The DHR12R1 register is a data holding register that is formatted for 12 bit right aligned data and since the DAC is defaulted to “single mode”, this register automatically transferred to the DAC output GPIO pin (Line 167).

To convert the ADC data register’s value to a usable format, the ADC data register mask must be applied (Line 170). The ADC’s measured potentiometer value must be normalized for displaying so, we divide it by the maximum ADC value and then multiply by the potentiometer’s maximum value (Line 173). This normalized value is stored in “gbPOT_Resistance” and is displayed on the LCD.

### Frequency Analysis
Frequency is calculated using EXTI interrupts and TIM2’s count register. GPIO Pin A1 is mapped to EXTI interrupt line 0 to trigger at rising edges to enable rising edge detection. The EXTI interrupt handler (Line 455) distinguishes between initial and final rising edges by enabling TIM2 count at the first detected rising edge and using a condition that checks if TIM2 count is enabled (Line 460) to detect the second edge. When the second edge is detected, frequency is calculated by dividing the system core clock (48MHz) by the value stored in TIM2’s CNT register (Line 468). The frequency measurement is accurate and does not require normalization (treated as a float). The variable responsible for storing the frequency “gb_frequency” is global because it is constantly updated and passed into “Writes_Values” (Line 96) to update the LCD.

### Frequency Measurement Accuracy
Using the PWM signal generation configuration in figure 1, the frequency range generated spanned from ~ (827-1301) Hz. Comparing the frequency displayed on the LCD to a reference oscilloscope, the frequency measurements differed less than 0.001%. This is a very impressive result and bears testament to the accuracy of the TIM2 timer and the speed of the EXTI interrupt in our program.

### Operating Frequency Range
The frequency calculation algorithm utilised can accept low frequencies without issue (even 0 Hz). Given the configuration of our PWM signal generation circuit (using R1 = R2 = 5.5K and C = 0.1uF), the high frequencies of this system are measured extremely accurately (@approx. 1.302 KHz).

The theoretical maximum input frequency the system is capable of reading is the system core clock frequency (48 MHz) because the TIM2 CNT register would count once and thus, F = 48MHZ/1 = 48MHZ . This of course is a complete fantasy because of the myriad of other external factors (program execution time, circuit logic delay, etc.) that contribute to obscuring the frequency measurement. It is also difficult to track the maximum frequency because functions like trace_printf() are not lightweight and can obscure frequency measurements. The only accurate way we have found to track the maximum input frequency is to use the Eclipse debugger to monitor the variable “gb_frequency” instead of trace_printf().

It is important to note that our display is optimized to display our configured frequency range (824-1300 Hz) but our actual measurement is capable of vastly exceeding this range.

The maximum accurately measurable input frequency: 700 KHz.

### Measurements
#### DAC Output Voltage
Maximum DAC output voltage was measured by producing maximum ADC reading (setting potentiometer to 5K) and connecting a DMM to PA4. The maximum DAC output voltage measured: 2.15 V. The minimum output voltage (setting potentiometer to 0) was 56.7 mV.

#### Minimum 4N35 Optocoupler Resistance
The minimum 4N35 phototransistor resistance is measured by inputting the maximum DAC input voltage (@Max. POT reading) and connecting a DMM to the output terminals. The measured minimum resistance was: 312 Ω.

#### 4N35 "Dead-Band" Range
The 4N35 consists of a LED that accepts a forward voltage to vary the phototransistor resistance. An important diode characteristic is the minimum forward voltage required to enable current to pass through. This is often referred to as a “Dead-Band” range because if the forward voltage is insufficient, the diode will act as an open circuit (infinite resistance). To find the maximum dead-band voltage, first I connected a variable power supply to the input pins (1 and 2) and connected a DMM to the output pins. Finding the maximum dead-band voltage is achieved by varying the input voltage until the DMM reads a resistance not equal to ~infinite (*I am aware this minimum forward voltage is listed in the data sheet). The measured maximum dead-band voltage was: 0.98V. This slightly differs from the minimum forward voltage described in the 4N35’s datasheet (0.9V).





