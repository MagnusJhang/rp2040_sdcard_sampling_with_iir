# Project of ADC sampling for RP2040

## IDE
It is highly recommanded to used the [Visual Studio Code](https://code.visualstudio.com/) IDE with [PlatformIO](https://platformio.org/) plugin

## Hardware
* Raspberry Pi Pico
* SD-card Module (SPI)
* SD-card
* Button * 1
* LED * 1
## Pin Connection
* ADC pin: A0
* SD-Card.CLK: GP18
* SD-Card.CS: GP17
* SD-Card.MOSI: GP19
* SD-Card.MISO: GP16
* System State LED: ACTIVATE_LED_PIN(GP21) & D25(On board LED)
* Button Trigger: GEN_NOISE_BUTTON_PIN(GP22) 
    * Trigger Mode: RISING

``` C++
attachInterrupt(digitalPinToInterrupt(GEN_NOISE_BUTTON_PIN), btn_dac_callback, RISING);
```

* Sampling Rate: 360.0 

``` C++
#define TIMER_FREQ_HZ 360.0
```

* Sampling Time: 180 Seconds 

``` C++
#define SAMPLING_CNT (64800) //3 Min
```
## How to use
Just press the button, and the RP2040 will record ADC value for 3 minutues after LED blink (fast).

``` C++
if (intr_trig_dac & !timer_en_dac) { // It would be triggered after press the button
    intr_trig_dac = false;
    timer_en_dac = true;
    rec_main_num = create_new_record(RECORD_ADC_DIR, FLAG_CREATE_NEW_RECORD);
    if (rec_main_num == FLAG_CREATE_NEW_RECORD_FAIL) error_led(ERROR_CREATE_RECORD_FILE);

    for (int i = 0; i < 50; i++) {
      digitalWrite(ACTIVATE_LED_PIN, !digitalRead(ACTIVATE_LED_PIN));
      delay(100);
    }
    digitalWrite(ACTIVATE_LED_PIN, HIGH);
    rd_pos = 0;
    wr_pos = 0;
    sample_idx = 0;
    Serial.println(F("Enable Timer"));
    st_time = millis();
    ITimer.attachInterrupt(TIMER_FREQ_HZ, TimerHandler);

  }
```

## IIR Filters

generate from MATLAB (matlab/iir_60hz.m & matlab/iir_120hz.m)

* 19 order 60 Hz cheby2 IIR Notch Filters
* 15 order 120 Hz cheby2 IIR Notch Filters

* Coefficient File: 
    * sdcard/a_coef_60.bin
    * sdcard/b_coef_60.bin
    * sdcard/a_coef_120.bin
    * sdcard/b_coef_120.bin

## SDcard Structural
* Create the RECORD folder for sdcard
* Copy the coefficient file (x_coef_xx.bin) to the root folder

```
SDCard
  | RECORD
    | REC_00_00.csv 
    | REC-01_00.csv
    | ...
  | a_coef_60.bin
  | b_coef_60.bin
  | a_coef_120.bin
  | a_ceof_120.bin
```

## REC_xx_00.csv

There are two values would be write int the recode file:

* value: the non-filtered value
* filtered: the value after 2 IIR filters

``` C++ 

bool write_SD2() {
  uint32_t t_wr_pos = wr_pos;
  char buff[50] = "";
  double filtered_value;
  if (rd_pos < t_wr_pos) {
    Serial.printf("-");
    while (rd_pos < t_wr_pos) {
//       Serial.println(data_logger[rd_pos++]);
      filtered_value = iir_120->filter(iir_60->filter(data_logger[rd_pos++]));
      sprintf(buff, "%d, %f\r\n", data_logger[rd_pos], filtered_value);
      rec_file.write(buff);
      rd_pos++;
    }
  } 
  .
  .
  .

```

# Schematics of Record file
``` 
value, filtered
3593.000000, 1499.554219
350.000000, 1257.036646
1819.000000, 1626.155630
3592.000000, 1865.304461
378.000000, 2739.257669
1819.000000, 2682.855024
3610.000000, 1040.279861
353.000000, 1623.142127
1822.000000, 2291.014175
3594.000000, 1705.452923
363.000000, 1931.639797
1805.000000, 2076.769067
3596.000000, 2035.031324
369.000000, 1890.992849
1806.000000, 1792.847916
3603.000000, 2155.584903
361.000000, 1679.946912
1798.000000, 1563.630937
.
.
```

It could be simple read and plot by the matlab:
``` matlab
clc; close all; clear;

data = readmatrix("REC_00_00.csv");
non_filtered = data(:, 1);
filtered = data(:, 2);
plot(non_filtered, "displayname", "non-filtered value");
hold on;
plot(filtered, "displayname", "filtered value");
legend on;
hold off;

```
