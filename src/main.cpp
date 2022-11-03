//#include <FIRFilter.h>
#include <Arduino.h>
#include <IIRFilter.h>

#include <RPi_Pico_TimerInterrupt.h>
#include <RPi_Pico_ISR_Timer.h>
#include <RPi_Pico_ISR_Timer.hpp>
#include <malloc.h>
#include "SdFat.h"
#include "sdios.h"
#include "FreeStack.h"

// ========================SD_SPI_SETUP===========================
#ifndef SDCARD_SS_PIN
const uint8_t SD_CS_PIN = SS;
#else  // SDCARD_SS_PIN
// Assume built-in SD is used.
const uint8_t SD_CS_PIN = SDCARD_SS_PIN;
#endif  // SDCARD_SS_PIN

// Try max SPI clock for an SD. Reduce SPI_CLOCK if errors occur.
#define SPI_CLOCK SD_SCK_MHZ(50) //pico
//#define SPI_CLOCK SD_SCK_MHZ(40) //pico

// Try to select the best SD card configuration.
#if HAS_SDIO_CLASS
#define SD_CONFIG SdioConfig(FIFO_SDIO)
#elif ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SPI_CLOCK)
#else  // HAS_SDIO_CLASS
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SPI_CLOCK)
#endif  // HAS_SDIO_CLASS
// ==========================FLAG==================================
#define FLAG_CREATE_NEW_RECORD -1
#define FLAG_CREATE_NEW_RECORD_FAIL -1
#define ERROR_SDCARD_INIT_FAIL 0
#define ERROR_CREATE_RECORD_FILE 1
#define ERROR_WRITE_FILE_FAIL 3
// =========================ADC_PREF===============================
#define TIMER_FREQ_HZ 360.0
#define ADC_BUFF_SIZE (21600)
#define DEBOUNCE_DELAY 500
#define SAMPLING_CNT (64800) //3 Min
// =======================ADC_SD_FILE==============================
#define RECORD_ADC_DIR "/RECORD"
#define RECORD_ADC_PREFIX "REC_"
#define RECORD_ADC_POSTFIX ".csv"
// =======================PIN==============================
#define GEN_NOISE_BUTTON_PIN D22
#define INTERRUPT_LED_PIN LED_BUILTIN
#define ACTIVATE_LED_PIN D21
#define ADC_PIN A0
// ================IIR Filters======================
typedef double adc_unit;

adc_unit* data_logger;

volatile uint32_t wr_pos = 0;
volatile uint32_t rd_pos = 0;
volatile uint32_t sample_idx = 0;
bool led_stat;

SdFat sd;
SdFile rec_file;

RPI_PICO_Timer ITimer(0);
IIRFilter* iir_60;
IIRFilter* iir_120;

uint32_t lastDebounceTime = 0;
volatile bool timer_en_dac = false;
volatile bool intr_trig_dac = false;
volatile bool timer_en_dac_pr = false;
volatile uint16_t rec_main_num;
float st_time = 0.0;

void error_led(int mode) {
  switch (mode) {
    case ERROR_SDCARD_INIT_FAIL:
      Serial.println(F("Error detect: SDcard initialized failure."));
      while (true) {
        digitalWrite(INTERRUPT_LED_PIN, HIGH);
        digitalWrite(ACTIVATE_LED_PIN, LOW);
        delay(400);
        digitalWrite(INTERRUPT_LED_PIN, LOW);
        digitalWrite(ACTIVATE_LED_PIN, HIGH);
        delay(400);
      }
  }
}

uint16_t create_new_record(String dir, int main_num) {
  int temp = 0;
  char fname[25];
  do {
    if (main_num == FLAG_CREATE_NEW_RECORD)
      sprintf(fname, "%s/%s%02d_00%s", RECORD_ADC_DIR, RECORD_ADC_PREFIX, temp, RECORD_ADC_POSTFIX);
    else
      sprintf(fname, "%s/%s%02d_%02d%s", RECORD_ADC_DIR, RECORD_ADC_PREFIX, main_num, temp, RECORD_ADC_POSTFIX);

    if (!sd.exists(fname)) {
      if (rec_file.open(fname, FILE_WRITE)) {
        Serial.print("File created: ");
        Serial.println(fname);
        rec_file.print(F("value, filterd\r\n"));
        break;
      }
      return FLAG_CREATE_NEW_RECORD_FAIL;
    }
    temp++;
  } while (true);

  return temp;
}


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
    
  } else {
    Serial.printf("*");
    while (rd_pos < ADC_BUFF_SIZE) {
//       Serial.println(data_logger[rd_pos++]);
      filtered_value = iir_120->filter(iir_60->filter(data_logger[rd_pos++]));
      sprintf(buff, "%d, %f\r\n", data_logger[rd_pos], filtered_value);
      rec_file.write(buff);
      rd_pos++;
    }
    rd_pos = 0;
    while (rd_pos < t_wr_pos) {
//      Serial.println(data_logger[rd_pos++]);
//      sprintf(buff, "%f\r\n", data_logger[rd_pos++]);
      filtered_value = iir_120->filter(iir_60->filter(data_logger[rd_pos++]));
      sprintf(buff, "%d, %f\r\n", data_logger[rd_pos], filtered_value);
      rec_file.write(buff);
      rd_pos++;
    }
  }
//  rec_file.flush();
  return rec_file.sync();
}

bool TimerHandler(struct repeating_timer *t)
{
  if (sample_idx < SAMPLING_CNT) {
    digitalWrite(INTERRUPT_LED_PIN, !digitalRead(INTERRUPT_LED_PIN));
    sample_idx++;
    uint16_t val_a0 = analogRead(ADC_PIN);
    
    
    // data_logger[wr_pos++] = val_a0 >> 4;
    // data_logger[wr_pos++] = (val_a0 & 0x000f);

    data_logger[wr_pos++] = val_a0;

    if (wr_pos == ADC_BUFF_SIZE)
      wr_pos = 0;
    //    REG_CHANGE_SHIFT(REG_PIOB_ODSR, 0x03, 26);
  } else {
    //    Timer3.stop();
    ITimer.stopTimer();
    write_SD2();
    rec_file.close();
    Serial.print("Done: ");
    Serial.println((millis() - st_time) / 1000.0);
    digitalWrite(INTERRUPT_LED_PIN, LOW);
    timer_en_dac = false;
  }
  return true;
}

#define IIR_60_ORDER 19
#define IIR_120_ORDER 15
void read_iir_coef() {
  SdFile file;
  void *buff = malloc(sizeof(double));
  double a_coefficients[IIR_60_ORDER];
  double b_coefficients[IIR_60_ORDER];

  if(!file.open("/a_coef_60.bin", O_READ)){
    error_led(ERROR_SDCARD_INIT_FAIL);
  }
  
  for(int i=0; i<IIR_60_ORDER; i++){
    file.read(buff, 8);
    a_coefficients[i] = *((double *)buff);
    Serial.printf("coef_a[%d]=%f\n", i, a_coefficients[i]);
//    Serial.println(a_coefficients[i]);
  }
  file.close();

  if(!file.open("/b_coef_60.bin", O_READ)){
    error_led(ERROR_SDCARD_INIT_FAIL);
  }
  
  for(int i=0; i<IIR_60_ORDER; i++){
    file.read(buff, 8);
    b_coefficients[i] = *((double *)buff);
    Serial.printf("coef_b)60[%d]=%f\n", i, b_coefficients[i]);
//    Serial.println(a_coefficients[i]);
  }
  file.close();

  iir_60 = new IIRFilter(b_coefficients, a_coefficients);

  double a_coefficients_120[IIR_120_ORDER];
  double b_coefficients_120[IIR_120_ORDER];

  if(!file.open("/a_coef_120.bin", O_READ)){
    error_led(ERROR_SDCARD_INIT_FAIL);
  }
  
  for(int i=0; i<IIR_120_ORDER; i++){
    file.read(buff, 8);
    a_coefficients_120[i] = *((double *)buff);
    Serial.printf("coef_a[%d]=%f\n", i, a_coefficients_120[i]);
//    Serial.println(a_coefficients[i]);
  }
  file.close();

  if(!file.open("/b_coef_120.bin", O_READ)){
    error_led(ERROR_SDCARD_INIT_FAIL);
  }
  
  for(int i=0; i<IIR_120_ORDER; i++){
    file.read(buff, 8);
    b_coefficients_120[i] = *((double *)buff);
    Serial.printf("coef_b)60[%d]=%f\n", i, b_coefficients_120[i]);
//    Serial.println(a_coefficients[i]);
  }
  file.close();
  iir_120 = new IIRFilter(b_coefficients_120, a_coefficients_120);
  
}

void btn_dac_callback() {
  //  digitalWrite(13, !digitalRead(13));
  if (timer_en_dac | intr_trig_dac)
    return;
  unsigned long curr = millis();
  if ((curr - lastDebounceTime) > DEBOUNCE_DELAY) {
    intr_trig_dac = true;
    lastDebounceTime = curr;
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(3000);
  SdFile dir;
  
  analogReadResolution(12);
  pinMode(ADC_PIN, INPUT);
  pinMode(INTERRUPT_LED_PIN, OUTPUT);
  pinMode(ACTIVATE_LED_PIN, OUTPUT);
  pinMode(GEN_NOISE_BUTTON_PIN, INPUT);

  digitalWrite(LED_BUILTIN, led_stat);
  data_logger = (adc_unit*) malloc(sizeof(adc_unit) * ADC_BUFF_SIZE);
  
  if (!sd.begin(SD_CONFIG) | !dir.open(RECORD_ADC_DIR))
    error_led(ERROR_SDCARD_INIT_FAIL);
  read_iir_coef();
  
  delay(1000);
  attachInterrupt(digitalPinToInterrupt(GEN_NOISE_BUTTON_PIN), btn_dac_callback, RISING);

}

void loop() {
  if (intr_trig_dac & !timer_en_dac) {
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

  } else if (!timer_en_dac) {
    digitalWrite(ACTIVATE_LED_PIN, !digitalRead(ACTIVATE_LED_PIN));
    if (timer_en_dac_pr) {
      //write callback function in the pooling staage
    }
  } else if (timer_en_dac) {
    Serial.print(".");
    write_SD2();
  }
  timer_en_dac_pr = timer_en_dac;
  delay(1000);
}
