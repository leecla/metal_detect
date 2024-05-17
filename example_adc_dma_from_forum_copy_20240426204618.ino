#include <driver/i2s.h>

// I2S
#define I2S_SAMPLE_RATE (100000)
#define ADC_INPUT (ADC1_CHANNEL_4) //pin 32
//#define ADC_INPUT (ADC1_CHANNEL_8) // pin 25
#define I2S_DMA_BUF_LEN (1024)
// signal generation stuff: 15KHz 50% duty cycle pwm wave
#define PWM_OUT_SIGGEN_PIN (16)  // pin 16 (RX2) for pwm output to signal generator
#define PWM_CHANNEL (0) // pwm properties
#define PWM_RESOLUTION (8) // resolution (0 to 255)
#define PWM_DUTY_CYCLE (60) // 50% duty cycle (127 is 1/2 of 255)
#define PWM_FREQUENCY (15000) // 15KHz signal generated
#define USER_OUT_PIN (17)

// The 4 high bits are the channel, and the data is inverted
size_t bytes_read;
uint16_t buffer[I2S_DMA_BUF_LEN] = {0};

unsigned long lastTimePrinted;
unsigned long loopTime = 0;

void i2sInit() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
    .sample_rate =  I2S_SAMPLE_RATE,              // The format of the signal using ADC_BUILT_IN
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, // is fixed at 12bit, stereo, MSB
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 2,
    .dma_buf_len = I2S_DMA_BUF_LEN,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_adc_mode(ADC_UNIT_1, ADC_INPUT);
  i2s_adc_enable(I2S_NUM_0);
  adc1_config_channel_atten(ADC_INPUT, ADC_ATTEN_DB_11);
}

void example_disp_buf(uint8_t* buf, int length)
{
//#if EXAMPLE_I2S_BUF_DEBUG
    printf("======\n");
    for (int i = 0; i < length; i++) {
        printf("%02x ", buf[i]);
        if ((i + 1) % 8 == 0) {
            printf("\n");
        }
    }
    printf("======\n");
//#endif
}

void setup() {
  Serial.begin(115200);

  i2sInit();

  // configure LED PWM functionalities
  ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);

  // attach the channel to the GPIO pin to be controlled
  ledcAttachPin(PWM_OUT_SIGGEN_PIN, PWM_CHANNEL);

  // user output pin setup
  pinMode(USER_OUT_PIN, OUTPUT);
}

void loop() {

  digitalWrite(USER_OUT_PIN, LOW); // set user output low

  ledcWrite(PWM_CHANNEL, PWM_DUTY_CYCLE); // start signal generation

  unsigned long startMicros = ESP.getCycleCount();

  i2s_read(I2S_NUM_0, &buffer, sizeof(buffer), &bytes_read, 0);

  unsigned long stopMicros = ESP.getCycleCount();

  loopTime = stopMicros - startMicros;

  if (millis() - lastTimePrinted >= 25) {
    Serial.println("------------------");
    Serial.println(buffer[0] & 0x0FFF);
    //example_disp_buf((uint8_t*) buffer, sizeof(buffer));
    //Serial.println(loopTime);
    lastTimePrinted = millis();
    if ((buffer[0] & 0x0FFF) >= 25){  // detect high amplitude
      Serial.println("---YEA---");
      digitalWrite(USER_OUT_PIN, HIGH);   // set user output high if high amplitude detected
      delay(500);
    }

  }
}