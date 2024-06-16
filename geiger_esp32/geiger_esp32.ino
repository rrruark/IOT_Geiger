//2024 Robert Ruark

//#include "arduino.h"
#include "driver/pcnt.h"                                                
#include "Tone32.h"

#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;

#define BUZZER_PIN 5
#define BUZZER_CHANNEL 0
#define PCNT_H_LIM_VAL        overflow                                  
#define FREQ_PIN              22
#define CPS_TO_MR             26
#define mem_depth             10

float           alpha         = 0.025;
bool            flag          = true;
bool            detection     = false;                                   
uint32_t        overflow      = 20000;                                  
//int16_t         pulses        = 0;    
float           CPM           = 20.0;  
float           CPM_last;
float           uSv           = 0.0;   
float           beta          =0.0;                        
uint32_t        overflow_cnt  = 0;                                        
volatile double pulses     = 0;
uint16_t result = 0;
int             pulse_cnt[mem_depth];
int             counter =0;

uint64_t chipid;
//void pcnt_get_counter(void *p); 
//void pcnt_event_handler(void *arg);

esp_timer_create_args_t timer_args;                                       // Create an esp_timer instance
esp_timer_handle_t timer_handle;                                          // Create an single timer
portMUX_TYPE timer_mux = portMUX_INITIALIZER_UNLOCKED;

pcnt_config_t pcnt_config = {
  .pulse_gpio_num    = FREQ_PIN,
  .ctrl_gpio_num     = -1,
  .lctrl_mode        = PCNT_MODE_KEEP,
  .hctrl_mode        = PCNT_MODE_KEEP,
  .pos_mode          = PCNT_COUNT_INC,
  .neg_mode          = PCNT_COUNT_INC,
  .counter_h_lim     = 20000,
  .counter_l_lim     = 0,
  .unit              = PCNT_UNIT_0, 
  .channel           = PCNT_CHANNEL_0
  };


void IRAM_ATTR pcnt_event_handler(void *arg)                               // Counting overflow pulses
{
  portENTER_CRITICAL_ISR(&timer_mux);                                      // disable interrupt
  overflow_cnt++;                                                          // increment Overflow counter
  //PCNT.int_clr.val = BIT(PCNT_UNIT_0);                                     // Clear Pulse Counter interrupt bit
  portEXIT_CRITICAL_ISR(&timer_mux);                                       // enable interrupt
}                                        
  
//----------------------------------------------------------------------------------------
void IRAM_ATTR isr_pulse() 
{
  detection = true;
}
//---------------------------------------------------------------------------------

void setup()
{
  Serial.begin(115200);  
  pinMode(13, OUTPUT);                                                
  pinMode(2, OUTPUT); 
  ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL); 
  //ledcSetup(BUZZER_PIN, frequency, bits); 
  pcnt_init();                                                             // Initialize
  attachInterrupt(FREQ_PIN, isr_pulse, FALLING);

  //chip id
  chipid = ESP.getEfuseMac();
  uint16_t chip = (uint16_t)(chipid >> 32);
  Serial.print(String(chip));
  String btname = "Geiger-" + String(chip);

  // Enable bluetooth
  SerialBT.begin(btname); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
}

//----------------------------------------------------------------------------------

void pcnt_init(void)                                                     
{  
  pinMode(FREQ_PIN,INPUT);

  pcnt_unit_config(&pcnt_config);
  
  pcnt_isr_register(pcnt_event_handler, NULL, 0, NULL);                   // Setup Register ISR handler
  pcnt_intr_enable(PCNT_UNIT_0);  

  pcnt_set_filter_value(PCNT_UNIT_0, 1000);
  pcnt_filter_enable(PCNT_UNIT_0); 

  pcnt_counter_pause(PCNT_UNIT_0);                                        // Pause PCNT unit
  pcnt_counter_clear(PCNT_UNIT_0);                                        // Clear PCNT unit
  
  pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_H_LIM);                         // Enable event to watch - max count
  pcnt_counter_resume(PCNT_UNIT_0);                                       // Resume PCNT unit - starts count

  timer_args.callback = pcnt_get_counter;
  timer_args.arg      = NULL;
  timer_args.name     = "one shot timer";

  if(esp_timer_create(&timer_args, &timer_handle) != ESP_OK) 
  {
    ESP_LOGE(TAG, "timer create");
  }

  timer_args.callback = pcnt_get_counter;                                 // Set esp-timer argument
  esp_timer_create(&timer_args, &timer_handle);                           // Create esp-timer instance
}
//----------------------------------------------------------------------------------

void pcnt_get_counter(void *p) 
{                                                  // Read Pulse Counter
  pcnt_counter_pause(PCNT_UNIT_0);
  pcnt_get_counter_value(PCNT_UNIT_0, (int16_t*) &result); 
  flag = true;
}
//---------------------------------------------------------------------------------

void loop() 
{
  if (flag == true)
  {
    flag = false;
    pulses =  (result + (overflow_cnt*20000))/2;

    //Speeds up settling in high radiation environment
    beta = alpha+0.005*(float)pulses;
    
    //Speeds up settling when transitioning to a low radiation environemnt

    pulse_cnt[counter]=pulses;
    int total_pulses=0;

    for(int i=0; i<mem_depth; i++)
    {
      total_pulses+=pulse_cnt[i];
    }
    counter++;
    if(counter==mem_depth) counter=0;    
    
    if(((float)total_pulses+1)*2<CPM/(60/mem_depth))
    {
      beta = alpha+0.1*CPM/60/(total_pulses+1);
    }
    if(beta>0.9) beta=0.9;

    

    
    CPM = CPM*(1-beta)+ (float)pulses*60*beta;
    
    uSv = CPM/60/CPS_TO_MR/100*1000; //only valid for sbm-20

    
    overflow_cnt = 0; 
    pcnt_counter_clear(PCNT_UNIT_0); 
    pcnt_counter_resume(PCNT_UNIT_0); 
    overflow_cnt = 0;    
//    Serial.print((float)total_pulses/mem_depth*60);
//    Serial.print(",");
    char message[128] = "";
    //size_t len = sprintf(message, "%3.0f counts, %5.3f CPM, %3.3f uSv/h\n", pulses, CPM, uSv);
    size_t len = sprintf(message, "%3.0f counts, %5.3f CPM\n", pulses, CPM);
    Serial.print(String(message));
    SerialBT.write((const uint8_t*) message, len);

    //Serial.print(pulses);
    //Serial.print(" counts");
    //Serial.print(",");
    //Serial.print(CPM);
    //Serial.print(" CPM");
    //Serial.print(",");
    //Serial.print(uSv);
    //Serial.println(" uSv/h");
    pcnt_counter_clear(PCNT_UNIT_0);
    esp_timer_start_once(timer_handle, 1000000);                    // Initialize High resolution timer (1 sec)
  }
    // Put your function here, if you want
  if(detection)
  {
    digitalWrite(13,1);
    tone(BUZZER_PIN, NOTE_D7, 2, BUZZER_CHANNEL);
    noTone(BUZZER_PIN, BUZZER_CHANNEL);
    detection = false;
    digitalWrite(13,0);
  }
}
