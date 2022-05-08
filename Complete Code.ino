#include <Arduino.h>
#include <LoRaWan-RAK4630.h> //http://librarymanager/All#SX126x
#include <SPI.h>
#include "SparkFunLIS3DH.h" //http://librarymanager/All#SparkFun-LIS3DH
#include <Wire.h>

// RAK4630 supply two LED
#ifndef LED_BUILTIN
#define LED_BUILTIN 35
#endif

#ifndef LED_BUILTIN2
#define LED_BUILTIN2 36
#endif

bool doOTAA = true;   // OTAA is used by default.
#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE 60                      /**< Maximum number of events in the scheduler queue. */
#define LORAWAN_DATERATE DR_0                   /*LoRaMac datarates definition, from DR_0 to DR_5*/
#define LORAWAN_TX_POWER TX_POWER_5             /*LoRaMac tx power definition, from TX_POWER_0 to TX_POWER_15*/
#define JOINREQ_NBTRIALS 3                      /**< Number of trials for the join request. */
#define BG77_POWER_KEY WB_IO1
#define BG77_GPS_ENABLE WB_IO2

DeviceClass_t g_CurrentClass = CLASS_A;         /* class definition*/
LoRaMacRegion_t g_CurrentRegion = LORAMAC_REGION_US915;    /* Region:US915*/
lmh_confirm g_CurrentConfirm = LMH_CONFIRMED_MSG;         /* confirm/unconfirm packet definition*/
uint8_t gAppPort = LORAWAN_APP_PORT;                      /* data port*/

char rspchar[256] = {0}; 
String bg77_rsp = "";
char putMessage[256] = {0};

 unsigned long startTime =0;                    // start time for stop watch
unsigned long elapsedTimeSeconds =0;                  // elapsed time for stop watch
int H,M,S = 0;
bool noLora = false;

/**@brief Structure containing LoRaWan parameters, needed for lmh_init()
*/
static lmh_param_t g_lora_param_init = {LORAWAN_ADR_ON, LORAWAN_DATERATE, LORAWAN_PUBLIC_NETWORK, JOINREQ_NBTRIALS, LORAWAN_TX_POWER, LORAWAN_DUTYCYCLE_OFF};

// Foward declaration
static void lorawan_has_joined_handler(void);
static void lorawan_join_failed_handler(void);
static void lorawan_rx_handler(lmh_app_data_t *app_data);
static void lorawan_confirm_class_handler(DeviceClass_t Class);
static void send_lora_frame(void);

/**@brief Structure containing LoRaWan callback functions, needed for lmh_init()
*/
static lmh_callback_t g_lora_callbacks = {BoardGetBatteryLevel, BoardGetUniqueId, BoardGetRandomSeed,
                                        lorawan_rx_handler, lorawan_has_joined_handler, lorawan_confirm_class_handler, lorawan_join_failed_handler
                                       };
//OTAA keys !!!! KEYS ARE MSB !!!!
//AD4630
//uint8_t nodeDeviceEUI[8] = {0x60, 0x81, 0xF9, 0x1A, 0x00, 0xD8, 0x9D, 0x17};//actual working one
//uint8_t nodeAppKey[16] = {0xA7, 0x6E, 0x8F, 0x9B, 0xC9, 0x77, 0xEA, 0x76, 0x11, 0x02, 0xB8, 0x15, 0xD0, 0x94, 0x21, 0x75};//actual working one
//uint8_t nodeAppEUI[8] = {0x60, 0x81, 0xF9, 0x2E, 0xEA, 0xBC, 0x1A, 0x0B}; //actual working one
uint8_t nodeDeviceEUI[8] = {0xFF, 0xFF, 0xFF, 0x1A, 0x00, 0xD8, 0x9D, 0x17}; //intentionally broken!
uint8_t nodeAppEUI[8] = {0x60, 0x80, 0xF9, 0x2E, 0xEA, 0xBC, 0x1A, 0x0B}; //intentionally broken
uint8_t nodeAppKey[16] = {0xFF, 0xFF, 0xFF, 0x9B, 0xC9, 0x77, 0xEA, 0x76, 0x11, 0x02, 0xB8, 0x15, 0xD0, 0x94, 0x21, 0x75}; //intentionally broken!

// ABP keys
uint32_t nodeDevAddr = 0x260116F8;
uint8_t nodeNwsKey[16] = {0x7E, 0xAC, 0xE2, 0x55, 0xB8, 0xA5, 0xE2, 0x69, 0x91, 0x51, 0x96, 0x06, 0x47, 0x56, 0x9D, 0x23};
uint8_t nodeAppsKey[16] = {0xFB, 0xAC, 0xB6, 0x47, 0xF3, 0x58, 0x45, 0xC7, 0x50, 0x7D, 0xBF, 0x16, 0x8B, 0xA8, 0xC1, 0x7C};

// Private defination
#define LORAWAN_APP_DATA_BUFF_SIZE 64                     /**< buffer size of the data to be transmitted. */
#define LORAWAN_APP_INTERVAL 20000                        /**< Defines for user timer, the application data transmission interval. 20s, value in [ms]. */
static uint8_t m_lora_app_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];            //< Lora user application data buffer.
static lmh_app_data_t m_lora_app_data = {m_lora_app_data_buffer, 0, 0, 0, 0}; //< Lora user application data structure.


static uint32_t count = 0;
static uint32_t count_fail = 0;
float battery =0.0;
bool hasSent = false;
uint8_t crash_status = 0;
float x,y,z = 0.00;
LIS3DH SensorTwo(I2C_MODE, 0x18);
char gnssChar[10][64] = {{}}; //global
float nFloat,wFloat = 0.0f; //global

  char batteryLife[256] = {0};

float getBatteryLife(){

  memset(batteryLife,0,64);
  bg77_at("AT+CBC",2000);
  bg77_at("AT+CBC",2000);
  char *p;
  char * res[256] = {0};
  p = strtok(rspchar, ",");
  p = strtok(NULL, ",");
  p = strtok(NULL, ",");
  if(p){
    strncpy(batteryLife, p, sizeof(p));
  }
  float f = atof(batteryLife);
  f = f / 1000.0;
  memset(batteryLife,0,64);
  return f;  
}

  void send_HTTP_PUT(char *URL, char *message){
//Only works with firmware version BG77LAR02A04_01.008.01.008 or newer. use AT+QGMR to find firmware version and use Qflash to update.
//can 100% be optimized, especially regarding these 4 char arrays. Look at how sample functions pass data to the bg77. Time delays may also likely be decreased.
  char urlAT[64] = "";
  char putAT[64] = "";
  char urlIN[256] = "";
  char msgIN[512] =  "";
  sprintf(urlIN, "%s", URL);
  sprintf(msgIN, "%s", message);
  sprintf(urlAT, "AT+QHTTPURL=%d,80", strlen(URL));
  sprintf(putAT, "AT+QHTTPPUT=%d,80,80", strlen(message));

    bg77_at(urlAT, 2000);
    //delay(3000);
    
    bg77_at(urlIN, 2000);
    //delay(3000);
    
    bg77_at(putAT, 2000);
    delay(3000);
    bg77_at(msgIN, 2000);
    //delay(3000);
    bg77_at("AT+QHTTPREAD=200", 4000);
    //delay(3000);
  
  }


void bg77_shutdown(){ 
  bg77_at("AT+QCFG=\"fast/poweroff\",63,1", 2000);
  bg77_at("AT+QIDEACT=1", 2000);
  bg77_at("AT+QPOWD=2",2000);
}

void bg77_at(char *at, uint16_t timeout)
{
  char tmp[256] = {0};
  int len = strlen(at);
  strncpy(tmp, at, len);
  uint16_t t = timeout;
  tmp[len] = '\r';
  Serial1.write(tmp);
  delay(10);
  while (t--)
  {
    if (Serial1.available())
    {
      bg77_rsp += char(Serial1.read());
    }
    delay(1);
  } 
//  while(Serial1.available()){
//    bg77_rsp += char(Serial1.read());
//    delay(1);
//  }
  Serial.println(bg77_rsp);
  strncpy(rspchar, bg77_rsp.c_str(), bg77_rsp.length());
  //Serial.printf("Response char array: %s", rspchar);
  //memset(rspchar,0,256);
  bg77_rsp = "";
}

void setup_bg77(){
    //BG77 init , Check if the modem is already awake
  time_t timeout = millis();
  bool moduleSleeps = true;
   Serial1.begin(115200);
  delay(1000);
  pinMode(BG77_GPS_ENABLE, OUTPUT);
  digitalWrite(BG77_GPS_ENABLE, 1);
  Serial1.println("ATI");
  //BG77 init
  while ((millis() - timeout) < 6000)
  {
    if (Serial1.available())
    {
      String result = Serial1.readString();
      Serial.println("Modem response after start:");
      Serial.println(result);
      moduleSleeps = false;
    }
  }
  if (moduleSleeps)
  {
    // Module slept, wake it up
    pinMode(BG77_POWER_KEY, OUTPUT);
    digitalWrite(BG77_POWER_KEY, 0);
    delay(1000);
    digitalWrite(BG77_POWER_KEY, 1);
    delay(2000);
    digitalWrite(BG77_POWER_KEY, 0);
    delay(1000);
  }
  Serial.println("BG77 power up!");
  bg77_at("AT+QGMR", 2000);
  //active and join to the net, this part may depend on some information of your operator.
  bg77_at("AT+CFUN=1,0", 500);
  //delay(2000);
  bg77_at("AT+CPIN?", 500);
  
    
  //delay(2000);
  //bg77_at("AT+COPS=?", 2000);

  //bg77_at("AT+QNWINFO", 500);
  //delay(2000);
  //bg77_at("AT+QCSQ", 500);
 // delay(2000);
  //bg77_at("AT+CSQ", 500);
  //delay(2000);
  bg77_at("AT+QIDEACT=1", 3000);
  bg77_at("AT+QICSGP=1,1,\"hologram\",\"\",\"\",0",2000);
  //delay(2000);
  bg77_at("AT+QIACT=1", 3000);
    
//  while(bg77_rsp.indexOf('OK') < 0 ){
//      //bg77_at("AT+QIACT=1", 3000);
//      Serial.printf("Failed to activate PDP context, trying again");
//      Serial.printf("%s\n", bg77_rsp.indexOf('OK'));
//      bg77_at("AT+QIDEACT=1", 3000);
//      bg77_at("AT+QIACT=1", 3000);
//
//      delay(5000);
//  }

  //open tcp link with Hologram server
  bg77_at("AT+QIOPEN=1,0,\"TCP\",\"cloudsocket.hologram.io\",9999,0,1", 5000);
  
  delay(2000);
  
}
 void splitTime(){
    char rawtime[16] = {};
    strncpy(rawtime,gnssChar[0],16);
    printf("time: %s\n", rawtime);
    char tmp[16] = {};
  
    sprintf(tmp,"%c%c", rawtime[1],rawtime[2]);
    H = atoi(tmp);
    memset(tmp, 0, 16);
    
    sprintf(tmp,"%c%c", rawtime[3],rawtime[4]);
    M = atoi(tmp);
    memset(tmp, 0, 16);
    
    sprintf(tmp,"%c%c", rawtime[5],rawtime[6]);
    S = atoi(tmp);
    memset(tmp, 0, 16);
    Serial.printf("H:%d, M: %d, S:%d\n", H,M,S);
  }

int currTime = 0;
void getDateTimeGPS(){
  int gpsAttempts = 0;
    bg77_at("AT+QGPS=1",2000);
  
    char nCoord[32] = {};
    char wCoord[32] = {};
    char rspCode[64] = {};
    for(int k = 0; k < 10; k++){
        memset(gnssChar[k],0,64);
    }
    //memset(rspchar, 0, 256);
    Serial.println("Waiting for GPS fix");
//        char *p;
//        bg77_at("AT+QGPSLOC=2",5000);
//        p = strtok(rspchar, ":");
//        
//    strncpy(rspCode, p, sizeof(p));
//    Serial.printf("rspCode: %s",rspCode);
//    while((strstr(rspCode, "CME") != NULL) && (gpsAttempts < 50)){
//        memset(rspchar, 0, 256);
//        bg77_at("AT+QGPSLOC=2",5000);
//        bg77_at("AT+QGPSLOC=2",5000);
//        p = strtok(rspchar, ":");
//        gpsAttempts++;
//        delay(2000);
//    }
    
  delay(24000);


        bg77_at("AT+QGPSLOC=2",5000);
        bg77_at("AT+QGPSLOC=2",5000);
//  for(int l = 0; l < 10; l++){
//    
//    memset(rspchar, 0, 256);
//    bg77_at("AT+QGPSLOC?",8000);
////    Serial.println(rspchar);
//    delay(1000);
//  }
  bg77_at("AT+QGPSLOC2",5000);
  
    char *p;  //migiht need these
        p = strtok(rspchar, ":");
    strncpy(rspCode, p, sizeof(p));
        p = strtok(NULL, ",");
    strncpy(gnssChar[0], p, sizeof(p)+10);
  
    startTime = 0;
    startTime = millis(); //needs to reset every night
    splitTime();
  for(int i = 1; i < 10; i++){
      if(p){
        p = strtok(NULL, ",");
        strncpy(gnssChar[i], p, sizeof(p)+10);
        }
    }
    
  for(int j = 0; j < 10; j++){
    Serial.printf("gnss char arr %d: %s\n",j,gnssChar[j]);
  }

   currTime = atoi(gnssChar[0]);
   Serial.printf("Time(int): %d\n", currTime);
   
  strncpy(nCoord, gnssChar[1], strlen(gnssChar[1]));
  strncpy(wCoord, gnssChar[2], strlen(gnssChar[2]));
  
  nCoord[strlen(nCoord)-1] = NULL;
  wCoord[strlen(wCoord)-1] = NULL;

   nFloat = atof(nCoord);
   wFloat = atof(wCoord);
//
//   nFloat = nFloat / 100.0;
//   wFloat = wFloat / 100.0;
   bg77_at("AT+QGPSEND",2000);
  }

void lis3dh_write_data(float &x, float &y, float &z)
{
  // read the sensor value
  uint8_t cnt = 0;
  x = SensorTwo.readFloatAccelX();
  y = SensorTwo.readFloatAccelY();
  z = SensorTwo.readFloatAccelZ();
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

   if (SensorTwo.begin() != 0)
  {
    Serial.println("Problem starting the sensor at 0x18.");
  }
  else
  {
    Serial.println("Sensor at 0x18 started.");
  }
  
  
  
  // Initialize LoRa chip.
  lora_rak4630_init();
  // Initialize Serial for debug output
  time_t timeout = millis();
  Serial.begin(115200);
  while (!Serial)
  {
    if ((millis() - timeout) < 5000)
    {
      delay(100);
    }
    else
    {
      break;
    }
  }
  Serial.println("=====================================");
  Serial.println("Welcome to RAK4630 LoRaWan!!!");
  if (doOTAA)
  {
    Serial.println("Type: OTAA");
  }
  else
  {
    Serial.println("Type: ABP");
  }
  switch (g_CurrentRegion)
  {
    case LORAMAC_REGION_AS923:
      Serial.println("Region: AS923");
      break;
    case LORAMAC_REGION_AU915:
      Serial.println("Region: AU915");
      break;
    case LORAMAC_REGION_CN470:
      Serial.println("Region: CN470");
      break;
    case LORAMAC_REGION_EU433:
      Serial.println("Region: EU433");
      break;
    case LORAMAC_REGION_IN865:
      Serial.println("Region: IN865");
      break;
    case LORAMAC_REGION_EU868:
      Serial.println("Region: EU868");
      break;
    case LORAMAC_REGION_KR920:
      Serial.println("Region: KR920");
      break;
    case LORAMAC_REGION_US915:
      Serial.println("Region: US915");
      break;
  }
  Serial.println("=====================================");
  
  // Setup the EUIs and Keys
  if (doOTAA)
  {
    lmh_setDevEui(nodeDeviceEUI);
    lmh_setAppEui(nodeAppEUI);
    lmh_setAppKey(nodeAppKey);
  }
  else
  {
    lmh_setNwkSKey(nodeNwsKey);
    lmh_setAppSKey(nodeAppsKey);
    lmh_setDevAddr(nodeDevAddr);
  }
  // Initialize LoRaWan
  uint32_t err_code;
  err_code = lmh_init(&g_lora_callbacks, g_lora_param_init, doOTAA, g_CurrentClass, g_CurrentRegion);
  if (err_code != 0)
  {
    Serial.printf("lmh_init failed - %d\n", err_code);
    return;
  }
  // Start Join procedure
  lmh_join();
  delay(10000);
  setup_bg77();
  delay(10);
  battery = getBatteryLife();
  getDateTimeGPS();
        sprintf(putMessage, "{\"\\\"Battery\\\"\":\"\\\"%f\\\"\",\"\\\"GPS\\\"\":\"\\\"%f,%f\\\"\",\"\\\"Time\\\"\":\"\\\"%dH:%dM:%dS UTC\\\"\",\"\\\"ID\\\"\":\"\\\"ZJS4630\\\"\",\"\\\"Status\\\"\":\"\\\"Normal.\\\"\"}", battery,nFloat,wFloat,H,M,S);
    send_HTTP_PUT("https://wrecks-24e46-default-rtdb.firebaseio.com/%22Cellular6%22.json", putMessage); //USE THIS ONE
  //bg77_shutdown();
}

void loop()
{
  //check the imu threshold{}
  lis3dh_write_data(x, y, z);
   if(sqrt(sq(x) + sq(y) + sq(z)) > 1.5){
    crash_status=1;
    //lmh_join();

    Serial.println("================Crash Detected================ ");
    Serial.print("Magnitude: ");
    Serial.println(sqrt(sq(x) + sq(y) + sq(z)));
    
    if(noLora){
      Serial.println("No Lora Coverage! Using 4G LTE");
        memset(putMessage,0,256);
      sprintf(putMessage, "{\"\\\"Battery\\\"\":\"\\\"%f\\\"\",\"\\\"GPS\\\"\":\"\\\"%f,%f\\\"\",\"\\\"Time\\\"\":\"\\\"%dH:%dM:%dS UTC\\\"\",\"\\\"ID\\\"\":\"\\\"ZJS4630\\\"\",\"\\\"Status\\\"\":\"\\\"Crash Detected!\\\"\"}", battery,nFloat,wFloat,H,M,S);
    send_HTTP_PUT("https://wrecks-24e46-default-rtdb.firebaseio.com/%22Cellular6%22.json", putMessage); //USE THIS ONE
    }else{
      Serial.println("Sending frame now...");
          send_lora_frame();
    }

    // Start Join procedure
    
    delay(2000);
   }
   
   delay(50);
}

void lorawan_has_joined_handler(void)
{
  Serial.println("OTAA Mode, Network Joined!");
  lmh_error_status ret = lmh_class_request(g_CurrentClass);
}

static void lorawan_join_failed_handler(void)
{
  Serial.println("OTAA join failed!");
  Serial.println("Check your EUI's and Keys's!");
  Serial.println("Check if a Gateway is in range!");
  noLora = true;
}

void lorawan_rx_handler(lmh_app_data_t *app_data)
{
  Serial.printf("LoRa Packet received on port %d, size:%d, rssi:%d, snr:%d, data:%s\n",
          app_data->port, app_data->buffsize, app_data->rssi, app_data->snr, app_data->buffer);
}

void lorawan_confirm_class_handler(DeviceClass_t Class)
{
  Serial.printf("switch to class %c done\n", "ABC"[Class]);
  // Informs the server that switch has occurred ASAP
  m_lora_app_data.buffsize = 0;
  m_lora_app_data.port = gAppPort;
  lmh_send(&m_lora_app_data, g_CurrentConfirm);
}

void send_lora_frame(void)
{
  if (lmh_join_status_get() != LMH_SET)
  {
    //Not joined, try again later
    return;
  }

  uint32_t i = 0;
  memset(m_lora_app_data.buffer, 0, LORAWAN_APP_DATA_BUFF_SIZE);
  m_lora_app_data.port = gAppPort;
  m_lora_app_data.buffer[i++] = 0x09;    
  m_lora_app_data.buffer[i++] = (battery/4.8)*100; //& 0xFF000000) >> 24;
  m_lora_app_data.buffer[i++] = crash_status;//(battery & 0x00FF0000) >> 16;
  m_lora_app_data.buffer[i++] = (((int)nFloat* 100000) & 0xFF000000) >> 24;
  m_lora_app_data.buffer[i++] = (((int)nFloat* 100000) & 0x00FF0000) >> 16;
  m_lora_app_data.buffer[i++] = (((int)nFloat* 100000) & 0x0000FF00) >> 8;
  m_lora_app_data.buffer[i++] =  ((int)nFloat* 100000) & 0x000000FF;
  m_lora_app_data.buffer[i++] = (((int)wFloat* 100000) & 0xFF000000) >> 24;
  m_lora_app_data.buffer[i++] = (((int)wFloat* 100000) & 0x00FF0000) >> 16;
  m_lora_app_data.buffer[i++] = (((int)wFloat* 100000) & 0x0000FF00) >> 8;
  m_lora_app_data.buffer[i++] =  ((int)wFloat* 100000) & 0x000000FF;
  
  //m_lora_app_data.buffer[i++] = (battery & 0x0000FF00) >> 8;
  //m_lora_app_data.buffer[i++] =  battery & 0x000000FF;
  
  m_lora_app_data.buffsize = i;
  lmh_error_status error = lmh_send(&m_lora_app_data, g_CurrentConfirm);
  if (error == LMH_SUCCESS)
  {
    count++;
    Serial.printf("lmh_send ok count %d\n", count);
  }
  else
  {
    count_fail++;
    Serial.printf("lmh_send fail count %d\n", count_fail);
  }
}
