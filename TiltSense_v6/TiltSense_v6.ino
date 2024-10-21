#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include <MsTimer2.h>
//#include <Watchdog.h>
#include <LowPower.h>
#include <math.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <Flash.h>

#define MODEM_MAX_RETRIES 5  // Number of retries before giving up
#define MODEM_DEFAULT_TIMEOUT 3000  // Default timeout of 3 seconds

#define MMA8451_REG_CTRL_REG3 0x2C
#define MMA8451_REG_CTRL_REG5 0x2E
#define RANGE 4096.0
const int FIRMWARE_VERSION = 33; //ascii!!
const int SKIP_MODEM_SETUP = 1; //FOR DEBUG
const int modemPwrDwn = 7;
const int MODEM_TIMEOUT = 30; //20 seconds before connection retried
const int wakeUpPin = 2;
const int arraySize = 64;
int phone_home_threshold = 2167; //this is how many events it takes before sending 39.88s is one fill -- 90 per hour -- 1083 per 12 hours --216v`7 per 24
const int MAX_ALARMS_PER_CHECKIN = 3;
const int SEND_ATTEMPTS_BEFORE_RESTART = 3;


// RAM buffer needed by the Flash library. Use flash[] to access the buffer.
uint8_t ram_buffer[SPM_PAGESIZE];

// Allocate two flash pages for storing data. Use PROGMEM1 to allocate space above 64kiB
#define NUMBER_OF_PAGES 2
const uint8_t flash_space[SPM_PAGESIZE * NUMBER_OF_PAGES] __attribute__((aligned(SPM_PAGESIZE))) PROGMEM = {};

// Flash constructor
Flash flash(flash_space, sizeof(flash_space), ram_buffer, sizeof(ram_buffer));

String inputString ="";
String MQTT_topic, MQTT_message;
String IMSI;
String THRESHOLD;
SoftwareSerial mySerial(9, 10); // RX, TX
Adafruit_MMA8451 mma = Adafruit_MMA8451();
//Watchdog watchdog;
uint8_t ZERO_FLAG = 0;
uint8_t PHONE_HOME_FLAG = 0; //daily is 0, twice daily is 1, hourly is 2;
uint8_t ALARM_NOW = 0;
uint8_t ALARMS_SENT = 0;

uint8_t HTTP_GET_NOW = 0;
uint8_t led_state = 0;
uint8_t MQTT_RETRIES = 0;
uint8_t CATASTROPHIC_ALARM_SENT = 0;
uint8_t LOG_FOR_TEST = 0;

int retry_counter =0;
int retry_attempts=0;
int sim_reset_attempts=0;
uint8_t SLEEP_BLOCKED = 1;
uint8_t dataReady = 0;
int counter=0;

int phone_home_counter = 1; //how many buffer fills before sending a packet home

bool stringComplete = false;  // whether the string is complete
int IMSI_READ = 0;

int16_t x[arraySize];
int16_t y[arraySize];
int16_t z[arraySize];

float xmin = 500, ymin = 500, zmin = 500;
float xmax = -500, ymax = -500, zmax = -500;
float x_zero = 0, y_zero = 0, z_zero = 0;
float overall_tilt;
float battery_voltage;

float x_theta_mean =0, y_theta_mean =0, z_theta_mean =0;


float meanX = 0, meanY = 0, meanZ = 0;
float M2X = 0, M2Y = 0, M2Z = 0;
unsigned long nX = 0, nY = 0, nZ = 0;
float stdDevX, stdDevY, stdDevZ = 0;

long x_total, y_total, z_total = 0;

float fltAlarmThreshold = 10.0;


union FloatToBytes {
  float f;
  byte b[4];
};

FloatToBytes converter;


float fltSend[15];
void setup() {
  // put your setup code here, to run once:
  MCUSR = 0;
  wdt_disable();
  Serial.begin(38400);
  mySerial.begin(115200);

  
  if(!flash.check_writable())
  {
    Serial.println(F("Incompatible or no Urboot compatible bootloader present! Please burn correct bootloader"));
    while(1);
  }

  mySerial.println(F("TiltSense v3.0 Booted")); 

  pinMode(modemPwrDwn, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(13, OUTPUT);

  digitalWrite(13, LOW);
  delay(1000);
  digitalWrite(8, LOW);
  digitalWrite(modemPwrDwn, LOW);
  mySerial.println(F("Modem Off")); 
  delay(5000);
  
  digitalWrite(8, HIGH);
  digitalWrite(modemPwrDwn, HIGH);
  mySerial.println(F("Modem On")); 
  delay(2000);  
  digitalWrite(8, LOW);
  

  // Fetch flash page 1, where we may have a flag
  flash.fetch_page(1);
  //read_data();
  check_sim_reset();
  // Check if our flag is present
  if(flash[5] == 'X')
  {
    mySerial.println(F("MESSAGE NOT SENT SUCCESSFULLY"));
    read_data();
    mySerial.print(F("ALARMS NOW:"));
    mySerial.println(ALARM_NOW);
    if (ALARM_NOW == 0){
      MQTT_topic ="check_in";
    } else{
      MQTT_topic ="alarm";
    }
  }else
  {
    if((flash[5] == 'Y')||(SKIP_MODEM_SETUP)){
      mySerial.println(F("LAST MESSAGE WAS SENT SUCCESSFULLY"));
    } else {
      mySerial.println(flash[5]);
      mySerial.println(F("FIRST RUN - SETTING UP MODEM..."));
      Serial.println(F("ATZ0"));
      delay(5000);
      Serial.println(F("AT"));
      delay(3000);
      Serial.println(F("ATE1"));
      delay(3000);
      Serial.println(F("AT+CGMM"));
      delay(3000);
      Serial.println(F("AT+CPIN?"));
      delay(3000);
      Serial.println(F("AT+CSQ"));
      delay(3000);
      Serial.println(F("AT+CGREG?"));
      delay(3000);
      // Serial.println(F("AT+CGREG=0"));
      // delay(3000);
      Serial.println(F("AT+CGATT?"));
      delay(3000);
      Serial.println(F("AT+CGACT?"));
      delay(3000);
      Serial.println(F("AT+COPS?"));
      delay(3000);
      for (int i =0; i<180; i++){
        mySerial.println(i);
        delay(1000);
      }
      flash.clear_buffer();
      flash[5] = 'Y'; //x means not sent successfully
      flash.write_page(1);
    }

    mySerial.print(F("Flash page size for this chip: "));
    mySerial.print(SPM_PAGESIZE);
    mySerial.print(F(" bytes\nTotal assigned flash space: "));
    mySerial.print(NUMBER_OF_PAGES * SPM_PAGESIZE);
    mySerial.println(F(" bytes"));

    MQTT_topic ="status";
    status_message();
  } 

  //watchdog.enable(Watchdog::TIMEOUT_8S);
  wdt_enable(WDTO_8S);
  //while(1);

  connectModem();
  
  

  
}

void status_message(){
  String strPhoneHome;
  
  if (PHONE_HOME_FLAG == 0){
    strPhoneHome = "30";
  } else if (PHONE_HOME_FLAG == 1){
    strPhoneHome = "31";
  } else if (PHONE_HOME_FLAG == 2){
    strPhoneHome = "32";
  }
  MQTT_message = strPhoneHome+THRESHOLD+FIRMWARE_VERSION;
  mySerial.println(F("STATUS MESSAGE:"));
  mySerial.println(MQTT_message);
}

void check_sim_reset(){
  uint8_t buffer_address = 0; // Buffer address to start from
  buffer_address += sizeof(fltSend); 
  buffer_address += sizeof(ALARM_NOW); 
  buffer_address += sizeof(ALARMS_SENT);
  buffer_address += sizeof(fltAlarmThreshold); 
  buffer_address += sizeof(sim_reset_attempts);
  flash.get(buffer_address, sim_reset_attempts);
}

void write_data(){
  mySerial.println("WRITING HISTORY TO FLASH");
  uint8_t buffer_address = 0;

  // First, make sure there are no content in out buffer
  flash.clear_buffer();

  flash.put(buffer_address, fltSend);
  buffer_address += sizeof(fltSend);
  flash.put(buffer_address, ALARM_NOW);
  buffer_address += sizeof(ALARM_NOW);
  flash.put(buffer_address, ALARMS_SENT);
  buffer_address += sizeof(ALARMS_SENT);
  flash.put(buffer_address, fltAlarmThreshold);
  buffer_address += sizeof(fltAlarmThreshold);
  flash.put(buffer_address, CATASTROPHIC_ALARM_SENT);
  buffer_address += sizeof(CATASTROPHIC_ALARM_SENT);
  flash.put(buffer_address, sim_reset_attempts);
  
  // Write buffer to the first allocated flash page (page 0)
  flash.write_page(0);

  // Now let's set a flag on another flash page to indicate that the flash memory contains content
  // Here we're treating the object as an array
  flash.clear_buffer();
  flash[5] = 'X'; //x means not sent successfully
  flash.write_page(1);

  mySerial.print(F("ALARM NOW:"));
  mySerial.println(ALARM_NOW);
  mySerial.print(F("ALARMS SENT:"));
  mySerial.println(ALARMS_SENT);
  mySerial.print(F("THRESHOLD STRING:"));
  mySerial.println(THRESHOLD);
  mySerial.print(F("THRESHOLD FLOAT:"));
  mySerial.println(fltAlarmThreshold);
  mySerial.print(F("CATASTROPHIC_ALARM_SENT:"));
  mySerial.println(CATASTROPHIC_ALARM_SENT);
  mySerial.print(F("sim_reset_attempts:"));
  mySerial.println(sim_reset_attempts);
  mySerial.println(F("Written custom data type!\nReset your board to view the contents!\n"));
}

void read_data(){
  mySerial.println("READING HISTORY FROM FLASH");
  // Fetch first flash page
  flash.fetch_page(0);

  uint8_t buffer_address = 0; // Buffer address to start from
  flash.get(buffer_address, fltSend);
  buffer_address += sizeof(fltSend);
  flash.get(buffer_address, ALARM_NOW);
  buffer_address += sizeof(ALARM_NOW);
  flash.get(buffer_address, ALARMS_SENT);
  buffer_address += sizeof(ALARMS_SENT);
  flash.get(buffer_address, fltAlarmThreshold);
  buffer_address += sizeof(fltAlarmThreshold);
  flash.get(buffer_address, CATASTROPHIC_ALARM_SENT);
  buffer_address += sizeof(CATASTROPHIC_ALARM_SENT);
  flash.get(buffer_address, sim_reset_attempts);

  mySerial.println(F("Read custom object from flash: "));
  mySerial.print(F("ALARM NOW:"));
  mySerial.println(ALARM_NOW);
  mySerial.print(F("ALARMS SENT:"));
  mySerial.println(ALARMS_SENT);
  mySerial.print(F("THRESHOLD STRING:"));
  mySerial.println(THRESHOLD);
  mySerial.print(F("THRESHOLD FLOAT:"));
  mySerial.println(fltAlarmThreshold);
  mySerial.print(F("CATASTROPHIC_ALARM_SENT:"));
  mySerial.println(CATASTROPHIC_ALARM_SENT);
  mySerial.print(F("sim_reset_attempts:"));
  mySerial.println(sim_reset_attempts);


  mySerial.print(F("THRESHOLD FLOAT:"));
  mySerial.println(fltAlarmThreshold);
      
}

void wakeUp(){
    // Just a handler for the pin interrupt.
    dataReady = 1 ;
}


void setupTilt(){
   if (! mma.begin()) {
    mySerial.println(F("Couldnt start"));
  }
  pinMode(wakeUpPin, INPUT);   //attach interrupt to pin
  attachInterrupt(0, wakeUp, HIGH);
  mma.writeRegister8(MMA8451_REG_CTRL_REG1, 0b00000000);
  delay(100);
  mma.writeRegister8(MMA8451_REG_CTRL_REG2, 0b00000011); //set to low power oversampling
  delay(100);
  mma.writeRegister8(MMA8451_REG_CTRL_REG3, 0b00000010); //set interrupt to open drain & high polarity
  delay(100);
  mma.writeRegister8(MMA8451_REG_CTRL_REG4, 0b00000001); //enable data ready interrupt
  delay(100);
  mma.writeRegister8(MMA8451_REG_CTRL_REG5, 0b00000011); //set DRDY interrupt to route to INT1
  delay(100);
  mma.writeRegister8(0x0E, 0b00000000); //set to 2g range
  mma.writeRegister8(MMA8451_REG_CTRL_REG1, 0b00111001); //set data rate and reenable
  delay(1000);
}

void connectModem() {
    wdt_reset();
    mySerial.println(F("Starting modem setup..."));

    // Power on modem
    digitalWrite(modemPwrDwn, HIGH); 
    delay(5000);  // Give time for the modem to boot up

    for (int i=0; i<5; i++) {
        Serial.println(F("AT"));
        delay(500);
    }

    //Serial.println(F("AT+CBC"));
    //Serial.flush();
    Serial.end();
    Serial.begin(38400);


    // Step 1: Check modem readiness (AT)
    if (!retryStep(F("AT"), F("OK"), MODEM_DEFAULT_TIMEOUT)) {
        handle_modem_failure();
        return;
    }

    // Step 3: Attach to NB-IoT or LTE-M
    if (!retryStep(F("AT+COPS=0"), F("OK"), MODEM_DEFAULT_TIMEOUT)) {
        handle_modem_failure();
        return;
    }

    // Step 3: Set APN
    if (!retryStep(F("AT+CGDCONT=1,\"IP\",\"iot.1nce.net\""), F("OK"), MODEM_DEFAULT_TIMEOUT)) {
        handle_modem_failure();
        return;
    }

    if (!retryStep(F("AT+CNCFG=1,\"iot.1nce.net\""), F("OK"), MODEM_DEFAULT_TIMEOUT)) {
        handle_modem_failure();
        return;
    }

    // Step 4: Set network mode to LTE
    if (!retryStep(F("AT+CNMP=38"), F("OK"), MODEM_DEFAULT_TIMEOUT)) {
        handle_modem_failure();
        return;
    }

    // Step 5: Attach to NB-IoT or LTE-M
    if (!retryStep(F("AT+CMNB=3"), F("OK"), MODEM_DEFAULT_TIMEOUT)) {
        handle_modem_failure();
        return;
    }

    mySerial.println(F("Waiting for network registration (LTE/NB-IoT)..."));
    int registrationAttempts = 0;
    int maxRegistrationAttempts = 6;  // Maximum retries set to 60 seconds
    bool registered = false;
    while (registrationAttempts < maxRegistrationAttempts) {
        if (checkRegistration(F("AT+CEREG?"))) {
            mySerial.println(F("SUCCESSFUL REGISTRATION"));
            registered = true;
            break;
        }
        mySerial.println(F("FAILED REGISTRATION"));
        registrationAttempts++;
        delay(5000);
    }

    if (!registered) {
        // Step 6: If LTE/NB-IoT registration fails, set LTE-M only mode
        mySerial.println(F("LTE/NB-IoT registration failed. Switching to LTE-M only mode..."));
        if (!retryStep(F("AT+CMNB=1"), F("OK"), MODEM_DEFAULT_TIMEOUT)) {
            handle_modem_failure();
            return;
        }

        // Step 7: Check network registration (LTE-M)
        mySerial.println(F("Waiting for network registration (LTE-M)..."));
        registrationAttempts = 0;
        while (registrationAttempts < maxRegistrationAttempts) {
            if (checkRegistration(F("AT+CEREG?"))) {
                mySerial.println(F("SUCCESSFUL LTE-M REGISTRATION"));
                registered = true;
                break;
            }
            mySerial.println(F("FAILED LTE-M REGISTRATION"));
            registrationAttempts++;
            delay(5000);
        }
    }

    if (!registered) {
        // Step 8: If LTE-M registration fails, try GSM
        maxRegistrationAttempts = 12;  // Maximum retries set to 60 seconds
        mySerial.println(F("LTE-M registration failed. Switching to GSM mode..."));
        if (!retryStep(F("AT+CNMP=13"), F("OK"), MODEM_DEFAULT_TIMEOUT)) {
            handle_modem_failure();
            return;
        }

        // Step 9: Check network registration (GSM)
        mySerial.println(F("Waiting for network registration (GSM)..."));
        registrationAttempts = 0;
        while (registrationAttempts < maxRegistrationAttempts) {
            if (checkRegistration(F("AT+CREG?"))) {
                mySerial.println(F("SUCCESSFUL GSM REGISTRATION"));
                registered = true;
                break;
            }
            mySerial.println(F("FAILED GSM REGISTRATION"));
            registrationAttempts++;
            delay(5000);
        }
    }

    if (!registered) {
        handle_modem_failure();
        return;
    }

    char tempByte; 
    while(Serial.available() > 0 ) tempByte = Serial.read();
    mySerial.println(tempByte);
    Serial.end();
    Serial.begin(38400);
    inputString = "";
    delay(1000);

    for (int i=0;i<3600;i++){
      digitalWrite(LED_BUILTIN, HIGH);
      delay(1000);
      digitalWrite(LED_BUILTIN, LOW); 
      delay(1000);
      wdt_reset();
    }

    // Step 10: Activate PDP context
    if (!retryStep(F("AT+CNACT=1,\"iot.1nce.net\""), " ACTIVE", 10000)) {
        handle_modem_failure();
        return;
    }

    // Step 12: Request IMSI
    if (!retryStep(F("AT+CIMI"), F("OK"), MODEM_DEFAULT_TIMEOUT)) {
        handle_modem_failure();
        return;
    }


    IMSI = inputString.substring(12, 27);

    if (!retryStep(F("AT+CBC"), F("OK"), MODEM_DEFAULT_TIMEOUT)) {
        handle_modem_failure();
        return;
    }

    String str_voltage = inputString.substring(22, 27);
    mySerial.println(str_voltage);
    battery_voltage  = str_voltage.toFloat()/1000;
    mySerial.println(battery_voltage);

    sendMQTT();
}

void sendMQTT(){

    display_freeram();
    Serial.println(F("AT+CMEE=1"));
    delay(200);
    Serial.println(F("AT+SMSTATE?"));
    delay(200);
    Serial.println(F("AT+SMCONF=\"CLEANSS\",1"));
    delay(200);
    Serial.println(F("AT+SMCONF=\"URL\",\"tilt-api.crosstech.co.uk\",\"1883\""));
    delay(200);
    Serial.println(F("AT+SMCONF=\"KEEPTIME\",60"));
    delay(200);
    Serial.println(F("AT+SMCONF=\"USERNAME\",\"node\""));
    delay(200);
    Serial.println(F("AT+SMCONF=\"PASSWORD\",\"Mz97GCduFDaqXyhEBWJRH8\""));
    delay(200);
    Serial.println(F("AT+SMPUBHEX=1"));
    delay(200);
    Serial.print(F("AT+SMCONF=\"CLIENTID\",\""));
    Serial.print(IMSI);
    Serial.println(F("\""));
    delay(200);

    char tempByte; 
    while(Serial.available() > 0 ) tempByte = Serial.read();
    mySerial.println(tempByte);
    Serial.end();
    Serial.begin(38400);
    inputString = "";
    delay(1000);
    
    if (!retryStep(F("AT+SMCONN"), F("OK"), 10000)) {
          handle_modem_failure();
    return;
    }

    delay(2000); 

    if (!retryStep("AT+SMSUB=\"" + MQTT_topic +  "\",1", F("OK"), MODEM_DEFAULT_TIMEOUT)) {
          handle_modem_failure();
          return;
    }

    delay(1000); 

    if ((MQTT_topic=="alarm") || (MQTT_topic=="check_in")){
           if (!retryStep("AT+SMPUB=\"" + MQTT_topic + "\",\"60\",1,1", F(">"), MODEM_DEFAULT_TIMEOUT)) {
                 handle_modem_failure();
                 return;
           }
    } else if(MQTT_topic=="status"){
          int length = MQTT_message.length() / 2;
          if (!retryStep("AT+SMPUB=\"" + MQTT_topic + "\",\"" + length + "\",1,1", F(">"), MODEM_DEFAULT_TIMEOUT)) {
                handle_modem_failure();
                return;
          }                     
    }

    if ((MQTT_topic=="alarm") || (MQTT_topic=="check_in")){
        for(int i=0; i<15; i++){ 
          if (fltSend[i] == 0) {
            fltSend[i] = 0.1; //patch for trailing zero in modem
          }
          converter.f = fltSend[i];
          
          for (int j = 0; j < 4; j++) { // Start from the most significant byte

            if(i==14){ //send last as battery
              //mySerial.println("SENDING BATTTERY");
              //mySerial.println(battery_voltage);
              converter.f = battery_voltage;
            } 
          
            if(converter.b[j] < 16) Serial.print("0"); // Print a leading zero for bytes < 16 to ensure 2 characters are printed
            //mySerial.println(fltSend[i]);            
            Serial.print(converter.b[j], HEX);
            Serial.flush();
          }
          
        }
        Serial.println("");
    } else if(MQTT_topic=="status"){
          Serial.println(MQTT_message);            
          Serial.flush();                     
    }

    delay(1000); 

    if (!retryStep("AT+SMUNSUB=\"" + MQTT_topic + "\"", F("OK"), MODEM_DEFAULT_TIMEOUT)) {
                 handle_modem_failure();
                 return;
    }

    delay(500); 


    mySerial.println(F("SETTING MQTT SUCCESS FLAG!"));
    flash.clear_buffer();
    flash[5] = 'Y'; //y means  sent successfully
    flash.write_page(1);
    mySerial.println(F("SAVED MQTT SUCCESS FLAG!"));

    if (MQTT_topic=="alarm") {
        mySerial.println(F("MQTT_TOPIC=ALARM"));
        ALARMS_SENT +=1; //every time it sends an alarm, add one to the total
    } else if (MQTT_topic=="check_in"){
        mySerial.println(F("MQTT_TOPIC=CHECKIN"));
        ALARMS_SENT =0; //reset the number of alarm flags every time it checks in
        mySerial.println(F("Reset Alarms Sent"));
        meanX=0;
        M2X=0;
        nX=0;
        meanY=0;
        M2Y=0;
        nY=0;
        meanZ=0;
        M2Z=0;
        nZ=0;
        CATASTROPHIC_ALARM_SENT=0;
    }
    
    getHTTP();

    delay(20);
}

void getHTTP(){
  wdt_reset();
  
  //   Serial.println(F("AT+SHCONF=\"URL\",\"http://tilt-api.crosstech.co.uk\""));
  //   delay(200);
  //   Serial.println(F("AT+SHCONF=\"BODYLEN\",1024"));
  //   delay(200);
  //   Serial.println(F("AT+SHCONF=\"HEADERLEN\",350"));
  //   delay(200);
  //   Serial.println(F("AT+SHCONN"));
  //   delay(200);
  //   Serial.println(F("AT+SHSTATE?"));
  //   delay(200);
  //   Serial.println(F("AT+SHCHEAD"));
  //   delay(200);
  //   Serial.println(F("AT+SHAHEAD=\"User-Agent\",\"curl/7.47.0\""));
  //   delay(200);
  //   Serial.println(F("AT+SHAHEAD=\"Cache-control\",\"no-cache\""));
  //   delay(200);
  //   Serial.println(F("AT+SHAHEAD=\"Connection\",\"keep-alive\""));
  //   delay(200);
  //   Serial.println(F("AT+SHAHEAD=\"Accept\",\"*/*\""));
  //   delay(200);
  //   Serial.println(F("AT+SHAHEAD=\"User-Agent\",\"curl/7.47.0\""));
  //   delay(200);

  //   char tempByte; 
  //   while(Serial.available() > 0 ) tempByte = Serial.read();
  //   mySerial.println(tempByte);
  //   Serial.end();
  //   Serial.begin(38400);
  //   inputString = "";
  //   delay(1000);

  // if (!retryStep("AT+SHREQ=\"/config/" + IMSI + "\",1", "SHREQ", MODEM_DEFAULT_TIMEOUT)) {
  //     handle_modem_failure();
  //     return;
  // }

  //   while(Serial.available() > 0 ) tempByte = Serial.read();
  //   mySerial.println(tempByte);
  //   Serial.end();
  //   Serial.begin(38400);
  //   inputString = "";
  // delay(1000);

  // if (!retryStep("AT+SHREAD=0,5", "+SHREAD:", MODEM_DEFAULT_TIMEOUT)) {
  //     handle_modem_failure();
  //     return;
  // }

  // delay(1000);

  // while (Serial.available()) {
  //   char inChar = Serial.read();
  //   inputString += inChar;   
  // }
    
  // THRESHOLD = inputString.substring(64,67);
  // mySerial.println(F("Threshold:"));
  // mySerial.println(THRESHOLD);
  
  // String convertedString = "";

  // // Convert the resulting ASCII string to an integer
  //   int integerValue = THRESHOLD.toInt();
  
  // fltAlarmThreshold = (float)integerValue / 10.0;
  // if ((fltAlarmThreshold>0) && (fltAlarmThreshold<180)){
  //   mySerial.print(F("The float value is: "));
  //   mySerial.println(fltAlarmThreshold, 1);  // Display up to 1 decimal place
  //   mySerial.println(inputString.substring(3,4));
  //   mySerial.println(inputString.substring(4,5));
  // }else{
  //   while(1);
  // }

  // if (inputString.substring(67,68) == "0"){
  //   mySerial.println(F("Setting Zero Flag to False"));
  //   ZERO_FLAG = 0; //REMOVE THIS 
  // } else if (inputString.substring(67,68) == "1"){
  //   mySerial.println(F("Setting Zero Flag to True"));
  //   ZERO_FLAG = 1;
  // }
  
  // if (inputString.substring(68,69) == "0"){
  //   mySerial.println(F("Setting Phone Home Frequency to Daily"));
  //   PHONE_HOME_FLAG = 0; //daily phone home
  //   phone_home_threshold = 2167; // 10; // ; //this is how many events it takes before sending 39.88s is one fill -- 90 per hour -- 1083 per 12 hours --2167 per 24
  // } else if (inputString.substring(68,69) == "1"){
  //   mySerial.println(F("Setting Phone Home Frequency to Every 2 Days"));
  //   PHONE_HOME_FLAG = 1; //daily phone twice a day
  //   phone_home_threshold = 4334; //this is how many events it takes before sending 39.88s is one fill -- 90 per hour -- 1083 per 12 hours --2167 per 24

  // } else if (inputString.substring(68,69) == "2"){  
  //   mySerial.println(F("Setting Phone Home Frequency to Weekly"));
  //   PHONE_HOME_FLAG = 2; //daily phone home hourly
  //   phone_home_threshold = 15169; //this is how many events it takes before sending 39.88s is one fill -- 90 per hour -- 1083 per 12 hours --2167 per 24
  // }

      if (!retryStep(F("AT+CNACT=0"), F("OK"), MODEM_DEFAULT_TIMEOUT)) {
                 handle_modem_failure();
                 return;
    }


  SLEEP_BLOCKED = 0;
  digitalWrite(modemPwrDwn, LOW);
  mySerial.println(F("Modem to Sleep"));
  setupTilt();  

}

void display_freeram() {
  mySerial.print(F("- SRAM left: "));
  mySerial.println(freeRam());
}

void reset_sim(){
  mySerial.println("CONNECTION ERROR CAUGHT");
  mySerial.println(F("SETTING UP MODEM..."));
  MsTimer2::stop();
  Serial.println(F("ATZ0"));
  delay(5000);
  wdt_reset();
  Serial.println(F("AT"));
  wdt_reset();
  delay(3000);
  Serial.println(F("ATE1"));
  delay(3000);
  wdt_reset();
  Serial.println(F("AT+CGMM"));
  delay(3000);
  wdt_reset();
  Serial.println(F("AT+CPIN?"));
  delay(3000);
  wdt_reset();
  Serial.println(F("AT+CSQ"));
  delay(3000);
  wdt_reset();
  Serial.println(F("AT+CGREG?"));
  delay(3000);
  wdt_reset();
  // Serial.println(F("AT+CGREG=0"));
  // delay(3000);
  Serial.println(F("AT+CGATT?"));
  delay(3000);
  wdt_reset();
  Serial.println(F("AT+CGACT?"));
  delay(3000);
  wdt_reset();
  Serial.println(F("AT+COPS?"));
  delay(3000);
  wdt_reset();

  retry_attempts=0;
  
  for (int i =0; i<180; i++){
    mySerial.println(i);
    delay(1000);
    wdt_reset();
  } 
}

int freeRam() {
  extern int __heap_start,*__brkval;
  int v;
  return (int)&v - (__brkval == 0  
    ? (int)&__heap_start : (int) __brkval);  
}

void connection_watchdog(){
  //isr for NTP time watchdog
  wdt_reset();
  mySerial.print(retry_counter);

  if (retry_counter == MODEM_TIMEOUT){ //after 4*8seconds
    mySerial.println(F("MODEM SLEEP"));
    digitalWrite(modemPwrDwn, LOW); //sleep modem
  }else if (retry_counter ==(MODEM_TIMEOUT+5)){
    mySerial.println(F("MODEM WAKE"));
    digitalWrite(modemPwrDwn, HIGH); //wake modem
  }else if (retry_counter ==MODEM_TIMEOUT+10){
    retry_attempts+=1;
    MsTimer2::stop();
    mySerial.println(F("Restarting Modem Connect"));
    retry_counter=0;
    connectModem();
  }
  retry_counter+=1;
  
}

void send_delay_isr(){
  MsTimer2::stop();
  mySerial.print(F("Timer2 stopped"));
  sendMQTT();
}

void updateWelford(float x, float &mean, float &M2, unsigned long &n) {                                      
  n++;
  float delta = x - mean;
  mean = mean + delta / n;
  M2 = M2 + delta * (x - mean); 
}

void bufferFull(){

    // Read the 'raw' data in 14-bit counts
  float x_sd =0;
  float y_sd = 0;
  float z_sd =0;

  ALARM_NOW=0;  

  mySerial.print(F("Cycles Completed:"));
  mySerial.print(phone_home_counter);
  mySerial.print(F("\tOut of:"));
  mySerial.println(phone_home_threshold);
  
    for (int i=0; i<arraySize;i++){
        float xi = x[i];
        float yi = y[i];
        float zi = z[i];
        float xi_g = x[i] / RANGE;
        float yi_g = y[i] / RANGE;
        float zi_g = z[i] / RANGE;

        float thetaXi = atan2(xi_g , sqrt(yi_g *yi_g  + zi_g*zi_g)) * (180.0 / M_PI);  
        float thetaYi = atan2(zi_g , sqrt(xi_g *xi_g  + yi_g*yi_g)) * (180.0 / M_PI);
        float thetaZi = atan2(sqrt(xi_g *xi_g  + zi_g*zi_g ), -1*yi_g) * (180.0 / M_PI);

       
        mySerial.print(F("\t"));
        mySerial.print(F("X:\t")); mySerial.print(thetaXi);
        mySerial.print(F("\tY:\t")); mySerial.print(thetaYi); 
        mySerial.print(F("\tZ:\t")); mySerial.print(thetaZi);
        mySerial.println(F(""));

        if (ZERO_FLAG){ //for the first sample, set it as the zero value
            mySerial.println(F("-----------"));
            mySerial.println(F("ZEROING NOW"));
            mySerial.println(F("-----------"));
            x_zero = thetaXi;
            y_zero = thetaYi;
            z_zero = thetaZi;
            mySerial.print(F("X Zero:"));mySerial.print(x_zero);mySerial.print(F("\tY Zero:"));mySerial.print(y_zero);mySerial.print(F("\tZ Zero:"));mySerial.println(z_zero);
            ZERO_FLAG =0;
            ALARM_NOW=0;
            ALARMS_SENT=0;
            mySerial.println(F("Reset Alarms Sent"));
        } 

        if ((thetaXi > (x_zero + fltAlarmThreshold)) || (thetaXi < (x_zero-fltAlarmThreshold)) || (thetaYi > (y_zero + fltAlarmThreshold)) || (thetaYi < (y_zero-fltAlarmThreshold)) || (thetaZi > (z_zero + fltAlarmThreshold)) || (thetaZi < (z_zero-fltAlarmThreshold))) {
          
          ALARM_NOW = 1;

          if ((thetaXi > (x_zero + (3*fltAlarmThreshold))) || (thetaXi < (x_zero-(3*fltAlarmThreshold))) || (thetaYi > (y_zero + (3*fltAlarmThreshold))) || (thetaYi < (y_zero-(3*fltAlarmThreshold))) || (thetaZi > (z_zero + (3*fltAlarmThreshold))) || (thetaZi < (z_zero-(3*fltAlarmThreshold)))){
            // mySerial.println(F("CATASTROPHIC ALARM Triggered"));
            
            //implement a limit to catastrophic alarms, maybe by using
            if (CATASTROPHIC_ALARM_SENT==0){
              ALARMS_SENT=0; //reset the alarms if the threshold is 3*threshold, but only once
              mySerial.println(F("Reset Alarms Sent"));
              CATASTROPHIC_ALARM_SENT =1;
            }
          } 
        }
        
        
        updateWelford(thetaXi, meanX, M2X, nX);
        updateWelford(thetaYi, meanY, M2Y, nY);
        updateWelford(thetaZi, meanZ, M2Z, nZ);


        stdDevX = sqrt(M2X / nX);
        stdDevY = sqrt(M2Y / nY);
        stdDevZ = sqrt(M2Z / nZ);


        if (thetaXi < xmin){
          xmin = thetaXi;
        }

        if (thetaYi < ymin){
          ymin = thetaYi;
        }

        if (thetaZi < zmin){
          zmin = thetaZi;
        }

        if (thetaXi > xmax){
          xmax = thetaXi;
        }

        if (thetaYi > ymax){
          ymax = thetaYi;
        }

        if (thetaZi > zmax){
          zmax = thetaZi;
        }

        
    }

        mySerial.print(F("\t"));
        delay(10);
        mySerial.print(F("Xmin: \t"));mySerial.print(xmin);mySerial.print(F("\tXmax: \t")); mySerial.print(xmax); mySerial.print(F("\tX_SD: \t")); mySerial.print(stdDevX, 6);mySerial.print(F("\X Mean: \t")); mySerial.print(meanX); 
        mySerial.println();
        delay(10); 
        mySerial.print(F("\tYmin: \t"));mySerial.print(ymin);mySerial.print(F("\tYmax: \t")); mySerial.print(ymax); mySerial.print(F("\tY_SD: \t")); mySerial.print(stdDevY, 6); mySerial.print(F("\Y Mean: \t")); mySerial.print(meanY); 
        mySerial.println();
        delay(10);
        mySerial.print(F("\tZmin: \t"));mySerial.print(zmin);mySerial.print(F("\tZmax: \t")); mySerial.print(zmax); mySerial.print(F("\tZ_SD: \t")); mySerial.print(stdDevZ, 6);mySerial.print(F("\Z Mean: \t")); mySerial.print(meanZ); 
        delay(10);
        mySerial.println();
        //mySerial.print(F("\tPitch (x): \t"));mySerial.print(theta_pitch);mySerial.print(F("\tRoll (y): \t")); mySerial.print(theta_roll); mySerial.print(F("\tTilt: \t")); mySerial.print(overall_tilt);

        

    if (phone_home_counter==phone_home_threshold || (ALARM_NOW && (ALARMS_SENT< MAX_ALARMS_PER_CHECKIN))){
      SLEEP_BLOCKED = 1;
      detachInterrupt(0);
      mySerial.println(F("Sending Status Update"));
      phone_home_counter =0;

      int msglength = 10;
    

      fltSend[0] = xmin;
      fltSend[1] = ymin;
      fltSend[2] = zmin;
      fltSend[3] = xmax;
      fltSend[4] = ymax;
      fltSend[5] = zmax;
      fltSend[6] = meanX;
      fltSend[7] = meanY;
      fltSend[8] = meanZ;
      fltSend[9] = stdDevX;
      fltSend[10] = stdDevY;
      fltSend[11] = stdDevZ;
      fltSend[12] = x_zero;
      fltSend[13] = y_zero;
      fltSend[14] = z_zero;
  

      MQTT_topic = "check_in";
      mySerial.print(F("ALARM NOW:"));
      mySerial.println(ALARM_NOW);
      mySerial.print(F("ALARMS SENT:"));
      mySerial.println(ALARMS_SENT);
      

      if (ALARM_NOW && (ALARMS_SENT< MAX_ALARMS_PER_CHECKIN)){
        MQTT_topic = "alarm";
        mySerial.print(F("Alarm Attempt:"));mySerial.println(ALARMS_SENT);
      }
      mySerial.println("CHAR MESSAGE:");
      mySerial.println(F("Xmin\tYmin\tZmin\tXmax\tYmax\tZmax\tXmean\tYmean\tZmean\tXsd\tYsd\tZsd\tXzero\tYzero\tZzero"));
      for (int i=0; i<15; i++){
          mySerial.print(fltSend[i]);
          mySerial.print(F("\t"));
      }
      


      write_data(); //write message to flash in-case of cockups

      xmin = 500;
      ymin = 500;
      zmin = 500;
      xmax = -500;
      ymax = -500;
      zmax = -500;

      connectModem();       
  }

    counter=0;

    x_sd =0;
    y_sd=0;
    z_sd=0;


    phone_home_counter+=1; 

}

void tiltDataReady(){

    long int t3 = millis();
    //digitalWrite(LED_BUILTIN, HIGH);
    //mySerial.println(F("Data Received from IMU"));
    //mySerial.println(counter);
    mma.read();
    x[counter] = mma.x;
    //x_total += x[counter];
    y[counter] = mma.y;
    //y_total += y[counter];
    z[counter] = mma.z;
    //z_total += z[counter];
    dataReady=0;
    counter++;
    //digitalWrite(LED_BUILTIN, LOW);  
    long int t4 = millis();  
    //mySerial.print(F("Time taken by last data acquisition: ");
    //mySerial.print(t4-t3);
    //mySerial.println(F(" milliseconds");
}

void loop() {



   if (counter==arraySize){

    bufferFull();
  }

  if (dataReady){ //accelerometer has data ready (interrupt)
    wdt_reset();
    //mySerial.println(F("DR Watchdog Reset"));    
    tiltDataReady();
  }

  if (!SLEEP_BLOCKED){
    //mySerial.println(F("Going to Sleep"));
    mySerial.flush();
    attachInterrupt(0, wakeUp, HIGH); //sleep on int pin
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); 
    detachInterrupt(0);
  }
}


// Non-blocking retry function using millis for timing
bool retryStep(String command, String expectedResponse, unsigned long timeout) {
    int retries = 0;
    bool success = false;

    mySerial.print("?");
    mySerial.print(command);

    while (retries < MODEM_MAX_RETRIES && !success) {
        success = sendATCommand(command, expectedResponse, timeout);

        if (!success) {
            mySerial.print(F("Retrying command: "));
            mySerial.println(command);
            retries++;
        } else {
            mySerial.print(F("Command successful: "));
            mySerial.println(command);
        }
    }

    return success;
}

// Non-blocking AT command function using millis for timeout
bool sendATCommand(String &command, String &expectedResponse, unsigned long timeout) {
    
    mySerial.print("sendATCommand COMMAND SENDING:");
    mySerial.println(command);
    
    inputString = "";  // Clear input buffer
    Serial.println(command);  // Send AT command to the modem
    Serial.flush();
    unsigned long startTime = millis();
    bool success = false;

    // Wait for a response or timeout
    while (millis() - startTime < timeout) {
        if (Serial.available()) {
            char c = Serial.read();
            inputString += c;
            
            //mySerial.print(c);
            //mySerial.print(" ");
            //mySerial.print(expectedResponse);
            //mySerial.print(" ");
            //mySerial.print(inputString.contains(expectedResponse));
            if (inputString.indexOf(expectedResponse) != -1) {
                mySerial.println("!");
                success = true;
                break;
            }
        }
        // Reset watchdog periodically
        wdt_reset();
    }

    if (!success) {
        mySerial.print(F("Command failed: "));
        mySerial.println(command);
    }

    return success;
}

// Function to check registration status based on response
bool checkRegistration(String command) {
    inputString = "";  // Clear input buffer
    Serial.println(command);  // Send AT command to the modem
    unsigned long startTime = millis();
    bool success = false;

    // Wait for a response or timeout
    while (millis() - startTime < MODEM_DEFAULT_TIMEOUT) {
        if (Serial.available()) {
            char c = Serial.read();
            inputString += c;
            if (inputString.indexOf(",1") != -1 || inputString.indexOf(",5") != -1) {
                success = true;
                break;
            }
        }
        // Reset watchdog periodically
        wdt_reset();
    }

    if (!success) {
        mySerial.print(F("Registration check failed: "));
        mySerial.println(command);
    }

    return success;
}

void handle_modem_failure() {
    mySerial.println(F("Modem setup failed, going on and will retry tomorrow"));
    // Pretend that it worked & start logging again

    SLEEP_BLOCKED = 0;

    ALARM_NOW = 0;
    ALARMS_SENT = 5;
    CATASTROPHIC_ALARM_SENT = 1;
    flash.clear_buffer();
    flash[5] = 'Y'; // y means sent successfully
    digitalWrite(modemPwrDwn, LOW);
    mySerial.println(F("Modem to Sleep"));
    sim_reset_attempts = 0;
    write_data();
    setupTilt();
    // Add any other failure handling, like resetting the modem or logging errors
}