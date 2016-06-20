#include <DS3231.h>

#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <Wire.h>
#include <DHT.h>
#include <string.h>

// ----------------------- Bluetooth -------------------------------- 
// replace with your channelâ€™s thingspeak API key,
#define IP "184.106.153.149" // thingspeak.com IP address
//String apiKey = "WJI6NI76Y57L4EK1";
String GET = "GET /update?api_key=WJI6NI76Y57L4EK1"; // API key
const char* ssid = "Alice_in_wonderLAN";
const char* password = "mbahjambrong";

// ----------------------- Temperature DHT --------------------------
#define         DHTPIN A1
#define         DHTTYPE DHT11                         // DHT 11 which is derived from the chart in datasheet
DHT dht(DHTPIN, DHTTYPE);
bool IsDHTEnabled = true;

// ----------------------- Movement PIR -----------------------------------
int PIRPIN = 8;               // choose the input pin (for PIR sensor)
int pirState = LOW;             // we start, assuming no motion detected
int val = 0;                    // variable for reading the pin status
bool IsPirEnabled = true;

// ----------------------- Sirene ----------------------------------------
int SPEAKERPIN = 12;
bool IsPanicActive = false;
bool IsSireneEnabled = true;

// ----------------------- CLock ----------------------------------------
DS3231  rtc(SDA, SCL);
double startingMinute;
bool isFirstFifteen;

// ------------------------ FAN ------------------------------------------
int FANPIN = 9;
double FANRUNTIME = 5; // in minutes
bool IsFanEnabled = true;
bool IsFanOn = false;
bool IsForcedOn = false;
int LastHour = 25;

// ----------------------- LED STATUS -----------------------------------
int REDPIN = 3;
int GREENPIN = 4;
int BLUEPIN = 5;
int LoopCounter = 0;

// ------------------------ FAN REVERSAL BUTTON --------------------------
#define LIGHTSENSOR A0
int REVERSERELAY = 7;
bool IsReverseEnabled = false;

/************************MQ-2 Gas Sensor************************************/
#define         MQ_PIN                       (2)     //define which analog input channel you are going to use
#define         RL_VALUE                     (5)     //define the load resistance on the board, in kilo ohms
#define         RO_CLEAN_AIR_FACTOR          (9.83)  //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
                                                     //which is derived from the chart in datasheet
                                                     
#define         CALIBARAION_SAMPLE_TIMES     (50)    //define how many samples you are going to take in the calibration phase
#define         CALIBRATION_SAMPLE_INTERVAL  (500)   //define the time interal(in milisecond) between each samples in the
                                                     //cablibration phase
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)     //define the time interal(in milisecond) between each samples in 
                                                     //normal operation
#define         GAS_LPG                      (0)
#define         GAS_CO                       (1)
#define         GAS_SMOKE                    (2)

float           LPGCurve[3]  =  {2.3,0.21,-0.47};   //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent"
                                                    //to the original curve. 
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.21), point2: (lg10000, -0.59) 
float           COCurve[3]  =  {2.3,0.72,-0.34};    //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.72), point2: (lg10000,  0.15) 
float           SmokeCurve[3] ={2.3,0.53,-0.44};    //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.53), point2: (lg10000,  -0.22)                                                     
float           Ro           =  10;                 //Ro is initialized to 10 kilo ohms

bool IsGasSensorEnable = true;
bool IsGasLeaking = false;
int GasTreshold = 30;

String LoadSettings(bool phoneRequest)
{
  char value[512];
  for(int i=0; i<=512; i++)
  {
    value[i]= (char)EEPROM.read(i);
    Serial.println((char)EEPROM.read(i));
  }

  String settingString;
  char *p = value;
  char *str;
  int i = 0;
  while ((str = strtok_r(p, ";", &p)) != NULL) // delimiter is the semicolon
  {
    bool isSkipped;
    if(str[i] == 'S') // speaker
    {
      IsSireneEnabled = (bool)str[i+2];
      Serial.print("Speaker status:");
      Serial.println(IsSireneEnabled);
    }
    else if(str[i] == 'P') // PIR
    {
      if(str[i+2] == '0')
        IsPirEnabled = false;
      else if(str[i+2] == '1')
        IsPirEnabled = true;
        
      Serial.print("PIR status:");
      Serial.println(IsPirEnabled);
    }
    else if(str[i] == 'T') // DHT
    {
      IsDHTEnabled = (bool)str[i+2];
      Serial.print("DHT status:");
      Serial.println(IsDHTEnabled);
    }
    else if(str[i] == 'G') // MQ2
    {
      IsGasSensorEnable = (bool)str[i+2];
      Serial.print("MQ2 status:");
      Serial.println(IsGasSensorEnable);
    }
    else if(str[i] == 'F') // Fan
    {
      IsFanEnabled = (bool)str[i+2];

      Serial.print("FAN status:");
      Serial.println(IsFanEnabled);
    }
    //else if(str[i] == 'D') // Timer
    //  FANRUNTIME = (
    else
    {
      isSkipped = true;
    }

    if(!isSkipped)
    {
      settingString += str[i] + str[i+1] + str[i+2] + ";";
    }
    
    Serial.println(str);
  }

  return settingString;
}

void UpdateSettings(String settingValue)
{
  Serial.println("UPDATING");
  for(int i = 0; i < settingValue.length(); i++)
  {
    EEPROM.write(i, (byte)settingValue[i]);
    Serial.println(settingValue[i]);
  }
  LoadSettings(false);
}

// Get Movement Reading
bool GetMovementReadings()
{
  if(!IsPirEnabled)
  {
    return false;
  }

  val = digitalRead(PIRPIN);  // read input value
  if (val == HIGH) {            // check if the input is HIGH
    if (pirState == LOW) 
    {
      // we have just turned on
      Serial.println("Motion detected!");
      // We only want to print on the output change, not state
      pirState = HIGH;
    }
  }
  else 
  { 
    if (pirState == HIGH)
    {
      // we have just turned off
      Serial.println("Motion ended!");
      // We only want to print on the output change, not state
      pirState = LOW;
    }
  }
  
  if(IsSireneEnabled)
  {
    digitalWrite(SPEAKERPIN, pirState);
  }
  
  return pirState;
}

// Get MQ2 Sensor value
String GetAirReadings()
{
  String mq2Info = "~L:";
  int lpg = (MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_LPG));
  mq2Info += lpg;
  mq2Info += "~|~C:";
  int co = (MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_CO));
  mq2Info += co;
  mq2Info += "~|~S:"; 
  int smoke = (MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_SMOKE)); 
  mq2Info += smoke;
  mq2Info += "~|";

  IsGasLeaking = (lpg > GasTreshold) || (co > GasTreshold) || (smoke > GasTreshold); 
  if(IsGasLeaking)
  {
    digitalWrite(REDPIN, LOW);
    digitalWrite(GREENPIN, HIGH);
    digitalWrite(SPEAKERPIN, HIGH);
  }
  else
  {
    digitalWrite(REDPIN, HIGH);
    digitalWrite(SPEAKERPIN, LOW);
  }

  return mq2Info;
}

// Get Temperature reading
float GetTemperature()
{  
  return dht.readTemperature();
}

float GetHumidity()
{
  return dht.readHumidity();
}

bool GetFanTimer()
{
  //byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
  // retrieve data from DS3231
  char *p = rtc.getTimeStr(FORMAT_SHORT);
  char *str;
  String sz[2];
  int i = 0;
  
  while ((str = strtok_r(p, ":", &p)) != NULL) // delimiter is the semicolon
  {
    sz[i] = (String)str;
    i++;
  }

  int hour = sz[0].toInt();
  int minute = sz[1].toInt();
  if(LastHour != hour)
  {
    startingMinute = minute;
    isFirstFifteen = true;
    LastHour = hour;
  }
  
  if(isFirstFifteen)
  {
    if(startingMinute <  (60 - FANRUNTIME) && minute > startingMinute + FANRUNTIME)
    {
      isFirstFifteen = false;
      return false;
    }
    else if(startingMinute >= (60 - FANRUNTIME) && (startingMinute + FANRUNTIME) - 60 > minute)
    {
      isFirstFifteen = false;
      return false;
    }
    else
    {
      return true;
    }
  }
return false;
}



/****************** MQResistanceCalculation ****************************************
Input:   raw_adc - raw value read from adc, which represents the voltage
Output:  the calculated sensor resistance
Remarks: The sensor and the load resistor forms a voltage divider. Given the voltage
         across the load resistor and its resistance, the resistance of the sensor
         could be derived.
************************************************************************************/ 
float MQResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE*(1023-raw_adc)/raw_adc));
}
 
/***************************** MQCalibration ****************************************
Input:   mq_pin - analog channel
Output:  Ro of the sensor
Remarks: This function assumes that the sensor is in clean air. It use  
         MQResistanceCalculation to calculates the sensor resistance in clean air 
         and then divides it with RO_CLEAN_AIR_FACTOR. RO_CLEAN_AIR_FACTOR is about 
         10, which differs slightly between different sensors.
************************************************************************************/ 
float MQCalibration(int mq_pin)
{
  int i;
  float val=0;
 
  for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {            //take multiple samples
    digitalWrite(GREENPIN, 0);
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL / 2);
    digitalWrite(GREENPIN, 1);
    delay(CALIBRATION_SAMPLE_INTERVAL / 2);
  }
  val = val/CALIBARAION_SAMPLE_TIMES;                   //calculate the average value
 
  val = val/RO_CLEAN_AIR_FACTOR;                        //divided by RO_CLEAN_AIR_FACTOR yields the Ro 
                                                        //according to the chart in the datasheet  
  return val; 
}
/*****************************  MQRead *********************************************
Input:   mq_pin - analog channel
Output:  Rs of the sensor
Remarks: This function use MQResistanceCalculation to caculate the sensor resistenc (Rs).
         The Rs changes as the sensor is in the different consentration of the target
         gas. The sample times and the time interval between samples could be configured
         by changing the definition of the macros.
************************************************************************************/ 
float MQRead(int mq_pin)
{
  int i;
  float rs=0;
 
  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }
 
  rs = rs/READ_SAMPLE_TIMES;
 
  return rs;  
}
 
/*****************************  MQGetGasPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
         gas_id      - target gas type
Output:  ppm of the target gas
Remarks: This function passes different curves to the MQGetPercentage function which 
         calculates the ppm (parts per million) of the target gas.
************************************************************************************/ 
int MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if ( gas_id == GAS_LPG ) {
     return MQGetPercentage(rs_ro_ratio,LPGCurve);
  } else if ( gas_id == GAS_CO ) {
     return MQGetPercentage(rs_ro_ratio,COCurve);
  } else if ( gas_id == GAS_SMOKE ) {
     return MQGetPercentage(rs_ro_ratio,SmokeCurve);
  }    
 
  return 0;
}
 
/*****************************  MQGetPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
         pcurve      - pointer to the curve of the target gas
Output:  ppm of the target gas
Remarks: By using the slope and a point of the line. The x(logarithmic value of ppm) 
         of the line could be derived if y(rs_ro_ratio) is provided. As it is a 
         logarithmic coordinate, power of 10 is used to convert the result to non-logarithmic 
         value.
************************************************************************************/ 
int  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10,( ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}

void sendData(float t, float h, bool isGasSafe, int lpg, int co, int smoke, bool isFanOn, float light)
{
  String cmd = "AT+CIPSTART=\"TCP\",\"";
  cmd += IP;
  cmd += "\",80";
 
  Serial.println(cmd);
  delay(5000);
 
  if (Serial.find("Error")) {
    return;
  }
  
  cmd = GET;
  cmd += "&field1=";
  cmd += String(t);
  cmd += "&field2=";
  cmd += String(h);
  cmd += "&field3=";
  cmd += String(isGasSafe);
  cmd += "&field4=";
  cmd += String(lpg);
  cmd += "&field5=";
  cmd += String(co);
  cmd += "&field6=";
  cmd += String(smoke);
  cmd += "&field7=";
  cmd += String(isFanOn);
  cmd += "&field8=";
  cmd += String(light); 
  cmd += "\r\n\r\n";
  Serial.print("AT+CIPSEND=");
  Serial.println(cmd.length());
  digitalWrite(13,1);
  delay(5000);
  digitalWrite(13,0);
  delay(5000);
  
  if (Serial.find(">")) { // Perintah AT+CIPSEND akan menampilkan prompt ">"
    digitalWrite(13,1);
    delay(200);
    digitalWrite(13,0);
    Serial.print(cmd);
  }

//  if (client.connect(server,80)) { // "184.106.153.149" or api.thingspeak.com
//  String postStr = apiKey;
//  postStr +="&field1=";
//  postStr += String(t);
//  postStr +="&field2=";
//  postStr += String(light);
//  postStr +="&field3=";
//  postStr += String(isGasSafe);
//  postStr +="&field4=";
//  postStr += String(isFanOn);
//  postStr += "\r\n\r\n";
//  
//  client.print("POST /update HTTP/1.1\n");
//  client.print("Host: api.thingspeak.com\n");
//  client.print("Connection: close\n");
//  client.print("X-THINGSPEAKAPIKEY: "+apiKey+"\n");
//  client.print("Content-Type: application/x-www-form-urlencoded\n");
//  client.print("Content-Length: ");
//  client.print(postStr.length());
//  client.print("\n\n");
//  client.print(postStr);
//  
//  Serial.print("Temperature: ");
//  Serial.print(t);
//  Serial.print(" degrees Celcius, Light: ");
//  Serial.print(light);
//  Serial.print(" ");
//  Serial.print("Gas Status: ");
//  if(isGasSafe)
//    Serial.print("OK");
//  else
//    Serial.print("DANGER");
//  }
//  Serial.print(" Fan Status: ");
//  if(isFanOn)
//  {
//    Serial.print("ON");
//  }
//  else
//  {
//    Serial.print("OFF");
//  }  
//  client.stop();
//  
//  Serial.println("Waitingâ€¦");
}

void setup() {
  dht.begin();
  // Initialize the rtc object
  rtc.begin();
  Wire.begin();
  Serial.begin(115200);
  Serial.write("AT+CWJAP=\"Alice_in_wonderLAN\",\"mbahjambrong\"");  
  delay(100);
  
  pinMode(SPEAKERPIN,OUTPUT);
  pinMode(REVERSERELAY, OUTPUT);
  pinMode(FANPIN, OUTPUT);
  pinMode(REDPIN, OUTPUT);
  pinMode(GREENPIN, OUTPUT);
  pinMode(BLUEPIN, OUTPUT);
  
  digitalWrite(REDPIN, HIGH);
  digitalWrite(GREENPIN, HIGH);
  digitalWrite(BLUEPIN, HIGH);
  digitalWrite(FANPIN, HIGH); // fan is reversed when installing the cable -_-
  digitalWrite(REVERSERELAY, HIGH); //reversed

  pinMode(13, OUTPUT);
  
  //MQ-2 Setup
    //UART setup, baudrate = 9600bps
  //Serial.println("Running..."); 
  //Serial.println("Calibrating...");                

  Ro = MQCalibration(MQ_PIN);                       //Calibrating the sensor. Please make sure the sensor is in clean air 

//  Serial.println("Calibration is done..."); 
//  Serial.println("Ro=");
//  Serial.print(Ro);
//  Serial.println("kohm");
  delay(3000);
  //UpdateSettings("S:1;P:0;T:1;G:1;F:0;D:15;");
  LoadSettings(false);

}

void loop() {
  if(!IsGasLeaking && !IsFanOn && LoopCounter % 5 == 0)
  {
    digitalWrite(GREENPIN, LOW);
    LoopCounter = 0;
  }
  else if(!IsGasLeaking && !IsFanOn && LoopCounter % 5 != 0)
  {
    digitalWrite(GREENPIN, HIGH);
  }
  else if(!IsGasLeaking && IsFanOn)
  {
    digitalWrite(BLUEPIN, LOW);
  }
  
  String input;
  digitalWrite(SPEAKERPIN, IsPanicActive);

  // Get Temperature reading
  float t = GetTemperature();
  float h = GetHumidity();

  GetAirReadings();
  int lpg = (MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_LPG));
  int co = (MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_CO));
  int smoke = (MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_SMOKE)); 

  float light = analogRead(LIGHTSENSOR);

//  if(IsPirEnabled)
//  {
//    message += "~M:";
//    message += GetMovementReadings();
//  }

  if(IsGasLeaking || IsForcedOn)
  {
    digitalWrite(REVERSERELAY, HIGH); //reversed
    digitalWrite(FANPIN, LOW);
  }
  else
  {
    if(light > 600)
    {
      delay(5000);
      // fan blow air TO kitchen
      IsReverseEnabled = true;
      digitalWrite(REVERSERELAY, LOW); //reversed
      digitalWrite(FANPIN, LOW);
    }
    else
    {
      // fan suck air FROM kitchen
      IsReverseEnabled = false;
      digitalWrite(REVERSERELAY, HIGH); // reversed
      if(GetFanTimer())
      {
        digitalWrite(GREENPIN, HIGH);
        digitalWrite(BLUEPIN, LOW);
        digitalWrite(FANPIN, LOW);
        IsFanOn = true;
      }
      else
      {
        digitalWrite(BLUEPIN, HIGH);
        digitalWrite(FANPIN, HIGH);
        IsFanOn = false;
      }
    }
  }
  sendData(t, h, IsGasLeaking, lpg, co, smoke, IsFanOn, light);
  
  delay(100);// prepare for next data ...
  LoopCounter += 1;
}

