#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <Wire.h>
#include <DHT.h>

// ----------------------- Bluetooth -------------------------------- 
SoftwareSerial Genotronex(10, 11); // RX, TX
String BluetoothData; // the data given from Computer
char inData[50]; // Allocate some space for the string
char inChar=-1; // Where to store the character read
byte index = 0; // Index into array; where to store the character

// ----------------------- Temperature DHT --------------------------
#define         DHTPIN A1
#define         DHTTYPE DHT11                         // DHT 11 which is derived from the chart in datasheet
#define         DHTTYPE DHT11
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
#define DS3231_I2C_ADDRESS 0x68
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
int LIGHTSENSOR = 2;
int REVERSERELAY = 7;
bool IsReverseEnabled = false;

/************************MQ-2 Gas Sensor************************************/
#define         MQ_PIN                       (0)     //define which analog input channel you are going to use
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
  Serial.println("XXXXXXXXXXXXXXX");
  //Serial.println(value);
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

String InputReader() {
  index = 0;
  while (Genotronex.available() > 0) // Don't read unless
                                 // there you know there is data
  {
      if(index < 50) // One less than the size of the array
      {
          inChar = Genotronex.read(); // Read a character
          inData[index] = inChar; // Store it
          index++; // Increment where to write next
          inData[index] = '\0'; // Null terminate the string
      }
      //Genotronex.flush();
      //Serial.println(inData);
  }
  
  Serial.print(inData);
  Serial.print("-------");
  //Serial.println(This);
  String result = String(inData);
//  if (strcmp(inData,This)  == 0) 
//  {
//    
//      result = true;
//  }
//  result = false;

  for (int i=0;i<50;i++) 
  {
    inData[i]=0;
  }
  index=0;

  return result;
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
String GetTemperature()
{
  String tempInfo;
  String temp;
  
  tempInfo += "T:";
  //dtostrf(dht.readTemperature(),3, 1, temp);
  tempInfo += String(dht.readTemperature(), 1);
  //tempInfo += temp;
  tempInfo += "~|";
  
  tempInfo += "~H:";
  //dtostrf(dht.readHumidity(),3, 1, temp);
  tempInfo += String(dht.readHumidity(),1);
  //tempInfo += temp;
  tempInfo += "~|";
  
  return tempInfo;
}

void readDS3231time(byte *second,
byte *minute,
byte *hour,
byte *dayOfWeek,
byte *dayOfMonth,
byte *month,
byte *year)
{
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0); // set DS3231 register pointer to 00h
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 7);
  // request seven bytes of data from DS3231 starting from register 00h
  *second = bcdToDec(Wire.read() & 0x7f);
  *minute = bcdToDec(Wire.read());
  *hour = bcdToDec(Wire.read() & 0x3f);
  *dayOfWeek = bcdToDec(Wire.read());
  *dayOfMonth = bcdToDec(Wire.read());
  *month = bcdToDec(Wire.read());
  *year = bcdToDec(Wire.read());
}

bool GetFanTimer()
{
  byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
  // retrieve data from DS3231
  readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month,&year);
//  if(hour % 2 == 0)
//  {

    if(LastHour != hour)
    {
      startingMinute = minute;
      isFirstFifteen = true;
      LastHour = hour;
      Serial.println("gethere!!!!!!!~~~~~~~~~~~~~~~~~");
    }
    
    if(isFirstFifteen)
    {
      Serial.print("Starting");
      Serial.println(startingMinute);
      if(startingMinute <  (60 - FANRUNTIME) && minute > startingMinute + FANRUNTIME)
      {
        Serial.println("123gethere!!!!!!!~~~~~~~~~~~~~~~~~");
        isFirstFifteen = false;
        return false;
      }
      else if(startingMinute >= (60 - FANRUNTIME) && (startingMinute + FANRUNTIME) - 60 > minute)
      {
        Serial.println("qwertygethere!!!!!!!~~~~~~~~~~~~~~~~~");
        isFirstFifteen = false;
        return false;
      }
      else
      {
        Serial.println("HEREHERE");
        return true;
      }
    }
//  }
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


// CLOCK HELPER
// Convert normal decimal numbers to binary coded decimal
byte decToBcd(byte val)
{
  return( (val/10*16) + (val%10) );
}
// Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val)
{
  return( (val/16*10) + (val%16) );
}

void setup() {

  //Bluetooth Setup
  // put your setup code here, to run once:
  Genotronex.begin(9600);
  Genotronex.println("Bluetooth On please press 1 or 0 blink LED ..");

  Wire.begin();
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
  digitalWrite(REVERSERELAY, LOW);
  
  
  //MQ-2 Setup
  Serial.begin(9600);  //UART setup, baudrate = 9600bps
  Serial.println("Running..."); 
  Serial.println("Calibrating...");                

  Ro = MQCalibration(MQ_PIN);                       //Calibrating the sensor. Please make sure the sensor is in clean air 

  Serial.println("Calibration is done..."); 
  Serial.println("Ro=");
  Serial.print(Ro);
  Serial.println("kohm");
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
  if(Genotronex.available())
  {
    input = (String)InputReader();
    input.trim();
  }
  if(input == "PIR_ON")
  {
    IsPirEnabled = true;
    IsSireneEnabled = true;
    Genotronex.println("PIR Activated ");
  }
  else if (input == "PIR_OFF")
  {
    IsPirEnabled = false;
    IsSireneEnabled = false;
    Genotronex.println("PIR Deactivated");
  }
  else if (input == "SIRENE_ON")
  {
    IsPanicActive = true;
    Serial.println("@@@@@@@@@@@@@@@@@@@@@@");
  }
  else if (input == "SIRENE_OFF")
  {
    IsPanicActive = false;
    Serial.println("^^^^^^^^^^^^^^^^^^^");
  }
  else if(input.startsWith("'UPDATE_SETTING"))
  {
    Serial.println("@@@@@@@@@@@@@@@@@@@@@@");
    input = input.substring(15, input.length());
    Serial.println(input);
    UpdateSettings(input);
  }
  else if(input.startsWith("READ_SETTING"))
  {
    Serial.println("-------RS--------");
    String settingMessage = LoadSettings(true);
    Serial.println(settingMessage);
    Genotronex.println(settingMessage);
  }
  else if(input.startsWith("FAN_ON"))
  {
    IsForcedOn = true;
  }
  else if(input.startsWith("FAN_OFF"))
  {
    IsForcedOn = false;
  }

  digitalWrite(SPEAKERPIN, IsPanicActive);

  String message = "|~";
  
  // Get Temperature reading
  if(IsDHTEnabled)
  {
    message += GetTemperature();
  }

  // Get MQ2 reading
  if(IsGasSensorEnable)
  {
    message += GetAirReadings();
  }

//  if(IsPirEnabled)
//  {
//    message += "~M:";
//    message += GetMovementReadings();
//  }

  message += "~|";
  Serial.print("value ");
  Serial.println(analogRead(LIGHTSENSOR));
  if(IsGasLeaking || IsForcedOn)
  {
    digitalWrite(REVERSERELAY, LOW);
    digitalWrite(FANPIN, LOW);
  }
  else
  {
    if(analogRead(LIGHTSENSOR) > 600)
    {
      delay(5000);
      // fan blow air TO kitchen
      IsReverseEnabled = true;
      digitalWrite(REVERSERELAY, HIGH);
      Serial.println("Fan Reversed");  
      Serial.println("Fan On");
      digitalWrite(FANPIN, LOW);
    }
    else
    {
      // fan suck air FROM kitchen
      IsReverseEnabled = false;
      digitalWrite(REVERSERELAY, LOW);
      Serial.println("Fan Normal");  
      if(GetFanTimer())
      {
        Serial.println("Fan On");
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

  Serial.println(message);
  Genotronex.println(message);
  
  delay(100);// prepare for next data ...
  LoopCounter += 1;
}
