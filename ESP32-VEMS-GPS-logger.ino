//-----------------------------------------------------
// ESP32-VEMS-GPS-logger
// Autor:   modemlamer
// Created: 04/2022
//-----------------------------------------------------

#include <SPI.h>
#include <TFT_eSPI.h>
#include <FS.h>
#include <SD.h>
#include <TinyGPS++.h> 
#include <HardwareSerial.h>
#include "EEPROM.h"
#include <Timezone.h>

TFT_eSPI tft = TFT_eSPI();

// Defines for playing field and dot
#define NUMROW 7        //Number of Rows (Spalten)
#define NUMCOLUMN 6     //Number of Columns (Reihen)
#define LINEWIDTH 3     //Wide of lines from playing field
#define XDOTBASIC 37    //Position 0 in x for dot
#define YDOTBASIC 210   //Position 0 in y for dot
#define BOXSHIFT 32     //Shift to next position in x and/or y for dot

// Defines for button
#define BUTTON_W 90
#define BUTTON_H 50
#define STARTBUTTON_X 10
#define STARTBUTTON_Y 180

#define STOPBUTTON_X 220
#define STOPBUTTON_Y 180

#define CALBUTTON_X 115
#define CALBUTTON_Y 180

uint16_t pixel_x, pixel_y;
static const int RX_VEMS = 25, TX_VEMS = 26;
static const int RX_GPS = 35, TX_GPS = 34;
#define EEPROM_SIZE 128

TinyGPSPlus gps;
HardwareSerial SerialGPS(1);
HardwareSerial roundSerial(2);

// Change these two rules corresponding to your timezone, see https://github.com/JChristensen/Timezone
//Central European Time (Frankfurt, Paris)  120 = +2 hours in daylight saving time (summer).
TimeChangeRule CEST = {"CEST", Last, Sun, Mar, 2, 120};
//Central European Time (Frankfurt, Paris)  60  = +1 hour in normal time (winter)
TimeChangeRule CET = {"CET ", Last, Sun, Oct, 3, 60};
Timezone CE(CEST, CET);

// time variables
time_t local, utc, prev_set;


struct GpsDataState_t {
  double originLat = 0;
  double originLon = 0;
  double originAlt = 0;
  double distMax = 0;
  double dist = 0;
  double altMax = -999999;
  double altMin = 999999;
  double spdMax = 0;
  double prevDist = 0;
};
GpsDataState_t gpsState = {};




int iEnableButtons = 1;   //(De-)activte "Start"-Button
int StartEnable = 0;
int StopEnable = 0;
int LogFileOpen = 0;
String filename = "";
String dot=".";
String delimiter = "-";
String csvdelimiter = ";";
String extension = ".csv";
int str_len =0;
String title = "GPS-log\nlat;lng;speed;satcount;date;time;Lambda;Lambda_RAW;EGT;Analog2;RPM;Vin;F01;F02;F03;F04;F05;F06;F07;F08;F09;F10;F11;F12;F13;F14;F15;F16;F17;F18;F19;F20;F21;F22;F23;F24;F25;F26;F27;F28;F29;F30;F31;F32;F33;F34;F35;F36;F37;F38;F39;F40;\n";
String logStringGPS = "";
String logStringVEMS = "";
String speedString = "";
String logString = "";
String logNewLine = "\n";
String logDay = "";
String logMonth = "";
String logYear = "";
String logHour = "";
String logMinute = "";
String logSecond = "";
String logDateString = "";
String logTimeString = "";
int logSessionCounter = 0;
int logLineCounter = 0;
int logFrameOnly = 0;
int hourOffset = 0;


String str0;
String str1;
String str2;
String str3;
String str4;


// VEMS Parameter

struct VEMS_Params
{
  String in_byte;
  String readString;
  String inputString;
  int checkPort=0;
  int counter=0;
  int int_message[40];
  int EGT=0;
  float Vin=0;
  int Analog0=0;
  int Analog0_RAW=0;
  int Analog2=0;
  int Analog2_RAW=0;
  float AFR=0;
  float Lambda=0;
  int RPM=0;
  int V_REV=0;
};

VEMS_Params vemsParams = {};

int DEBUGMODE=0;

// VEMS Parameter

struct CAR_PARAMS{
    float remoteTemperature[4] = {0.0,0.0,0.0,0.0};
    float lastRemoteTemperature[4] ={0.0,0.0,0.0,0.0};
    float maxOilTemp =0.0;
    float maxOilPress =0.0;
    float maxWaterTemp =0.0;
    float maxAirTemp =0.0;
    float maxBoost =0.0;
  
    int rawPress[2] = {1,1};
    int lastoilPress=2;
    int lastboostPress = 2;

    int EGT = 0;
    int lastEGT = 1;

    float Lambda = 0.0;
    float lastLambda = 0.1;
    
    double lastSpeed = 0.00;
    double maxSpeed = 0.00;

    
};

CAR_PARAMS carParams = {};

struct MAP_DATA{
  int Vin_in_min = 31800;
  int Vin_in_max = 34380;
  int Vin_out_min = 1070;
  int Vin_out_max = 1150;  // mVolt

  int L_in_min = 12692;
  int L_in_max = 16635;
  int L_out_min = 77;
  int L_out_max = 100;   // /100

  int BOOST_in_min = 500;
  int BOOST_in_max = 1400;
  float BOOST_out_min = 0;
  float BOOST_out_max = 1000; // mBar
};

MAP_DATA mapData = {};


uint16_t calibrationData[5];


void setthetime(void)
{
  int Year = gps.date.year();
  byte Month = gps.date.month();
  byte Day = gps.date.day();
  byte Hour = gps.time.hour();
  byte Minute = gps.time.minute();
  byte Second = gps.time.second();
  // Set Time from GPS data string
  setTime(Hour, Minute, Second, Day, Month, Year);  // set the time of the microcontroller to the UTC time from the GPS
}



void readFile(fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if(!file){
        Serial.println("Failed to open file for reading");
        return;
    }

    Serial.print("Read from file: ");
    while(file.available()){
        Serial.write(file.read());
    }
    file.close();
}

String readConfigFile(fs::FS &fs, const char * path){
    Serial.printf("Reading configFile: %s\n", path);
    String resultSet="";
    File file = fs.open(path);
    if(!file){
        Serial.println("Failed to open file for reading");
         resultSet="noFile";
        
    }else{
      
      while(file.available()){
        resultSet=file.readStringUntil('\n');	
      }
    }
    file.close();
    
    return resultSet;
}

void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("File written");
    } else {
        Serial.println("Write failed");
    }
    file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("Failed to open file for appending");
        return;
    }
    if(file.print(message)){
        Serial.println("Message appended");
    } else {
        Serial.println("Append failed");
    }
    file.close();
}

String ToString(uint64_t x)
{
     boolean flag = false; // For preventing string return like this 0000123, with a lot of zeros in front.
     String str = "";      // Start with an empty string.
     uint64_t y = 10000000000000000000;
     int res;
     if (x == 0)  // if x = 0 and this is not testet, then function return a empty string.
     {
           str = "0";
           return str;  // or return "0";
     }    
     while (y > 0)
     {                
            res = (int)(x / y);
            if (res > 0)  // Wait for res > 0, then start adding to string.
                flag = true;
            if (flag == true)
                str = str + String(res);
            x = x - (y * (uint64_t)res);  // Subtract res times * y from x
            y = y / 10;                   // Reducer y with 10    
     }
     return str;
}  

void setup()
{
  
  
  pinMode(15, OUTPUT);
  digitalWrite(15, LOW);
  digitalWrite( 2, HIGH);
  Serial.begin(115200);
  
  randomSeed(analogRead(34));
  if (!SD.begin(2)) {
    Serial.println("Card Mount Failed");
    
  }
 uint8_t cardType = SD.cardType();
 
  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    
  }
 
  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }
   uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
 
  Serial.println("initialisation done.");

  roundSerial.begin(19200, SERIAL_8N1, RX_VEMS, TX_VEMS);

  SerialGPS.begin(9600, SERIAL_8N1, RX_GPS, TX_GPS);

  while (!EEPROM.begin(EEPROM_SIZE)) {
    true;
  }

    long readValue;
  EEPROM_readAnything(0, readValue);
  gpsState.originLat = (double)readValue / 1000000;

  EEPROM_readAnything(4, readValue);
  gpsState.originLon = (double)readValue / 1000000;

  EEPROM_readAnything(8, readValue);
  gpsState.originAlt = (double)readValue / 1000000;
/** GPS END **/
  
  tft.init();
  tft.setRotation(3);
  String configString="";
  configString=readConfigFile(SD, "/touch.conf");
  Serial.println("gibts ein configFile?");
  if(configString != "noFile"){
    Serial.print("ConfigParamsFound");
    Serial.println(configString);
    String param1;
    String param2;
    String param3;
    String param4;
    String param5;

int ind1; // , locations
int ind2;
int ind3;
int ind4;
int ind5;
      ind1 = configString.indexOf(';');  //finds location of first ,
      param1 = configString.substring(0, ind1);   //captures first data String
      ind2 = configString.indexOf(';', ind1+1 );   //finds location of second ,
      param2 = configString.substring(ind1+1, ind2+1);   //captures second data String
      ind3 = configString.indexOf(';', ind2+1 );
      param3 = configString.substring(ind2+1, ind3+1);
      ind4 = configString.indexOf(';', ind3+1 );
      param4 = configString.substring(ind3+1, ind4+1); 
      ind5 = configString.indexOf(';', ind4+1 );
      param5 = configString.substring(ind4+1); 

int iParam1=param1.toInt();
int iParam2=param2.toInt();
int iParam3=param3.toInt();
int iParam4=param4.toInt();
int iParam5=param5.toInt();
    
    calibrationData[0]=iParam1;
    calibrationData[1]=iParam2;
    calibrationData[2]=iParam3;
    calibrationData[3]=iParam4;
    calibrationData[4]=iParam5;

str0 = ToString(calibrationData[0])+";"+ToString(calibrationData[1])+";"+ToString(calibrationData[2])+";"+ToString(calibrationData[3])+";"+ToString(calibrationData[4]);

Serial.println(str0);
 tft.setTouch(calibrationData);
printMainScreen();
  }else{
  calibrateTouchScreen();

  }

  


  
  iEnableButtons = 1;
}

/*
   Hilfsfunktionen um vereinfacht Speicher zu lesen und schreiben
*/
template <class T> int EEPROM_writeAnything(int ee, const T& value)
{
  const byte* p = (const byte*)(const void*)&value;
  int i;
  for (i = 0; i < sizeof(value); i++)
    EEPROM.write(ee++, *p++);
  return i;
}

template <class T> int EEPROM_readAnything(int ee, T& value)
{
  byte* p = (byte*)(void*)&value;
  int i;
  for (i = 0; i < sizeof(value); i++)
    *p++ = EEPROM.read(ee++);
  return i;
}

void printStartButton(){

  tft.setTextColor(TFT_BLACK);
  if (StartEnable){
  tft.fillRect(STARTBUTTON_X,STARTBUTTON_Y,BUTTON_W,BUTTON_H,TFT_GREEN);
  }else{
  tft.fillRect(STARTBUTTON_X,STARTBUTTON_Y,BUTTON_W,BUTTON_H,(0xBBBB));
  }
  tft.drawString("Start", STARTBUTTON_X+15, STARTBUTTON_Y+20);
}

void printStopButton(){
if(StopEnable){
    tft.fillRect(STOPBUTTON_X,STOPBUTTON_Y,BUTTON_W,BUTTON_H,TFT_RED);
}else{
    tft.fillRect(STOPBUTTON_X,STOPBUTTON_Y,BUTTON_W,BUTTON_H,(0xBBBB));
}
tft.setTextColor(TFT_BLACK);
 tft.drawString("Stop", STOPBUTTON_X+15, STOPBUTTON_Y+20);
}

void printMainScreen(){

  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  //Draw red frame
  drawFrame(5, TFT_RED);
  //Set first text
  //Set second text
  //Set last line
  tft.setTextColor(TFT_WHITE);
  //StartButton
  
  printStartButton();
   // StopButton
  printStopButton();
  //Cal Button
  tft.fillRect(CALBUTTON_X,CALBUTTON_Y,BUTTON_W,BUTTON_H,TFT_RED);
  tft.drawString("Config", CALBUTTON_X+10, CALBUTTON_Y+20);
  
  tft.setTextColor(TFT_WHITE);

  tft.drawString("Datum/Uhrzeit", 10, 10);

  tft.drawString("SatCount", 10, 30);

  tft.drawString("Speed", 10, 50);

  tft.drawString("Long", 10, 70);

  tft.drawString("Lat", 150, 70);

  tft.drawString("Status", 10, 90);

  tft.fillRect(10,108,300,3,TFT_WHITE);
  tft.fillRect(10,174,300,3,TFT_WHITE);
  
  tft.drawString("RPM", 40, 152);

  tft.drawString("BOOST", 134, 152);

  tft.drawString("Lambda", 232, 152);
  
 

}
void calibrateTouchScreen(){

      tft.fillScreen((0xFFFF));
      tft.setCursor(40, 20, 2);
      tft.setTextColor(TFT_RED, TFT_WHITE);
      tft.setTextSize(2);
      tft.println("Kalibrierung vom");
      tft.setCursor(40, 60, 2);
      tft.println("Display");
      tft.setTextColor(TFT_BLACK, TFT_WHITE);
      tft.setCursor(40, 100, 2);
      tft.println("Die angegebene Ecken");
      tft.setCursor(40, 140, 2);
      tft.println("zum kalibrieren");
      tft.setCursor(40, 180, 2);
      tft.println("beruehren");
      tft.calibrateTouch(calibrationData, TFT_GREEN, TFT_RED, 15);
      Serial.printf("Kalibration data:"); 
      str0 = ToString(calibrationData[0])+";"+ToString(calibrationData[1])+";"+ToString(calibrationData[2])+";"+ToString(calibrationData[3])+";"+ToString(calibrationData[4]);
      
// Length (with one extra character for the null terminator)
  int str_len = str0.length() + 1; 

// Prepare the character array (the buffer) 
  char char_array[str_len];

// Copy it over 
  str0.toCharArray(char_array, str_len);

  writeFile(SD, "/touch.conf", char_array);


  
  Serial.printf("Kalibration ENDE:");

printMainScreen();
}

void checkGPSInfo()
{

  static int p0 = 0;

  // GPS Koordinaten von Modul lesen
  gpsState.originLat = gps.location.lat();
  gpsState.originLon = gps.location.lng();
  gpsState.originAlt = gps.altitude.meters();


   
 //Serial.println("StartEEPROM write");
 // Aktuelle Position in nichtflüchtigen ESP32-Speicher schreiben
  long writeValue;
  writeValue = gpsState.originLat * 1000000;
  EEPROM_writeAnything(0, writeValue);
  writeValue = gpsState.originLon * 1000000;
  EEPROM_writeAnything(4, writeValue);
  writeValue = gpsState.originAlt * 1000000;
  EEPROM_writeAnything(8, writeValue);
  EEPROM.commit(); // erst mit commit() werden die Daten geschrieben

  //Serial.println("Sat's: "+String(gps.satellites.value()));
 //Serial.println("EEPROM commit");
  
    //delay(200);
    tft.fillRect(302,10,10,30,TFT_RED);
     
  

  gpsState.distMax = 0;
  gpsState.altMax = -999999;
  gpsState.spdMax = 0;
  gpsState.altMin = 999999;

  /*
   * Rohdaten von Serieller Verbndung zum GPS-Modul
   * einlesen. Die Daten werden mittels TinyGPS++ verarbeitet
   * Die Daten werden bewusst erst nach der Zuweisung der Variablen
   * gelesen, damit wir noch im nachfolgenden vereinfacht 
   * Berechnungen anstellen können.
   */
  while (SerialGPS.available() > 0) {
    tft.fillRect(302,10,10,30,TFT_BLUE);
    /*Serial.println(SerialGPS.read());*/
    gps.encode(SerialGPS.read());
   }

  /*
   * Diverse Berechnungen von Maximum und Minimum-Werten und zurückgelegter Distanz
   * Diese werden aber erst gemacht, wenn mindestens ein Fix mit 4 Satelliten vorhanden
   * ist, allenfalls wäre die Genauigkeit nicht gegeben und es würden falsche
   * Werte berechnet werden.
   */
  if (gps.satellites.value() > 4) {
    gpsState.dist = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), gpsState.originLat, gpsState.originLon);

    if (gpsState.dist > gpsState.distMax && abs(gpsState.prevDist - gpsState.dist) < 50) {
      gpsState.distMax = gpsState.dist;
    }
    gpsState.prevDist = gpsState.dist;

    if (gps.altitude.meters() > gpsState.altMax) {
      gpsState.altMax = gps.altitude.meters();
    }

    if (gps.speed.kmph() > gpsState.spdMax) {
      gpsState.spdMax = gps.speed.kmph();
    }

    if (gps.altitude.meters() < gpsState.altMin) {
      gpsState.altMin = gps.altitude.meters();
    }
  }

 


}



/*
* =================================================================
* Function:     loop   
* Returns:      void
* Description:  Main loop to let program work
* =================================================================
*/
void loop()
{
  static uint16_t color;
  tft.setTextColor(TFT_WHITE,TFT_BLACK);
  if (tft.getTouch(&pixel_x, &pixel_y) && iEnableButtons)
  {
    tft.fillRect(pixel_x-5,pixel_y-5,10,10,TFT_RED);
   // tft.fillRect(100,50,100,50,TFT_RED);
    if ((pixel_x > STARTBUTTON_X) && (pixel_x < (STARTBUTTON_X + BUTTON_W))  && StartEnable)
    {
      if ((pixel_y > STARTBUTTON_Y) && (pixel_y <= (STARTBUTTON_Y + BUTTON_H)))
      {
        Serial.println("---- StartButton pressed ----");
        if (LogFileOpen != 1){
          logSessionCounter = logSessionCounter +1;
          logLineCounter = 0;
          
          
          filename = String("/Log-")+String(logSessionCounter)+String("-")+String(logYear)+String("-")+String(logMonth)+String("-")+String(logDay)+delimiter+String(logHour)+String(logMinute)+String(logSecond)+extension;
          //filename = "/20220430-123300.csv";
          // Length (with one extra character for the null terminator)
          str_len = title.length() + 1; 

          // Prepare the character array (the buffer) 
          char char_array[str_len];

        // Copy it over 
          title.toCharArray(char_array, str_len);
          str_len = filename.length() + 1; 
          char char_array_filename[str_len];
          filename.toCharArray(char_array_filename, str_len);
          writeFile(SD, char_array_filename, char_array);
          LogFileOpen=1;
        }
        StartEnable = 0;
        StopEnable = 1;
        printStartButton();
        printStopButton();
        delay(100);
      //calibrateTouchScreen();
     
      }
    }
    if ((pixel_x > STOPBUTTON_X) && (pixel_x < (STOPBUTTON_X + BUTTON_W)) && StopEnable)
    {
      if ((pixel_y > STOPBUTTON_Y) && (pixel_y <= (STOPBUTTON_Y + BUTTON_H)))
      {
        Serial.println("---- StopButton pressed ----");
        
        LogFileOpen = 0;
        StartEnable = 1;
        StopEnable = 0;
        printStartButton();
        printStopButton();
      delay(100);
     
      }
    }
    if ((pixel_x > CALBUTTON_X) && (pixel_x < (CALBUTTON_X + BUTTON_W)))
    {
      if ((pixel_y > CALBUTTON_Y) && (pixel_y <= (CALBUTTON_Y + BUTTON_H)))
      {
        Serial.println("---- CalButton pressed ----");
        //calibrateTouchScreen(); 
        if(logFrameOnly){  
          logFrameOnly = 0;
           tft.drawString("fullLog", 110, 180);
        }else{
          logFrameOnly = 1;
          tft.drawString("frameLog", 110, 180);
        }
      delay(100);
     
      }
    }
  }

if(logLineCounter > 4){
  if(logFrameOnly){
        LogFileOpen = 0;
        StartEnable = 1;
        StopEnable = 0;
        printStartButton();
        printStopButton();
  }
}

  checkGPSInfo();
  readRoundData();

  tft.drawString(String(gps.satellites.value()), 150, 30);

if (gps.satellites.value() > 4) {
  setthetime();
  prev_set = now();
  utc = now();  // read the time in the correct format to change via the TimeChangeRules
  local = CE.toLocal(utc);
  logYear = String(gps.date.year());
  if (gps.date.month() < 10){logMonth = "0"+String(gps.date.month());}else{logMonth = String(gps.date.month());}
  if (gps.date.day() < 10){logDay = "0"+String(gps.date.day());}else{logDay = String(gps.date.day());}
  hourOffset=hour(local)-gps.time.hour();
      
  if (gps.time.hour()+hourOffset < 10){logHour = "0"+String(gps.time.hour()+hourOffset);}else{logHour = String(gps.time.hour()+hourOffset);}
  if (gps.time.minute() < 10){logMinute = "0"+String(gps.time.minute());}else{logMinute = String(gps.time.minute());}
  if (gps.time.second() < 10){logSecond = "0"+String(gps.time.second());}else{logSecond = String(gps.time.second());}




  tft.drawString(String(logDay)+"."+logMonth+"."+logYear, 190, 10);
  tft.drawString(String(logHour+":"+logMinute+":"+logSecond), 190, 30);

  tft.drawString(String(gps.speed.kmph()), 150, 50);
  tft.drawString(String(" km/h"), 230, 50);
  
  
  tft.drawString(String(gps.location.rawLng().deg)+dot+String(gps.location.rawLng().billionths).substring(0,3), 60, 70);

  tft.drawString(String(gps.location.rawLat().deg)+dot+String(gps.location.rawLat().billionths).substring(0,3), 200, 70);


  //vemsParams.EGT
  tft.drawString(String(vemsParams.RPM), 32, 122);

  tft.drawString(String(vemsParams.Analog2), 134, 122);

  tft.drawString(String(vemsParams.Lambda), 240, 122);


if(StartEnable){
    //tft.drawString("satfix", 150, 120);
    
}else{
  if(LogFileOpen){
    tft.drawString("logging...", 150, 90);
  }else{
    StartEnable = 1;
    tft.drawString("ready.....", 150, 90);
   printStartButton();
  }
   
}

}



  if(LogFileOpen){
    logStringGPS = "lat;lng;speed;satcount;date;time;";


    if (gps.satellites.value() > 4) {


    logDateString=String(logYear)+String("-")+String(logMonth)+String("-")+String(logDay);
    logTimeString=String(logHour)+String(":")+String(logMinute)+String(":")+String(logSecond);
  //      String(gps.date.value())+csvdelimiter+String(gps.time.hour()+2)+String(':')+String(gps.time.minute())+String(':')+String(gps.time.second())+String(':')+String(gps.time.centisecond())


    speedString=String(gps.speed.kmph());
    speedString.replace(".", ",");
      
    logStringGPS = String(gps.location.rawLat().deg)+dot+String(gps.location.rawLat().billionths)+csvdelimiter
    +String(gps.location.rawLng().deg)+dot+String(gps.location.rawLng().billionths)+csvdelimiter+speedString+csvdelimiter
    +String(gps.satellites.value())+csvdelimiter+logDateString+csvdelimiter+logTimeString+csvdelimiter;
    }    
    //Lambda;EGT;Analog2;RPM;Vin;
    //logStringVEMS = String(0.016*vemsParams.int_message[1]-0.016)+csvdelimiter //Lambda /linFkt
    logStringVEMS = String(vemsParams.Lambda)+csvdelimiter //Lambda / maped
    +String((vemsParams.int_message[1]*256)+vemsParams.int_message[2])+csvdelimiter // Lambda_RAW
    +String(vemsParams.EGT)+csvdelimiter // EGT
    +String(vemsParams.Analog2)+csvdelimiter // Analog_2 (BOOST)
    +String(vemsParams.RPM)+csvdelimiter // RPM
    +String(vemsParams.Vin)+csvdelimiter // V_in
    +String(vemsParams.int_message[1])+csvdelimiter+String(vemsParams.int_message[2])+csvdelimiter
    +String(vemsParams.int_message[3])+csvdelimiter+String(vemsParams.int_message[4])+csvdelimiter
    +String(vemsParams.int_message[5])+csvdelimiter+String(vemsParams.int_message[6])+csvdelimiter
    +String(vemsParams.int_message[7])+csvdelimiter+String(vemsParams.int_message[8])+csvdelimiter
    +String(vemsParams.int_message[9])+csvdelimiter+String(vemsParams.int_message[10])+csvdelimiter
    +String(vemsParams.int_message[11])+csvdelimiter+String(vemsParams.int_message[12])+csvdelimiter
    +String(vemsParams.int_message[13])+csvdelimiter+String(vemsParams.int_message[14])+csvdelimiter
    +String(vemsParams.int_message[15])+csvdelimiter+String(vemsParams.int_message[16])+csvdelimiter
    +String(vemsParams.int_message[17])+csvdelimiter+String(vemsParams.int_message[18])+csvdelimiter
    +String(vemsParams.int_message[19])+csvdelimiter+String(vemsParams.int_message[20])+csvdelimiter
    +String(vemsParams.int_message[21])+csvdelimiter+String(vemsParams.int_message[22])+csvdelimiter
    +String(vemsParams.int_message[23])+csvdelimiter+String(vemsParams.int_message[24])+csvdelimiter
    +String(vemsParams.int_message[25])+csvdelimiter+String(vemsParams.int_message[26])+csvdelimiter
    +String(vemsParams.int_message[27])+csvdelimiter+String(vemsParams.int_message[28])+csvdelimiter
    +String(vemsParams.int_message[29])+csvdelimiter+String(vemsParams.int_message[30])+csvdelimiter
    +String(vemsParams.int_message[31])+csvdelimiter+String(vemsParams.int_message[32])+csvdelimiter
    +String(vemsParams.int_message[33])+csvdelimiter+String(vemsParams.int_message[34])+csvdelimiter
    +String(vemsParams.int_message[35])+csvdelimiter+String(vemsParams.int_message[36])+csvdelimiter
    +String(vemsParams.int_message[37])+csvdelimiter+String(vemsParams.int_message[38])+csvdelimiter
    +String(vemsParams.int_message[39])+csvdelimiter+String(vemsParams.int_message[40])+csvdelimiter;
    logStringVEMS.replace(".", ",");
    logString = logStringGPS+logStringVEMS+logNewLine;
    int str_len = logString.length() + 1; 
    // Prepare the character array (the buffer) 
    char char_array[str_len];
    logString.toCharArray(char_array, str_len);
    str_len = filename.length() + 1; 
    char char_array_filename[str_len];
    filename.toCharArray(char_array_filename, str_len);
    logLineCounter = logLineCounter +1;
    appendFile(SD, char_array_filename, char_array);
  }

    
 
  
  //StartEnable = 1;
  // printStartButton();
  delay(300);
}




/*
* =================================================================
* Function:     drawFrame   
* Returns:      void
* INPUT iSize:  Size of the frame
* INPUT color:  Color of the frame
* Description:  Draw frame with given size and color
* =================================================================
*/
void drawFrame(int iSize, uint16_t color)
{
  int iCnt;
  for (iCnt = 0; iCnt <= iSize; iCnt++)
    tft.drawRect(0 + iCnt, 0 + iCnt, 320 - (iCnt * 2), 240 - (iCnt * 2), color);
}


/*
* =================================================================
* Function:     drawVerticalLine   
* Returns:      void
* INPUT x:      Posititon in x-coordinate
* INPUT color:  Color of the frame
* Description:  Draw vertical line with given color
* =================================================================
*/
void drawVerticalLine(int x, uint16_t color)
{
  int iCnt = 0;
  for(iCnt = 0; iCnt  < int(LINEWIDTH); iCnt ++)
    tft.drawLine(x+iCnt, 34, x+iCnt, 225, color);
}

/*
* =================================================================
* Function:     drawHorizontalLine   
* Returns:      void
* INPUT x:      Posititon in y-coordinate
* INPUT color:  Color of the frame
* Description:  Draw horizontal line with given color
* =================================================================
*/
void drawHorizontalLine(int y, uint16_t color)
{
  int iCnt = 0;
  for(iCnt = 0; iCnt < int(LINEWIDTH); iCnt++)
    tft.drawLine(20, y+iCnt, 246, y+iCnt, color);
}


 void readRoundData(){
   if(DEBUGMODE == 1){
  
    if ( Serial.available() ){
      vemsParams.inputString=Serial.readString();
      Serial.println("Requenst: "+ vemsParams.inputString);
      roundSerial.print("A");  
    }
  }
   
  
  roundSerial.print("A");  
  if(DEBUGMODE == 1){
    Serial.println("Send A");
  }
  delay(30);

  vemsParams.checkPort=roundSerial.available();

    while (vemsParams.counter < vemsParams.checkPort) {
      //delay(3);  //delay to allow buffer to fill 
      vemsParams.counter++;
      if (vemsParams.checkPort >0) {
        int c = roundSerial.read();
        vemsParams.int_message[vemsParams.counter]=c;
        vemsParams.readString += c;
        vemsParams.readString +=".";
      } 
    }
    vemsParams.counter=0;
    if (vemsParams.checkPort == 40){
      if (vemsParams.readString.length() >0) {
         // Serial.println(vemsParams.readString); //see what was received
          vemsParams.EGT=(vemsParams.int_message[5]*256)+vemsParams.int_message[6]-50;
          vemsParams.Vin=map(((vemsParams.int_message[15]*256)+vemsParams.int_message[16]),mapData.Vin_in_min,mapData.Vin_in_max,mapData.Vin_out_min,mapData.Vin_out_max);
          vemsParams.Analog0=(vemsParams.int_message[31]*256)+vemsParams.int_message[32];
          vemsParams.Analog0_RAW=(vemsParams.int_message[23]*256)+vemsParams.int_message[24];
          vemsParams.Analog2=(vemsParams.int_message[35]*256)+vemsParams.int_message[36];
          //vemsParams.Lambda=0.016*vemsParams.int_message[1]-0.016;
          vemsParams.Lambda=map(((vemsParams.int_message[1]*256)+vemsParams.int_message[2]),mapData.L_in_min,mapData.L_in_max,mapData.L_out_min,mapData.L_out_max);
          vemsParams.Analog2_RAW=(vemsParams.int_message[27]*256)+vemsParams.int_message[28];
          vemsParams.V_REV=(vemsParams.int_message[29]*256)+vemsParams.int_message[30];
          vemsParams.RPM=(vemsParams.int_message[7]*256)+vemsParams.int_message[8];
          
            Serial.println("EGT: " + String(vemsParams.EGT));
            Serial.println("Vin: " + String(vemsParams.Vin));
            Serial.print("Lambda: "+ String(vemsParams.Lambda));
          

          carParams.EGT = vemsParams.EGT;
          carParams.Lambda = vemsParams.Lambda;
          


      } 
    }
    if (vemsParams.checkPort == 9){
       //Serial.println("BootSequence detected.");
      }
    vemsParams.readString="";
}
