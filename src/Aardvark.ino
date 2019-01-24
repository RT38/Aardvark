
//Libraries ===================================================================
#include "adafruit-sht31.h" // include the SHT31 library from adafruit
#include "Statistic.h"      // https://github.com/RobTillaart/Arduino/blob/master/libraries/Statistic/Statistic.h
#include "QuickStats.h"
#include "I2CSoilMoistureSensor.h"
#include "Ubidots.h"
#define DEBUG 0  // 1 debug on, 0 debug off
//#define PARTICLE_KEEPALIVE 20

//Global constants ============================================================
const int samp_interval = 10;           // sampling/measurement interval in seconds, unique
const int stor_interval = 1;           // averaging/storage interval in minutes, unique
const int send_interval = 5;          // write/send interval


//Ubidots constants ============================================================
#ifndef UBIDOTS_TOKEN
#define UBIDOTS_TOKEN "BBFF-f3e6xyGaYY4dz1UXgUoFaA5I2Hw0MY"  // Put here your Ubidots TOKEN
#endif
//#define DEVICE_NAME "VarTrial_v1"

//SHT31 - Temperature, Humidity, Dewpoint ====================================
Adafruit_SHT31 sht31 = Adafruit_SHT31();  // create SHT31 instance
float h;                  // humidity, %
float t;                  // air temperature, C
float d;                  // calculated dewpoint temperature, C

// Tipping Bucket Rain Gauge (Ambient Weather WS2080) - Precip Data=============
const uint8_t rainPin = D3;      // pin for rain gauge input
const float rain_cal = 0.2794;   // gauge calibration, mm per tip
volatile unsigned long  rainlast; // timing variables
volatile int raintips = 0;       // total # of tips
volatile float raintotal_mm=0.0; // total rain in mm
float rain_hr, rain_total;
float previous_rain_total;

// Anemometer (Inspeed) - Wind Data =====================================================
const uint8_t anemomPin = D2;    // pin for cup anemometer
const float windTo_mph = 2.5;    // anemom calibration: Davis=2.25, Inspeed=2.5, Ambient Weather=1.492
volatile unsigned long PulseTimeLast = 0;       // Time stamp of the previous pulse
volatile unsigned long PulsesCumulatedTime = 0; // Time Interval since last wind speed calc
volatile unsigned long PulsesNbr = 0; // total pulses over subsample interval
float windSpeed_Hz,windSpeed_mps,windSpeed_mph,windSpeed_rpm,windGust;

//Pyranometer
int Pyrraw = 0;
int PyrValue = 0; // raw value from diode
float mV; // convert raw value from diode to mV
float r; // solar irradiance, W/m2
const float mVTo_Wm2 = 5.740; // cal coeff for photodiode pyranometer, must customize

//Debouncer ===================================================================
//volatile unsigned long LastPulseTimeInterval= 10000000;
volatile unsigned long MinPulseTimeInterval = 10000000;
volatile unsigned long LastPulseTimeInterval = 10000000;

// battery volts ==============================================================
FuelGauge fuel;
float batt;

//Soil Sensors (3) ============================================================
I2CSoilMoistureSensor sensor1 (0x20);
I2CSoilMoistureSensor sensor2 (0x25);
I2CSoilMoistureSensor sensor3 (0x30);
/*float sm1, sm2, sm3; // sm = soil moisture: capacitive: calibrated for each soil and depth
float st1, st2, st3; // st = soil temperature temperature deg C
*/

const int numreadings = 9;
const int numsensors = 3;
float sm1[numreadings];
float sm2[numreadings];
float sm3[numreadings];
float st1[numreadings];
float st2[numreadings];
float st3[numreadings];

int sm1med;
int sm2med;
int sm3med;
int st1med;
int st2med;
int st3med;

//Statistics ==================================================================
// define statistic object for each variable
Statistic tStats; // Temperature, C
Statistic hStats; // Humidity
Statistic dStats; // dewpoint, C
Statistic pStats; // Preciptiation
Statistic wStats; // wind
Statistic wgStats; // wind gust
Statistic rStats; // Pyranometer

Statistic sm1Stats; // soil 1 moisture, cap
Statistic st1Stats; // soil 1 temp, C
Statistic sm2Stats; // soil 2 moisture, cap
Statistic st2Stats; // soil 2 temp,
Statistic sm3Stats; // soil 3 moisture, cap
Statistic st3Stats; // soil 3 temp, C
Statistic sw1Stats; // soil 1 moisture: %
Statistic sw2Stats; // soil 2 moisture: %
Statistic sw3Stats; // soil 3 moisture: %

//Define Averages ==============================================================
float tavg;
float tmax;
float tmin;
float havg;
float davg;
float wavg;
float ravg;
float wgmax;

int CurrentSecond = 99;
int CurrentMinute = 99;
int CurrentHour = 99;
int previous_min=9;
int previous_hours=99;
int previous_days=999;
int last_send_time = 99;
int i=0;
int oldminute = 99;
int oldseconds = 99;
//int write_time = 99; // delete?
//int numloop = 0;
int numsend = 0;

//TCPClient client; //needed for WXUnderground not used in this sketch

const char* WEBHOOK_NAME = "B";

Ubidots ubidots("webhook", UBI_PARTICLE);
QuickStats stats;

//SETUP --------------------------------------------------------------------------------------------------------
void setup()
{
 Serial.begin(9600); // start serial port
 Wire.begin();
 Time.zone(-7); //set time zone to MST
 //Particle.keepAlive(PARTICLE_KEEPALIVE);

 /////Power Management for Solar/Battery
  PMIC pmic; //Initalize the PMIC class so you can call the Power Management functions below. 
  pmic.setChargeCurrent(0,0,1,0,0,0); //Set charging current to 1024mA (512 + 512 offset)
  pmic.setInputVoltageLimit(5080);   //Set the lowest input voltage to 5.08 volts. This keeps my 5v solar panel from operating below 4.84 volts.
 
  // start SHT31 T/RH sensor
  if (! sht31.begin(0x44)) {      //  0x44 for adafruit
    delay(1000);
    while (1) delay(1);
  }

  // rain gauge
  pinMode(rainPin, INPUT_PULLUP); // set pin mode for rain gauge input
  attachInterrupt(rainPin, rainIRQ, FALLING); // attach interrupt

  // anemometer
  pinMode(anemomPin, INPUT_PULLUP); // set pin mode for anemometer input
  attachInterrupt(anemomPin, windIRQ, FALLING); // attach interrupt
  //pyranometer
  pinMode(A0, INPUT);

  // soil sensor

  sensor1.begin(); // reset sensor
  delay(500);
  sensor2.begin(); // reset sensor
  delay(500);
  sensor3.begin(); // reset sensor
  delay(500);

    //delay(1000); // give some time to boot up
#ifdef DEBUG
  Serial.print("I2C Soil Moisture Sensor 1 Address: ");
  Serial.println(sensor1.getAddress(),HEX);
  Serial.println();
  Serial.print("I2C Soil Moisture Sensor 2 Address: ");
  Serial.println(sensor2.getAddress(),HEX);
  Serial.println();
  Serial.print("I2C Soil Moisture Sensor 3 Address: ");
  Serial.println(sensor3.getAddress(),HEX);
  Serial.println();
  Serial.println(F("sm1,sm2,sm3,st1,st2,st3"));
#endif
//clear stat data arrays
 clearStats(); // Cleaar Stats subroutine
 sample();
 add_stats();
 calc_stats();
#ifdef DEBUG
 Serial.println("setup complete");
#endif
  Particle.publish("Setup complete",NULL);
  
  //ubidots.setDebug(true); //Uncomment this line for printing debug messages
}// End Set Up


//MAIN LOOP ----------------------------------------------------------------------------------------------------
void loop()
{
 if (Time.second()%samp_interval==0) // main sampling loop, read sensors
 {
  sample();       
  add_stats();
  // end interval
if (Time.minute()%send_interval ==0 && Time.minute()!=last_send_time)
  {
  last_send_time = Time.minute();
    if (sm1med <100) // take initial soil readings
    {
      soilmoisture();
    }
      calc_stats();
      clearStats(); // clear stats after assigning values to object
      #ifdef DEBUG                               // print results during debug
      Serial.println("===============================");
      Serial.println("Average Readings");
      Serial.println("===============================");
      #endif

      delay(100);
      send_data();  // Send Soils Data
      #ifdef DEBUG                               // print results during debug
      Serial.println("===============================");
      Serial.println("send data to Ubidots");
      Serial.println("===============================");
      #endif
  }
       #ifdef DEBUG
      Serial.println("check sensors");                              // print results during debug
      Serial.println();
      Serial.print("Time: "); Serial.print(Time.hour()); Serial.print(":"), Serial.print(Time.minute()); Serial.print(":"), Serial.println(Time.second());
      Serial.println("temp \t hum \t dwpt \t wspd \t wgust \t rad \t r_mm");
      Serial.print(tavg);Serial.print("\t");
      Serial.print(havg);Serial.print("\t");
      Serial.print(davg);Serial.print("\t");
      Serial.print(wavg);Serial.print("\t");
      Serial.print(wgmax);Serial.print("\t");
      Serial.print(r);Serial.print("\t");
      Serial.println(raintips);
      Serial.println();

      Serial.println("=========================================");
      Serial.print("Median Shallow Soil Moisture: ");
      Serial.println(sm1med);
      Serial.print("Median Medium Soil Moisture: ");
      Serial.println(sm2med);
      Serial.print("Median Deep Soil Moisture: ");
      Serial.println(sm3med);
      Serial.println("=========================================");
      Serial.print("Median Shallow Temp: ");
      Serial.println(st1med);
      Serial.print("Median Medium Temp: ");
      Serial.println(st2med);
      Serial.print("Median Deep Temp: ");
      Serial.println(st3med);
      Serial.println();
      Serial.println("=========================================");
      Serial.print("Batt_volts: ");
      Serial.println(batt);
      Serial.println("=================================");
      #endif
      }
      if (Time.minute()==0 && Time.hour()!=previous_hours)
        {       // on the hour, calc hourly rain and read soil moisture
           previous_hours=Time.hour();
           soilmoisture();

           delay(50);
           rain_hr=rain_total-previous_rain_total;
           previous_rain_total=rain_total;
        }

       if (Particle.connected() == false) { //Keep Electron connected
           Particle.connect();
         }
  
}
//SUB ROUTINES ------------------------------------------------------------------------------------------------
//================================================================================

void sample()
{
   h= sht31.readHumidity();                    // measure humidity
   t=sht31.readTemperature();                 // measure temp
   d=Tdew(t,h);
   anemometer();                       // calc dewpoint with subroutine
   pyranometer();
   fuelguage();
}
void  add_stats()
{
   tStats.add(t);
   hStats.add(h);
   dStats.add(d);
   rStats.add(r);
   wStats.add(windSpeed_mph);
   wgStats.add(windGust);
   pStats.add(raintotal_mm);
}

 void calc_stats()
 {
  tavg=tStats.average();                         // calc stats
  tmax=tStats.maximum();
  tmin=tStats.minimum();
  havg=hStats.average();
  davg=dStats.average();
  wavg=wStats.average();
  wgmax=wgStats.maximum();
  ravg=rStats.average();
  rain_total=pStats.sum();
 }
//Clear Stats ==================================================================

void clearStats()
 {
   tStats.clear();                       // clear stats
   hStats.clear();
   dStats.clear();
   wStats.clear();
   wgStats.clear();
   rStats.clear();

   raintips = 0;                         // zero raingauge
   raintotal_mm = 0;
   if (Time.hour()==0 && Time.day()!=previous_days)
   { // clear 24h rain total at midnight
     previous_days=Time.day();
     pStats.clear();
   }
 }


// Calculate voltage
void fuelguage(){
  batt = fuel.getVCell();
  Serial.print("Batt_volts");
    Serial.println(batt);
}
// Calc dewpoint temperature from Tetens eq. using coeff of Murray (1967)
float Tdew(float Tair, float Rh)
{
  float X=log(Rh/100)+((17.27*Tair)/(237.3+Tair));
  float dp =(237.3*X)/(17.27-X);
  return dp;
}
// Calc Windspeed ==============================================================
void anemometer()
{
  if (PulsesCumulatedTime>4000) //if time since last wind speed calc is > 4 seconds then:
  {
    windSpeed_Hz=1000.0*PulsesNbr/PulsesCumulatedTime; // pulses per second over 5 s sampling period = (1000*# of pulses since last calc/
    windSpeed_rpm = PulsesNbr/(PulsesCumulatedTime/1E3)*60;
    windGust = 1000*windTo_mph/MinPulseTimeInterval; // Determine wind gust (mpg)
  }
  else
  {
       windSpeed_Hz=0.0;
       windSpeed_rpm=0.0;
       windGust=0.0;
  }
  windSpeed_mph=windSpeed_Hz*windTo_mph;       //
  //windSpeed_mps=windSpeed_Hz*windTo_mph*0.44704 ;

  // zero variables
  PulsesCumulatedTime = 0;
  PulsesNbr = 0;
  MinPulseTimeInterval =1000000;
  LastPulseTimeInterval =1000000;
}

// interrupt service routine for cup anemometer ===============================
void windIRQ()
{
  //noInterrupts();               // disable global interrupts
  unsigned long PulseTimeNow = millis();  // get timer
  unsigned long PulseTimeInterval = PulseTimeNow - PulseTimeLast; // s
  if (PulseTimeInterval > 10) //  if < 10 ms, Ignore switch-bounce glitches
  {
    PulseTimeLast = PulseTimeNow;// set current millis as last pulse time
    PulsesCumulatedTime = PulsesCumulatedTime + PulseTimeInterval;
    PulsesNbr++;              // count pulses

    if ( PulseTimeInterval < LastPulseTimeInterval )   // gust, fastest wind speed = shortest pulse
    {
      MinPulseTimeInterval = PulseTimeInterval;
      LastPulseTimeInterval = MinPulseTimeInterval;
    }
 // interrupts();              // Re-enable Interrupts
  }
}
//=============================================================================
// interrupt service routine for rain gauge
void rainIRQ()
{
  noInterrupts();
  unsigned long raintime = millis();                // get current time
  unsigned long raininterval = raintime - rainlast; // calculate interval between this and last tip event
  if (raininterval > 500)               // ignore switch-bounce glitches less than 500 ms
   {
     raintips++;                        // increment counter, sum # of tips
     raintotal_mm = raintips*rain_cal;  // total mm of rain
     rainlast = raintime;               // set up for next event
   }
   interrupts();                        // Re-enable Interrupts
}

//Pyranometer===================================================================
void pyranometer()
{
Pyrraw = analogRead(A0);
mV = Pyrraw*(3.3/4096)*1000; //(256 8-bit, 1024 10-bit, 4096 12-bit)
r = mVTo_Wm2*mV;
//Serial.println();
//Serial.println("====pyranometer========================");
//Serial.print("raw analog:");
//Serial.println(Pyrraw);
//Serial.print("mV:");
//Serial.println(mV);
//Serial.print("rads:");
//Serial.println(r);
//Serial.print("===============================================");

}
//Soil Moisture Data============================================================
void soilmoisture()
{
// sensor1.begin(); // reset sensor
// delay(500);
// sensor2.begin(); // reset sensor
// delay(500);
// sensor3.begin(); // reset sensor
// delay(500);

for(int k=0; k<numreadings; k++){
  // while (sensor1.isBusy()) delay(50);
  // while (sensor2.isBusy()) delay(50);
  // while (sensor2.isBusy()) delay(50);

  sm1[k] = sensor1.getCapacitance();
  sm2[k] = sensor2.getCapacitance();
  sm3[k] = sensor3.getCapacitance();

  st1[k] = sensor1.getTemperature()/(float) 10;
  st2[k] = sensor2.getTemperature()/(float) 10;
  st3[k] = sensor3.getTemperature()/(float) 10;
 }

 sm1med = stats.median(sm1, numreadings);
 sm2med = stats.median(sm2, numreadings);
 sm3med = stats.median(sm3, numreadings);

 st1med = stats.median(st1, numreadings);
 st2med = stats.median(st2, numreadings);
 st3med = stats.median(st3, numreadings);

 // sensor1.sleep();
 // sensor2.sleep();
 // sensor3.sleep();

unsigned long t = Time.now();
  ubidots.add("SM1", sm1med, NULL, t);
  ubidots.add("SM2", sm2med, NULL, t);
  ubidots.add("SM3", sm3med, NULL, t);
  
 ubidots.send(WEBHOOK_NAME, PUBLIC);  // Will send data to a device label that matches the device Id

  ubidots.add("ST1", st1med, NULL, t);
  ubidots.add("ST2", st2med, NULL, t);
  ubidots.add("ST3", st3med, NULL, t);
  bool bufferSent = false;
  bufferSent = ubidots.send(WEBHOOK_NAME, PUBLIC);  // Will send data to a device label that matches the device Id

  if(bufferSent){
    // Do something if values were sent properly
    Particle.publish("STSENT",NULL);
  }
}
//=============================================================================
void send_data()
{
  Serial.println("Begin Ubidots Send");
  //char context[25];
  //sprintf(context, "lat=%f$lng=%f",Lat, Long); //uncomment to send GPS coords from Global Defs
  unsigned long t = Time.now();
  ubidots.add("T", tavg, NULL, t);  // Change for your variable name
  ubidots.add("H", havg, NULL, t);
  ubidots.add("D", davg, NULL, t);
  ubidots.add("W", wavg, NULL, t);
  bool UbiBuffer = false;
  UbiBuffer = ubidots.send(WEBHOOK_NAME, PUBLIC);  // Will send data to a device label that matches the device Id
    if(UbiBuffer){
    // Do something if values were sent properly
    Particle.publish("5mSENT1",NULL);
  }


  ubidots.add("R", raintotal_mm, NULL, t);
  ubidots.add("B", batt, NULL, t);
  ubidots.add("WG", wgmax, NULL, t);
  ubidots.add("S", ravg, NULL, t);
  bool bufferSent = false;
  bufferSent = ubidots.send(WEBHOOK_NAME, PUBLIC);  // Will send data to a device label that matches the device Id

  if(bufferSent){
    // Do something if values were sent properly
    Particle.publish("5mSENT2",NULL);
  }
}
