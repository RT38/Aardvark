# Aardvark
Weather Station Project

## Platform
Boron

### Revision
Revised from Wx_E_V3.1 for the Boron
Revized from Jay Ham Particle_RH_T_Rain_U_NTP_Stats_WU_Thingspeak_V5)

#### Purpose
Monitors Primary Weather Data + Soil Moisture and Temperature
Sends Data to Ubidots
Measured Variables:
Ambient Temp and RH, Dewpoint, Rain Precip, Solar irradiance
Soil moisture and temp at 3 locations (or depths 0-5, 20-25 & 60-65 cm), battery voltage

##### Version 3.1
Updated 28NOV2018
Move averaging of values to the send time instead of its own 1 minute time which was dumb because it was over writing the variables every minute

Added Battery Check SOC code from Particle to avoid brown outs. Set to 10% and 60 minute wake up timer.

Updated 12/24/2017
 Created clear stats subroutine
 Change sm constant from unsigned int ==> float
 change st constant from int ==> float

 // DATA===============================
// Sample every 10 seconds
// Precip: Hr  total, 24 hr Cummulative
// Soil Moisture: 5 min Average: Capacitive -> Volumetric
// Soil Temperature: 5 Min average
// Soil Probe Depths: 5-10, 20-25, 60-65cm
// 5 min wind avg speed
// 5 min wind Gust
// Roger Tyler, 05May2018 (Cinco de Mayo)
//Adapted From:
// SD Card test for Feather M0
// Reads battery volt on pin A7 via 2x volt divider
// Read Ultimate GPS for time/location
// writes data to sd card with formatted time stamp
// Sources
// https://learn.adafruit.com/adafruit-feather-m0-adalogger/using-the-sd-card
// https://github.com/cavemoa/Feather-M0-Adalogger
// J Ham, 25March2018

////////////Roger Tyler, Sept 18, 2018

###### PINS
 Rain = D2
 Wind = D3
 SDL = D0
 SCK = D1
 Pyr = A0