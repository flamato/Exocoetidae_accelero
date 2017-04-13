/*

  This code creates a data logger which stores accelerometer values from an MPU9250 onto an EEPROM.  An ESP8266 connects to WiFi, when available, to upload the stored data to the IoT Pipe web service.
  Special thanks to Kris Winer for his awesome MPU9250 library.

  Required Libraries:
  1) IoT Pipe
  2) extEEPROM

  Hardware setup:
  MPU9250 Breakout --------- Adafruit Huzzah
  VDD ---------------------- 3.3V
  VDDI --------------------- 3.3V
  SDA ----------------------- Pin4
  SCL ----------------------- Pin5
  GND ---------------------- GND

  EEPROM
  Vcc to 3.3V
  GND to GND
  AO, A1, A2 to GND  (on 24LC256 this gives an i2c slave address as 1010000 which is 0x50)
  SDA/SCL to Pin4 and Pin5 of Adafruit Huzzah, respectively
*/
#include <ESP8266WiFi.h>
//#include <PubSubClient.h>
#include "src/quaternionFilters.h"
#include "src/MPU9250.h"
#include "src/IMUResult.h"
#include "src/IMUWriter.h"

const char* ssid = "Artilect_Fablab";
const char* password = "moto jaune";

///////////////////////////////////////////////////////////////////
//Debug information
///////////////////////////////////////////////////////////////////
#define serialDebug true  // Set to true to get Serial output for debugging
#define baudRate 115200

///////////////////////////////////////////////////////////////////
//Determines how often we sample and send data
///////////////////////////////////////////////////////////////////
#define samplingRateInMillis 1000

///////////////////////////////////////////////////////////////////
//Setup for the Accelerometer
///////////////////////////////////////////////////////////////////
#define declination 15.93  //http://www.ngdc.noaa.gov/geomag-web/#declination . This is the declinarion in the easterly direction in degrees.  
#define calibrateMagnetometer false  //Setting requires requires you to move device in figure 8 pattern when prompted over serial port.  Typically, you do this once, then manually provide the calibration values moving forward.
MPU9250 myIMU;
IMUWriter writer(kbits_256, 1, 64, 0x50);  //These are the arguments needed for extEEPROM library.  See their documentation at https://github.com/JChristensen/extEEPROM
IMUResult magResult, accResult, gyroResult, orientResult;


///////////////////////////////////////////////////////////////////
//Wifi object, MQTT object, and IoT Pipe object
///////////////////////////////////////////////////////////////////
WiFiClient espClient;
//IotPipe iotpipe(deviceId);

bool reconnect();

void setup()
{

  Serial.begin(baudRate);
  Serial.setDebugOutput(true); //Used for more verbose wifi debugging

  //Start IMU.  Assumes default SDA and SCL pins 4,5 respectively.
  myIMU.begin();


  //This tests communication between the accelerometer and the ESP8266.  Dont continue until we get a successful reading.
  //It is expected that the WHO_AM_I_MPU9250 register should return a value of 0x71.
  //If it fails to do so try the following:
  //1) Turn power off to the ESP8266 and restart.  Try this a few times first.  It seems to resolve the issue most of the time.  If this fails, then proceed to the followingn steps.
  //2) Go to src/MPU9250.h and change the value of ADO from 0 to 1
  //3) Ensure your i2c lines are 3.3V and that you haven't mixed up SDA and SCL
  //4) Run an i2c scanner program (google it) and see what i2c address the MPU9250 is on.  Verify your value of ADO in src/MPU9250.h is correct.
  //5) Some models apparently expect a hex value of 0x73 and not 0x71.  If that is the case, either remove the below check or change the value fro 0x71 to 0x73.
  byte c;
  do
  {
    c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
    if (c != 0x71)
    {
      Serial.println("Failed to communicate with MPU9250");
      Serial.print("WHO_AM_I returned ");
      Serial.println(c, HEX);
      delay(500);
    }
  } while (c != 0x71);

  Serial.println("Successfully communicated with MPU9250");


  // Calibrate gyro and accelerometers, load biases in bias registers, then initialize MPU.
  myIMU.calibrate();  
  myIMU.init();
  if (calibrateMagnetometer)
    myIMU.magCalibrate();
  else
    myIMU.setMagCalibrationManually(-166, 16, 663);    //Set manually with the results of magCalibrate() if you don't want to calibrate at each device bootup.
                                                       //Note that values will change as seasons change and as you move around globe.  These values are for zip code 98103 in the fall.

  Serial.println("Accelerometer ready");

  WiFi.begin(ssid,password);
  

  accResult.setName("acc");
  gyroResult.setName("gyro");
  magResult.setName("mag");
  orientResult.setName("orien");
}

uint32_t lastSample = 0;
void loop()
{

	// If intPin goes high, all data registers have new data
	// On interrupt, check if data ready interrupt
	if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
	{
		myIMU.readAccelData(&accResult);
		myIMU.readGyroData(&gyroResult);
		myIMU.readMagData(&magResult);
	}

	// Must be called before updating quaternions!
	myIMU.updateTime();
	MahonyQuaternionUpdate(&accResult, &gyroResult, &magResult, myIMU.deltat);
	readOrientation(&orientResult, declination);

	if (millis() - lastSample > samplingRateInMillis)
	{
    lastSample = millis();
		
		if (serialDebug)
		{
			accResult.printResult();
			gyroResult.printResult();
			magResult.printResult();
			orientResult.printResult();
		}

	}  
}

