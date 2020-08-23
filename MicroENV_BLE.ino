#include <Arduino_HTS221.h>
#include <ArduinoBLE.h>
#include <Arduino_LPS22HB.h>
#include <Arduino_LSM9DS1.h>
#include <sps30.h>

#define SP30_COMMS Serial1  //(S)  {All lines with an S are used for the SPS30 and can be removed if not using an SPS30}

SPS30 sps30;

float Po = 1013.25; // average pressure at mean sea-level (MSL) in the International Standard Atmosphere (ISA) is 1013.25 hPa
float P, T;
float To = 273.15;
float k = 0.0065;
void ErrtoMess(char *mess, uint8_t r);
void Errorloop(char *mess, uint8_t r);
void GetDeviceInfo();
bool read_all();


BLEService sensorService("181A");  // Environmental_Sensor-Service
BLELongCharacteristic sensorLevelChar1("2A6E", BLERead | BLENotify);   // Temperature-Characteristic
BLELongCharacteristic sensorLevelChar2("2A6F", BLERead | BLENotify);   // Humidity-Characteristic
BLELongCharacteristic sensorLevelChar3("2A6C", BLERead | BLENotify);    // Elevation-Characteristic
BLELongCharacteristic sensorLevelChar4("2A6D", BLERead | BLENotify); //Pressure Characteristic
BLELongCharacteristic sensorLevelChar5("2A75", BLERead | BLENotify); //Pollen concentration Characteristic  (S)


long previousMillis = 0;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);   // (S)
  pinMode(LEDB,OUTPUT);
  digitalWrite(LEDB, HIGH);

  while (!Serial){
    
  }


if (!HTS.begin()) {
Serial.println("Failed to initialize humidity temperature sensor!");
while (1);
}

if (!BARO.begin()) {
    Serial.println("Failed to initialize pressure sensor!");
    while (1);
}
  
if (!BLE.begin()) {
    Serial.println("BLE init. failed!");
    while (1);
}

if (!sps30.begin(SP30_COMMS)){      // (S)
  Serial.println("'Couldn't initialise SPS30");   // (S)
  
}        // (S)

sps30.EnableDebugging(0);   // (S)

  BLE.setLocalName("Name to be displayed on BLE Connections");
  BLE.addService(sensorService);
  BLE.setAdvertisedService(sensorService);
  sensorService.addCharacteristic(sensorLevelChar1);
  sensorLevelChar1.writeValue(0);

  sensorService.addCharacteristic(sensorLevelChar2);
  BLE.addService(sensorService);
  sensorLevelChar2.writeValue(0);

  sensorService.addCharacteristic(sensorLevelChar3);
  BLE.addService(sensorService);
  sensorLevelChar3.writeValue(0);

  sensorService.addCharacteristic(sensorLevelChar4);
  BLE.addService(sensorService);
  sensorLevelChar4.writeValue(0);

  sensorService.addCharacteristic(sensorLevelChar5);    // (S)
  BLE.addService(sensorService);    // (S)
  sensorLevelChar5.writeValue(0);      // (S)

  if (! sps30.begin(SP30_COMMS))   // (S)
    Errorloop((char *) "could not initialize communication channel.", 0); // (S)

  // check for SPS30 connection
  if (! sps30.probe()) Errorloop((char *) "could not probe / connect with SPS30.", 0);  // (S)
  else  Serial.println(F("Detected SPS30."));  // (S)

  SetAutoClean();  // (S)

  // reset SPS30 connection
  if (! sps30.reset()) Errorloop((char *) "could not reset.", 0); // (S)

  // read device info
  GetDeviceInfo();  // (S)

  // start measurement
  if (sps30.start()) Serial.println(F("Set to measurement mode"));  // (S)
  else Errorloop((char *) "Could NOT start measurement", 0);  // (S)

  // start advertising
  BLE.advertise();
  Serial.print("MicroENV BLE started, waiting for connections...");
}

void loop() {
  // waiting for a BLE central device.
  BLEDevice central = BLE.central();
  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    while (central.connected()) {
      long currentMillis = millis();
      if (currentMillis - previousMillis >= 500) {
        previousMillis = currentMillis;
        updateSensorLevel1(); 
        updateSensorLevel2();
        updateSensorLevel3();
        updateSensorLevel4();   
        read_all();   // (S)
        digitalWrite(LEDB, LOW); // indicate connection
      }
    }
    digitalWrite(LEDB, HIGH); // indicate connection,HIGH);
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}

void updateSensorLevel1() {
  float temperature = HTS.readTemperature();
  Serial.print("Temperature = ");
  Serial.print(temperature);
  Serial.println("*C");
  sensorLevelChar1.writeValue(temperature * 100);
}

void updateSensorLevel2() {
  float humidity = HTS.readHumidity();
  Serial.print("Humidity = ");
  Serial.print(humidity);
  Serial.println(" %");
  sensorLevelChar2.writeValue(humidity * 100);
}

void updateSensorLevel3() {
  float pressure = BARO.readPressure();
  P = pressure * 10; // in hectoPascals. // [1hPa = 10kPa]
  float fP = (Po / P);     // pressure-factor.
  float x = (1 / 5.257);   // power-constant.
  double z = pow(fP, x);   // exponential pressure-grad.
  float height = (((z - 1) * (T + To)) / k);

  Serial.print("Altitude = ");
  Serial.print(height);
  Serial.println(" metres");
  sensorLevelChar3.writeValue(height * 100);
}

void updateSensorLevel4() {
  float pressure = BARO.readPressure();
  Serial.print("Pressure = ");
  Serial.print(pressure);
  Serial.println("kPa");
  sensorLevelChar4.writeValue(pressure * 1000);
}

void SetAutoClean()  // (S)  This full void is used for the SPS30 only
{
  uint32_t interval;
  uint8_t ret;

  // try to get interval
  ret = sps30.GetAutoCleanInt(&interval);
  if (ret == ERR_OK) {
    Serial.print(F("Current Auto Clean interval: "));
    Serial.print(interval);
    Serial.println(F(" seconds"));
  }
  else {
    ErrtoMess((char *) "could not get clean interval.", ret);
  }
}


    
bool read_all()    // (S)  This full void is used for the SPS30 only
{
  static bool header = true;
  uint8_t ret, error_cnt = 0;
  struct sps_values val;

  // loop to get data
  do {

    ret = sps30.GetValues(&val);

    // data might not have been ready
    if (ret == ERR_DATALENGTH){

        if (error_cnt++ > 3) {
          ErrtoMess((char *) "Error during reading values: ",ret);
          return(false);
        }
       
    }

    // if other error
    else if(ret != ERR_OK) {
      ErrtoMess((char *) "Error during reading values: ",ret);
      return(false);
    }

  } while (ret != ERR_OK);

  // only print header first time
  if (header) {
    Serial.println(F("-------------Mass -----------    ------------- Number --------------   -Average-"));
    Serial.println(F("     Concentration [μg/m3]             Concentration [#/cm3]             [μm]"));
    Serial.println(F("P1.0\tP2.5\tP4.0\tP10\tP0.5\tP1.0\tP2.5\tP4.0\tP10\tPartSize\n"));
    header = true;
  }

  Serial.print(val.MassPM1);
  Serial.print(F("\t"));
  Serial.print(val.MassPM2);
  Serial.print(F("\t"));
  Serial.print(val.MassPM4);
  Serial.print(F("\t"));
  Serial.print(val.MassPM10);
  Serial.print(F("\t"));
  Serial.print(val.NumPM0);
  Serial.print(F("\t"));
  Serial.print(val.NumPM1);
  Serial.print(F("\t"));
  Serial.print(val.NumPM2);
  Serial.print(F("\t"));
  Serial.print(val.NumPM4);
  Serial.print(F("\t"));
  Serial.print(val.NumPM10);
  Serial.print(F("\t"));
  Serial.print(val.PartSize);
  Serial.print(F("\n"));
  sensorLevelChar5.writeValue(val.NumPM2);
  return(true);
}

void Errorloop(char *mess, uint8_t r)   // (S)  This full void is used for the SPS30 only
{
  if (r) ErrtoMess(mess, r);
  else Serial.println(mess);
  Serial.println(F("Program on hold"));
  for(;;) delay(100);
}

/**
 *  @brief : display error message
 *  @param mess : message to display
 *  @param r : error code
 *
 */
void ErrtoMess(char *mess, uint8_t r)   // (S)  This full void is used for the SPS30 only
{
  char buf[80];

  Serial.print(mess);

  sps30.GetErrDescription(r, buf, 80);
  Serial.println(buf);
}

void GetDeviceInfo()   // (S)  This full void is used for the SPS30 only
{
  char buf[32];
  uint8_t ret;
  SPS30_version v;

  //try to read serial number
  ret = sps30.GetSerialNumber(buf, 32);
  if (ret == ERR_OK) {
    Serial.print(F("Serial number : "));
    if(strlen(buf) > 0)  Serial.println(buf);
    else Serial.println(F("not available"));
  }
  else
    ErrtoMess((char *) "could not get serial number", ret);

  // try to get product name
  ret = sps30.GetProductName(buf, 32);
  if (ret == ERR_OK)  {
    Serial.print(F("Product name  : "));

    if(strlen(buf) > 0)  Serial.println(buf);
    else Serial.println(F("not available"));
  }
  else
    ErrtoMess((char *) "could not get product name.", ret);

  // try to get version info
  ret = sps30.GetVersion(&v);
  if (ret != ERR_OK) {
    Serial.println(F("Can not read version info"));
    return;
  }

  Serial.print(F("Firmware level: "));  Serial.print(v.major);
  Serial.print("."); Serial.println(v.minor);

  if (SP30_COMMS != I2C_COMMS) {
    Serial.print(F("Hardware level: ")); Serial.println(v.HW_version);

    Serial.print(F("SHDLC protocol: ")); Serial.print(v.SHDLC_major);
    Serial.print("."); Serial.println(v.SHDLC_minor);
  }

  Serial.print(F("Library level : "));  Serial.print(v.DRV_major);
  Serial.print(".");  Serial.println(v.DRV_minor);
}
