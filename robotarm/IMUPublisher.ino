#include <Wire.h>
#include <ESP8266WiFi.h> // Or WiFiNINA.h, ESP8266WiFi.h etc.
#include <WiFiUdp.h>

#define BMI160_I2C_ADDRESS 0x68
// #define ACCEL_SENSITIVITY 16384.0 // Sensitivity for ±2g in LSB/g (adjust based on your configuration)
#define ACCEL_SENSITIVITY 1000.0


int status = WL_IDLE_STATUS;
char ssid[] = "squid"; //  your network SSID (name)
char pass[] = "securepassword";   
unsigned int localPort = 2390;   
IPAddress remote_IP(192,168,4,1);
bool remotely_pinged = false;
unsigned int remotePort = 2390;

char packetBuffer[255]; //buffer to hold incoming packet

char outputBuffer[1024];
char  ReplyBuffer[] = "acknowledged";       // a string to send back
WiFiUDP wifiudp;



bool wifiskip;
bool calibrating = true;
float xdrift = 0;
float ydrift = 0;
float zdrift = 0;

float xsum =0;
float ysum = 0;
float zsum = 0;
unsigned long lastTime = micros();


#define TOTAL_CALIBRATION_STEPS 2000

int calibrationSteps = TOTAL_CALIBRATION_STEPS;

void setup() {
  Serial.begin(115200); // Initialize Serial communication
  Wire.begin();         // Initialize I2C communication
  WiFi.begin(ssid, pass);
  wifiudp.begin(localPort);
  
  wifiskip = false;
  while (WiFi.status() != WL_CONNECTED && !wifiskip) {

    delay(500);
    Serial.println("Connecting...");
  }
  Serial.println("Connected to wifi");
  Serial.println();
  Serial.println(WiFi.localIP());
  delay(3000);
  Serial.println("\nStarting connection to server...");

  // if you get a connection, report back via serial:

  // Initialize BMI160 accelerometer
  // Wire.beginTransmission(BMI160_I2C_ADDRESS);
  // Wire.write(0x7E); // Command register
  // Wire.write(0x11); // Set accelerometer to normal mode
  // Wire.endTransmission();
  // delay(100);

  // Initialize Gyroscope
  Wire.beginTransmission(BMI160_I2C_ADDRESS);
  Wire.write(0x7E);
  Wire.write(0x15); // set gyro to normal mode
  Wire.endTransmission();
  // Perform accelerometer auto-calibration
  delay(100);

  autoCalibrateAccelerometer();

  Serial.println("BMI160 Initialized and Calibrated");
}

void writeUDP(char* message){
  wifiudp.beginPacket(remote_IP, remotePort);
  wifiudp.write(message);
  wifiudp.endPacket();
}


void readUDP(){
  int packetlength = wifiudp.parsePacket();
  if(packetlength){
  Serial.println(packetlength);
     remote_IP = wifiudp.remoteIP();
    int len = wifiudp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = '\0'; // Null-terminate the received data to treat as a string
    }
    Serial.println(packetBuffer); // Print the contents
      remotely_pinged = true;

  }
  
}


void loop() {
  int16_t ax, ay, az;
  bool gyro_ready = false;

  while(!gyro_ready){
    Wire.beginTransmission(BMI160_I2C_ADDRESS);
    Wire.write(0x1B);
    Wire.endTransmission();
    Wire.requestFrom(BMI160_I2C_ADDRESS, 1);
    if(Wire.available()==1){
      int8_t data = Wire.read();
      gyro_ready = (0b01000000 & data)>>6;
      // delay(1);
      // Serial.println(data, BIN);
      // Serial.println();

    }
    // Serial.println("waiting");
    //   delay(1);

    // delay(100);
  }

  // Read accelerometer data
  Wire.beginTransmission(BMI160_I2C_ADDRESS);
  Wire.write(0x0C); // Start register for gryo data
  // Wire.write(0x12); // Start register for accelerometer data

  Wire.endTransmission(false);
  Wire.requestFrom(BMI160_I2C_ADDRESS, 6);

  if (Wire.available() == 6) {
    ax = (Wire.read() | (Wire.read() << 8));
    ay = (Wire.read() | (Wire.read() << 8));
    az = (Wire.read() | (Wire.read() << 8));
  }

  // // Convert raw accelerometer values to g
  float ax_g = (ax / ACCEL_SENSITIVITY)-xdrift;
  float ay_g = (ay / ACCEL_SENSITIVITY)-ydrift;
  float az_g = (az / ACCEL_SENSITIVITY)-zdrift;

  unsigned long dt = micros()-lastTime;
  lastTime = micros();
  // float ax_g = (ax / ACCEL_SENSITIVITY);
  // float ay_g = (ay / ACCEL_SENSITIVITY);
  // float az_g = (az / ACCEL_SENSITIVITY);

  xsum += ax_g;
  ysum += ay_g;
  zsum += az_g;

  if(calibrating){
    if (calibrationSteps>0){
      calibrationSteps--;
    }
    else{
      calibrating = false;
      xdrift = xsum/TOTAL_CALIBRATION_STEPS;
      ydrift = ysum/TOTAL_CALIBRATION_STEPS;
      zdrift = zsum/TOTAL_CALIBRATION_STEPS;

      xsum = 0;
      ysum = 0;
      zsum = 0;
    }
  }



  // // Calculate tilt angles (pitch and roll) in degrees
  // float pitch = atan2(ay_g, sqrt(ax_g * ax_g + az_g * az_g)) * 180.0 / PI;
  // float roll = atan2(-ax_g, az_g) * 180.0 / PI;

  // // Print tilt angles
  // Serial.print("x: ");
  // Serial.println(xsum, 5);
  // Serial.print("°, Roll: ");
  // Serial.print(roll, 2);
  // Serial.println("°");

  // if(!wifiskip){
    
    // if(remotely_pinged)
    // {
      sprintf(outputBuffer, "X:%f,Y:%f,Z:%f,dt:%ld\n", ax_g, ay_g, az_g, dt);
      writeUDP(outputBuffer);
    // }
    // else{
      
    //   readUDP();
    // }
  // }
  // delay(100);
}

void autoCalibrateAccelerometer() {

  // int8_t registerValues = 0;
  // Wire.beginTransmission(BMI160_I2C_ADDRESS);
  // Wire.write(0x77);
  // Wire.endTransmission();
  // Wire.requestFrom(BMI160_I2C_ADDRESS, 1);
  // if(Wire.available() == 1){
  //   registerValues = Wire.read();
  // }


  // Wire.beginTransmission(BMI160_I2C_ADDRESS);
  // Wire.write(0x77); // Offset config
  // Wire.write(0b10000000|registerValues); // enable gyro offset compensation
  // Wire.endTransmission();
  // delay(100);


  Wire.beginTransmission(BMI160_I2C_ADDRESS);
  Wire.write(0x69); // FOC config address
  Wire.write(0b100); // enable FOC on Gyro
  Wire.endTransmission();
  delay(250);// max time it can take to complete FOC

  // Configure accelerometer for auto-calibration
  Wire.beginTransmission(BMI160_I2C_ADDRESS);
  Wire.write(0x7E); // Command register
  // Wire.write(0x37); // Start accelerometer offset calibration
  Wire.write(0x03); // Start accelerometer offset calibration

  Wire.endTransmission();
  delay(100);



  // Wait for calibration to complete
  delay(1000);
  Serial.println("Accelerometer Auto-Calibration Complete");
}
