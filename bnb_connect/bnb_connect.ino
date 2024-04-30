/*----------------------------------------------------------------------
Data Read Demo

 Created  12 Dec 2016
 Modified 10 Mar 2019 for Settimino 2.0.0
 by Davide Nardella
 
------------------------------------------------------------------------
This demo shows how to read data from the PLC.
A DB with at least 1024 byte into the PLC is needed.
Specify its number into DBNum variable

- Both small and large data transfer are performed (see DO_IT_SMALL)
- During the loop, try to disconnect the ethernet cable.
  The system will report the error and will reconnect automatically
  when you re-plug the cable.
- For safety, this demo *doesn't write* data into the PLC, try
  yourself to change ReadArea with WriteArea.
- This demo uses ConnectTo() with Rack=0 and Slot=2 (S7300)
  - If you want to connect to S71200/S71500 change them to Rack=0, Slot=0.
  - If you want to connect to S7400 see your hardware configuration.
  - If you want to work with a LOGO 0BA7 or S7200 please refer to the
    documentation and change 
    Client.ConnectTo(<IP>, <Rack>, <Slot>);
    with the couple
    Client.SetConnectionParams(<IP>, <LocalTSAP>, <Remote TSAP>);
    Client.Connect();
    
----------------------------------------------------------------------*/
#include "Platform.h"
#include "Settimino.h"
#include <Adafruit_VL53L0X.h>
#include <Wire.h>
#include "sens_dist_ballAndBeam.h"
// #include <ESP32Servo.h>

TaskHandle_t TASK_1;
TaskHandle_t TASK_2;

//---------------------------------- GLOBAIS -----------------------------------
uint8_t varA = 0;
uint8_t varB = 0;
uint16_t myVar = 0;

int8_t sens_dist_byte_1 = 0, sens_dist_byte_2 = 0;
int16_t sens_dist_val = 0;

Adafruit_VL53L0X sensor1;
Adafruit_VL53L0X sensor2;
// VL53L0X_SENSE_HIGH_ACCURACY or VL53L0X_SENSE_DEFAULT
sensorList_t sensors[] = {
  { &sensor1, &SENSOR1_WIRE, 0x30, 33, 35,
    Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0 },
  { &sensor2, &SENSOR2_WIRE, 0x25, 32, 35,
    Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0 }
};

const int COUNT_SENSORS = 2;
uint16_t ranges_mm[COUNT_SENSORS];
bool timeouts[COUNT_SENSORS];
uint32_t stop_times[COUNT_SENSORS];

// #define servoPin 26 // ESP32 pin GPIO26 connected to servo motor
// Servo myservo;

// Uncomment next line to perform small and fast data access
#define DO_IT_SMALL

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {
  0x90, 0xA2, 0xDA, 0x0F, 0x08, 0xE1
};

// =========================== PLC 6 LAB ===============================
IPAddress Local(192, 168, 0, 30);  // Local Address
IPAddress PLC(192, 168, 0, 6);     // PLC Address

// Following constants are needed if you are connecting via WIFI
// The ssid is the name of my WIFI network (the password obviously is wrong)
char ssid[] = "TP-LINK_241751";  // Your network SSID (name)
char pass[] = "16847224";      // Your network password (if any)
IPAddress Gateway(192, 168, 0, 1);
IPAddress Subnet(255, 255, 255, 0);

// =========================== PLC SIM Casa ===============================
// IPAddress Local(192,168,0,30); // Local Address
// IPAddress PLC(192,168,0,1);   // PLC Address

// Following constants are needed if you are connecting via WIFI
// The ssid is the name of my WIFI network (the password obviously is wrong)
// char ssid[] = "VIVOFIBRA-7700";    // Your network SSID (name)
// char pass[] = "4F3122D1";  // Your network password (if any)
// IPAddress Gateway(192, 168, 0, 30);
// IPAddress Subnet(255, 255, 255, 0);

int DBNum = 101;  // This DB must be present in your PLC
int DBNum_read = 100;
uint8_t Buffer[1024] = { 0 };
byte Buffer_read[1024] = { 0 };
S7Client Client;
S7Client Client_2;

unsigned long Elapsed = 0;  // To calc the execution time
unsigned long read_interval = 0;
unsigned long write_interval = 0;

int Size, Result;
void* Target;

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  // Allow allocation of all timers
  // ESP32PWM::allocateTimer(0);
  // ESP32PWM::allocateTimer(1);
  // ESP32PWM::allocateTimer(2);
  // ESP32PWM::allocateTimer(3);
  // myservo.setPeriodHertz(50);    // standard 50 hz servo
  // myservo.attach(servoPin, 1000, 2000); // attaches the servo on pin 18 to the servo object
  // using default min/max of 1000us and 2000us
  // different servos may require different min/max settings
  // for an accurate 0 to 180 sweep

  Wire.begin();
  #if defined(WIRE_IMPLEMENT_WIRE1)
    Wire1.begin();
  #endif
    // wait until serial port opens ... For 5 seconds max
    while (!Serial && millis() < 5000)
      ;

    pinMode(13, OUTPUT);

    // initialize all of the pins.
    Serial.println(F("VL53LOX_multi start, initialize IO pins"));
    for (int i = 0; i < COUNT_SENSORS; i++) {
      pinMode(sensors[i].shutdown_pin, OUTPUT);
      digitalWrite(sensors[i].shutdown_pin, LOW);

      if (sensors[i].interrupt_pin >= 0)
        pinMode(sensors[i].interrupt_pin, INPUT_PULLUP);
    }
    Serial.println(F("Starting..."));
    Initialize_sensors();

  #ifdef S7WIFI
    //--------------------------------------------- ESP8266 Initialization
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, pass);
    WiFi.config(Local, Gateway, Subnet);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.print("Local IP address : ");
    Serial.println(WiFi.localIP());
  #else
    //--------------------------------Wired Ethernet Shield Initialization
    // Start the Ethernet Library
    EthernetInit(mac, Local);
    // Setup Time, someone said me to leave 2000 because some
    // rubbish compatible boards are a bit deaf.
    delay(2000);
    Serial.println("");
    Serial.println("Cable connected");
    Serial.print("Local IP address : ");
    Serial.println(Ethernet.localIP());
  #endif

  xTaskCreatePinnedToCore(task_write_in_clp,
                          "TASK_1",
                          10000,
                          NULL,
                          1,
                          &TASK_1,
                          1);

  xTaskCreatePinnedToCore(task_read_clp,
                          "TASK_2",
                          10000,
                          NULL,
                          1,
                          &TASK_2,
                          1);
}

void loop() {
  //MarkTime();

  //task_write_in_clp();
  // tesk_read_clp();

  //ShowTime();
  disableCore0WDT();
  disableCore1WDT();
}

void task_read_clp(void* pvParameters) {
  unsigned long time=0;
  do {
    read_interval = millis();
    #ifdef DO_IT_SMALL
        Size = 22;
        Target = NULL;  // Uses the internal Buffer (PDU.DATA[])
    #else
        Size = 1024;
        Target = &Buffer_read;  // Uses a larger buffer
    #endif

    while (!Client_2.Connected) {
      if (!Connect(&Client_2))
        delay(500);
    }

    Result = Client_2.ReadArea(S7AreaDB,    // We are requesting DB access
                               DBNum_read,  // DB Number
                               0,           // Start from byte N.0
                               Size,        // We need "Size" bytes
                               Target);     // Put them into our target (Buffer or PDU)

    if (Result == 0) {
      //ShowTime();
      Dump(Target, Size);
    } 
    else {
      CheckError(Result, &Client_2);
    }
    ShowTime(read_interval);
    Serial.println();
    //delay(30);
  } while(true);
}

void task_write_in_clp(void* pvParameters) {
  unsigned long time=0;
  do {
    //MarkTime(&time);
    //write_interval = millis();
    // =============================== READ SENSOR DIST =========================
    while (!Client.Connected) {
      if (!Connect(&Client))
        delay(500);
    }

    read_sensors();
    sens_dist_val = ranges_mm[0];

    // Serial.print("Var_3 base: ");printOut1(sens_dist_val,15);
    // Serial.println();

    sens_dist_byte_1 = sens_dist_val;
    sens_dist_byte_2 = sens_dist_val >>= 8;

    // Serial.print("Var_1: ");printOut1(sens_dist_byte_1);
    // Serial.println();
    // Serial.print("Var_2: ");printOut1(sens_dist_byte_2);
    // Serial.println();

    Buffer[0] = sens_dist_byte_2;
    Buffer[1] = sens_dist_byte_1;

    Client.WriteArea(S7AreaDB,  // We are requesting DB access
                     DBNum,     // DB Number
                     0,         // Start from byte N.0
                     2,         // We need "Size" bytes
                     &Buffer);  // Put them into our target (Buffer or PDU)
    //Client.Disconnect();
    //ShowTime(write_interval);
  } while (true);
}

unsigned long GetTimeDif(unsigned long compare_time) {
  return millis() - compare_time;
}

void MarkTime(unsigned long Elapsed_local) {
  Elapsed_local = millis();
}

void ShowTime(unsigned long Elapsed_local) {
  // Calcs the time
  unsigned long time_dif = millis() - Elapsed_local;
  Serial.print("     Job time (ms) : ");
  Serial.print(time_dif);
  Serial.print("         ");
}

void printOut1(int c, int val_bits) {
  for (int bits = val_bits; bits > -1; bits--) {
    // Compara bits 7-0 no byte
    if (c & (1 << bits)) {
      Serial.print("1");
    } else {
      Serial.print("0");
    }
  }
}

// Connects to the PLC
bool Connect(S7Client* Client_local) {
  int Result = Client_local->ConnectTo(PLC,
                                       0,   // Rack (see the doc.)
                                       1);  // Slot (see the doc.)
  Serial.print("Connecting to ");
  Serial.println(PLC);
  if (Result == 0) {
    Serial.print("Connected ! PDU Length = ");
    Serial.println(Client_local->GetPDULength());
  } else
    Serial.println("Connection error");
  return Result == 0;
}

// Dumps a buffer (a very rough routine)
void Dump(void* Buffer, int Length) {
  //int i, cnt=0;
  pbyte buf;

  if (Buffer != NULL)
    buf = pbyte(Buffer);
  else
    buf = pbyte(&PDU.DATA[0]);

  // Serial.print("[ Dumping ");Serial.print(Length);
  // Serial.println(" bytes ]===========================");
  // for (i=0; i<Length; i++)
  // {
  //   cnt++;
  //   if (buf[i]<0x10)
  //     Serial.print("0");
  //   Serial.print(buf[i], HEX);
  //   Serial.print(" ");
  //   if (cnt==16)
  //   {
  //     cnt=0;
  //     Serial.println();
  //   }
  // }
  //Serial.println("===============================================");
  varA = buf[0];
  varB = buf[1];
  myVar = 0;

  myVar = varA;
  myVar <<= 8;           //Left shit 8 bits, so from 1 Dec it's not 256 Dec. From 0x01 to 0x100;
  myVar = myVar | varB;  //OR operation, basically 0x100 merged with 0x71, which will result in 0x171
  Serial.print("======== Servo: ");
  Serial.print(myVar, DEC);
  Serial.print("               ");
  // Serial.println(myVar,HEX);
  //myservo.write(myVar);
  //Serial.println("===============================================");
}

// Prints the Error number
void CheckError(int ErrNo, S7Client* Client_local) {
  Serial.print("Error No. 0x");
  Serial.println(ErrNo, HEX);

  // Checks if it's a Severe Error => we need to disconnect
  if (ErrNo & 0x00FF) {
    Serial.println("SEVERE ERROR, disconnecting.");
    Client_local->Disconnect();
  }
}

// Infra-red sensors
void Initialize_sensors() {
  bool found_any_sensors = false;
  pinMode(OutFreqPin, OUTPUT);
  // Set all shutdown pins low to shutdown sensors
  for (int i = 0; i < COUNT_SENSORS; i++)
    digitalWrite(sensors[i].shutdown_pin, LOW);
  delay(10);

  for (int i = 0; i < COUNT_SENSORS; i++) {
    // one by one enable sensors and set their ID
    digitalWrite(sensors[i].shutdown_pin, HIGH);
    delay(10);  // give time to wake up.
    if (sensors[i].psensor->begin(sensors[i].id, false, sensors[i].pwire,
                                  sensors[i].sensor_config)) {
      found_any_sensors = true;
    } else {
      Serial.print(i, DEC);
      Serial.print(F(": failed to start\n"));
    }
  }
  if (!found_any_sensors) {
    Serial.println("No valid sensors found");
    while (1)
      ;
  }
}

void read_sensors() {

  //digitalWrite(13, HIGH);
  uint32_t start_time = millis();
  for (int i = 0; i < COUNT_SENSORS; i++) {
    ranges_mm[i] = sensors[i].psensor->readRange();
    timeouts[i] = sensors[i].psensor->timeoutOccurred();
    stop_times[i] = millis();
  }
  for (int i = 0; i < COUNT_SENSORS; i++)
    if (ranges_mm[i] > 60000)
      return;
  //delay(1000);
  uint32_t delta_time = millis() - start_time;
  //digitalWrite(13, LOW);

  Serial.print(delta_time, DEC);
  Serial.print(F(" "));
  for (int i = 0; i < COUNT_SENSORS; i++) {
    Serial.print(i);
    Serial.print(F(":"));
    Serial.print(ranges_mm[i]);
    Serial.print(F("        "));

    // Serial.print(F(" "));
    // Serial.print(stop_times[i] - start_time, DEC);
    //if (timeouts[i])
    //Serial.print(F("(TIMEOUT) "));
    //else
    //Serial.print(F("          "));


    start_time = stop_times[i];
  }
  Serial.println();
}