#define SENSOR1_WIRE Wire
#define SENSOR2_WIRE Wire
#if defined(WIRE_IMPLEMENT_WIRE1)
#define SENSOR3_WIRE Wire1
#else
#define SENSOR3_WIRE Wire
#endif
#define DEBUG 1
#define OutFreqPin 9
// Setup mode for doing reads


typedef struct {
  Adafruit_VL53L0X *psensor; // pointer to object
  TwoWire *pwire;
  int id;            // id for the sensor
  int shutdown_pin;  // which pin for shutdown;
  int interrupt_pin; // which pin to use for interrupts.
  Adafruit_VL53L0X::VL53L0X_Sense_config_t
      sensor_config;     // options for how to use the sensor
  uint16_t range;        // range value used in continuous mode stuff.
  uint8_t sensor_status; // status from last ranging in continuous.
} sensorList_t;

//Comunications main functions, used in separeted core
void task_read_clp(void* pvParameters=NULL);
void task_write_in_clp(void* pvParameters=NULL);

// Util
unsigned long GetTimeDif(unsigned long compare_time);
void printOut1(int c, int val_bits=7);
void MarkTime(unsigned long Elapsed_local);
void ShowTime(unsigned long Elapsed_local);

//Distance infra-red sensor
void Initialize_sensors();
void read_sensors();

//Settimino routines
bool Connect(S7Client *Client_local);
void Dump(void *Buffer, int Length);
void CheckError(int ErrNo, S7Client* Client_local);