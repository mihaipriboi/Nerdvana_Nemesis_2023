// Include the 'Wire.h' libary to allow the Arduino to communicate with the time-of-flight sensors of the 'IIC' bus
#include <Wire.h>  

// Include the 'Adafruit_VL53L0X' libary for the VL53L0X time-of-flight distance sensors
#include <Adafruit_VL53L0X.h>  

// Create a 'Adafruit_VL53L0X' object for each sensor:
Adafruit_VL53L0X sensor1;
Adafruit_VL53L0X sensor2;
Adafruit_VL53L0X sensor3;

//====================================================================
// Define global variables for the sensors:
//====================================================================
typedef struct {
  Adafruit_VL53L0X *psensor; // pointer to object
  TwoWire *pwire;
  int id;            // IIC id number for the sensor
  int shutdown_pin;  // which pin for shutdown;
  int interrupt_pin; // which pin to use for interrupts.
  Adafruit_VL53L0X::VL53L0X_Sense_config_t sensor_config;     // options for how to use the sensor
  uint16_t range;        // range value used in continuous mode stuff.
  uint8_t sensor_status; // status from last ranging in continuous.
} sensorList_t;

// Setup for 2 sensors by defining information for each sensor in a 'sensors' array. Include
// a separate line of information for each sensor
sensorList_t sensors[] = {
  // For 'sensor1', define the IIC accress as hexadecimal value 0x30. Assign digital pin #4 to this 
  // sensor's XSHUT pin (shut-down pin). Assign digital pin #5 to the sensor's INTERRUPT pin.
  {&sensor1, &Wire, 0x30, 33, 34, Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0},

  // For 'sensor2', define the IIC accress as hexadecimal value 0x31. Assign digital pin #6 to this 
  // sensor's XSHUT pin (shut-down pin). Assign digital pin #7 to the sensor's INTERRUPT pin.
  {&sensor2, &Wire, 0x31, 35, 36, Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0},

  {&sensor3, &Wire, 0x32, 37, 38, Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0}
};

// Calculate the number of sensors by checking the size of the above 'sensors' array:
const int COUNT_SENSORS = sizeof(sensors) / sizeof(sensors[0]);

// Create array-variable for the sensors' range (in mm):
uint16_t ranges_mm[COUNT_SENSORS];

//====================================================================
// The 'Initialize_sensors' function:
//====================================================================
/*
    Reset all sensors by setting all of their XSHUT pins low for delay(10), then
    set all XSHUT high to bring out of reset. Keep sensor #1 awake by keeping XSHUT
    pin high. Put all other sensors into shutdown by pulling XSHUT pins low.
    Initialize sensor #1 with lox.begin(new_i2c_address). Pick any number except
    0x29 and it must be under 0x7F. Going with 0x30 to 0x3F is probably OK.
    Keep sensor #1 awake, and now bring sensor #2 out of reset by setting its
    XSHUT pin high. Initialize sensor #2 with lox.begin(new_i2c_address) Pick any
    number but 0x29 and whatever you set the first sensor to.
*/
void Initialize_sensors() {
  bool found_any_sensors = false;
  // Set all shutdown pins low to shutdown sensors
  for (int i = 0; i < COUNT_SENSORS; i++)
    digitalWrite(sensors[i].shutdown_pin, LOW);
  delay(10);

  for (int i = 0; i < COUNT_SENSORS; i++) {
    // one by one enable sensors and set their ID
    digitalWrite(sensors[i].shutdown_pin, HIGH);
    delay(10); // give time to wake up.
    if (sensors[i].psensor->begin(sensors[i].id, false, sensors[i].pwire,
                                  sensors[i].sensor_config)) {
      found_any_sensors = true;
    } else {
      Serial.print(i, DEC);
      Serial.print(F(": c to start\n"));
    }
  }
  if (!found_any_sensors) {
    Serial.println("No valid sensors found");
    while (1);
  }
}  // End of function 'Initialize_sensors'

//====================================================================
// The 'setup' function:
//====================================================================
void setup() {
  // Start the serial monitor at 9600 baud rate:
  Serial.begin(9600);

  // Start the IIC bus:
  Wire.begin();

  // Wait until serial port opens ... For 5 seconds max
  while (!Serial && (millis() < 5000));

  // Initialize all of the pins.
  Serial.println(F("VL53LOX_multi start, initialize IO pins"));
  for (int i = 0; i < COUNT_SENSORS; i++) {
    pinMode(sensors[i].shutdown_pin, OUTPUT);
    digitalWrite(sensors[i].shutdown_pin, LOW);

    if (sensors[i].interrupt_pin >= 0)
      pinMode(sensors[i].interrupt_pin, INPUT_PULLUP);
  }
  Serial.println(F("Starting..."));
  Initialize_sensors();
}  // End of function 'setup'

//====================================================================
// The 'loop' function:
//====================================================================
void loop() {

  // Read the data from the sensors using a 'for' loop:
  for (int i = 0; i < COUNT_SENSORS; i++) {
    ranges_mm[i] = sensors[i].psensor->readRange();  // This is where the sensor's data is captured
  }
  
  // Print out the distances to the serial monitor, again using a 'for' loop:
  for (int i = 0; i < COUNT_SENSORS; i++) {
    Serial.print("Sensor #");
    Serial.print(i, DEC);
    Serial.print(" at IIC address 0x");
    Serial.print(sensors[i].id, HEX);
    Serial.print(": ");
    Serial.print(ranges_mm[i], DEC);
    Serial.print(" mm       ");
  }
  Serial.println();

  // Delay until the next reading:
  delay(200);  // Argument is in milliseconds
  
}  // End of function 'loop'
