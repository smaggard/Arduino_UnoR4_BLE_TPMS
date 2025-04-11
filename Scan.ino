/*
  TPMS CAN Sender
  This application scan BLE for the known TPMS sensors, 
  converts their values and sends them accross the CAN bus
  for use on a Holley Terminator X or Dominator as CAN expansion
  modules. 
*/

#include <ArduinoBLE.h>
#include <Arduino_CAN.h>
#include <tuple>
#include <Modulino.h>
#include <Arduino_FreeRTOS.h>
#include <ArduinoGraphics.h>
#include <Arduino_LED_Matrix.h>

// Initialize Modulino devices
ModulinoThermo thermo;
ArduinoLEDMatrix matrix;
ModulinoPixels leds;
ModulinoBuzzer buzzer;

// Set default values for TPMS Sensors
float rf_press = 0.0;
int rf_status = 0;
float lf_press = 0.0;
int lf_status = 0;
float rr_press = 0.0;
int rr_status = 0;
float lr_press = 0.0;
int lr_status = 0;
float fahrenheit = 0.0;
float humidity = 0;
float voltage = 0;
float voltage2 = 0;

#define SERIAL_LED 0
#define CAN_LED 1
#define BLE_LED 2
#define MATRIX_LED 3
#define LF_TIRE 4
#define RF_TIRE 5
#define LR_TIRE 6
#define RR_TIRE 7
#define BRIGHTNESS 10

void setLED(int led, ModulinoColor color) {
  leds.set(led, color, BRIGHTNESS);
  leds.show();
}

void floatToHexArray(float value, unsigned char *hexArray) {
  // Function to convert float values to hex arrays to be sent as a CAN message.
  memcpy(hexArray, &value, sizeof(float));
  std::swap(hexArray[0], hexArray[3]);
  std::swap(hexArray[1], hexArray[2]);
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void sendCANMsg(float value, uint32_t canId) {
  // Function to send CAN Messages containing the float value provided
  // on the CAN ID provided.
  unsigned char hexArray[4];
  floatToHexArray(value, hexArray);
  CanMsg const msg(CanExtendedId(canId), sizeof(hexArray), hexArray);
  int const rc = CAN.write(msg);
  //Serial.println(rc);
  delay(5);
}

std::tuple<float, int> processBLEDevice(BLEDevice peripheral) {
  // Function to process the devices found during scan and pull the proper
  // values from the Manufacturer data.
  if (peripheral.hasManufacturerData()) {
    int ManuDataLen = peripheral.manufacturerDataLength();
    uint8_t manuDataBuffer[ManuDataLen];
    int status = 1;
    // retrieve the data.... using manufacturerData etc.
    if (peripheral.manufacturerData(manuDataBuffer, ManuDataLen)) {
      float pressureInPSI = (((manuDataBuffer[10]) << 16 | (manuDataBuffer[9]) << 8 | manuDataBuffer[8]) / 100000.0) * 14.5037738;
      float battery = manuDataBuffer[16];
      if (battery <= 20 || (pressureInPSI < 30 && pressureInPSI > 0)) {
        status = 0;
      }
      return std::make_tuple(pressureInPSI, status);
    } 
    else {
      return std::make_tuple(0.00, 0);
    }
  }
  else {
    return std::make_tuple(0.00,  0);
  }
}

void setup() {
  Modulino.begin();
  leds.begin();
  Serial.begin(115200);
  while (!Serial) {
    setLED(SERIAL_LED, RED);
  }
  setLED(SERIAL_LED, GREEN);

  if (!thermo.begin()) {
    Serial.println("Thermo Start Failed.");
  }

  if (!matrix.begin()) {
    Serial.println("Matrix");
    setLED(MATRIX_LED, RED);
  } 
  else {
    setLED(MATRIX_LED, GREEN);
  }
  if (!buzzer.begin()) {
    Serial.println("Buzzer failed to start");
  }

  //begin initialization
  //Start CAN
  if (!CAN.begin(CanBitRate::BR_1000k)) {
    Serial.println("CAN.begin(...) failed.");
    setLED(CAN_LED, RED);
    //while(1);
  } 
  else {
    setLED(CAN_LED, GREEN);
  }

  // Start BLE
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");
    setLED(BLE_LED, RED);
    //while (1);
  } 
  else {
    setLED(BLE_LED, GREEN);
  }
  // Serial.println("Bluetooth® Low Energy Central scan");
  // start scanning for peripheral
  BLE.scan();

  // Start Loops
  auto const read_sensor = xTaskCreate(
    ReadSensor,
    "ReadSensor",
    100,
    NULL,
    1,
    NULL);
  if (read_sensor != pdPASS) {
    Serial.println(read_sensor);
    Serial.println("Failed to create Read Sensor Loop");
  }
  
  auto const tpms_scanner = xTaskCreate(
    TPMSScanner,
    "TPMSScanner",
    1000,
    NULL,
    1,
    NULL);
  if (tpms_scanner != pdPASS) {
    Serial.println(tpms_scanner);
    Serial.println("Failed to create TPMS Scanner loop");
  }
  
  auto const matrix_draw = xTaskCreate(
    MatrixDraw,
    "MatrixDraw",
    256,
    NULL,
    1,
    NULL);
  if (matrix_draw != pdPASS) {
    Serial.println(matrix_draw);
    Serial.println("Failed to create matrix draw loop");
  }
 
  vTaskStartScheduler();
}


void ReadSensor(void *parameters) {
  for (;;) {
    // Read the Amp sensor
    int sensorValue = analogRead(A0);
    int sensorValue2 = analogRead(A1);
    // Convert to 5.0V scale
    voltage = sensorValue * (5.0 / 1023.0);
    voltage2 = sensorValue2 * (5.0 / 1023.0);
    // Send Can messages
    sendCANMsg(voltage, 0x1E202629);
    sendCANMsg(voltage2, 0x1E206629);

    // Read the temperature Sensor
    fahrenheit = (thermo.getTemperature() * 9 / 5) + 32;
    humidity = thermo.getHumidity();
    // Send Can messages
    sendCANMsg(fahrenheit, 0x1E20A629);
    sendCANMsg(humidity, 0x1E20E629);

    if (fahrenheit > 120) {
      int frequency = 440;
      int duration = 1000;
      buzzer.tone(frequency, duration);
    }
    // Delay 50 milliseconds
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void MatrixDraw(void *parameters) {
  for (;;) {
    String temperature_text = String(fahrenheit, 1) + "°F";
    String humidity_text = String(humidity, 0) + "%";
    String low_amperage_text = String(mapfloat(voltage, 0.0, 5.0, -50.0, 50.0), 3) + "Amps";
    String high_amperage_text = String(mapfloat(voltage2, 0.0, 5.0, -200.0, 200.0), 3) + "Amps";
    matrix.beginDraw();
    matrix.stroke(0xFFFFFFFF);
    matrix.textScrollSpeed(75);
    matrix.textFont(Font_5x7);
    matrix.beginText(0, 1, 0xFFFFFF);
    matrix.println(" " + temperature_text + " " + humidity_text + " " + low_amperage_text + " " + high_amperage_text + " ");
    matrix.endText(SCROLL_LEFT);
    matrix.endDraw();
  }
}

void TPMSScanner(void *parameters) {
  // check if a peripheral has been discovered
  // Serial.println("In TPMSSCanner");
  float rf_press = 0.0;
  int rf_status = 1;
  float lf_press = 0.0;
  int lf_status = 1;
  float rr_press = 0.0;
  int rr_status = 1;
  float lr_press = 0.0;
  int lr_status = 1;
  for (;;) {
    BLEDevice peripheral = BLE.available();
    if (peripheral) {
      String address = peripheral.address();
      if (address == "80:ea:ca:50:2e:d8") {
        std::tuple<float, int> values = processBLEDevice(peripheral);
        lf_press = std::get<0>(values);
        lf_status = std::get<1>(values);
      } 
      else if (address == "81:ea:ca:50:2e:61") {
        std::tuple<float, int> values = processBLEDevice(peripheral);
        rf_press = std::get<0>(values);
        rf_status = std::get<1>(values);
      } 
      else if (address == "82:ea:ca:50:2d:18") {
        std::tuple<float, int> values = processBLEDevice(peripheral);
        lr_press = std::get<0>(values);
        lr_status = std::get<1>(values);
      } 
      else if (address == "83:ea:ca:50:2d:34") {
        std::tuple<float, int> values = processBLEDevice(peripheral);
        rr_press = std::get<0>(values);
        rr_status = std::get<1>(values);
      } 
      else {
        //Serial.println("Address " + address + " Didn't match");
      }
    }
    //Serial.println("No Devices Found");
    if (lf_status == 0) {
      setLED(LF_TIRE, RED);
    } 
    else {
      setLED(LF_TIRE, GREEN);
    }
    sendCANMsg(lf_press, 0x1E202627);
    sendCANMsg(lf_status, 0x1E212627);
    
    if (rf_status == 0) {
      setLED(RF_TIRE, RED);
    } 
    else {
      setLED(RF_TIRE, GREEN);
    }
    sendCANMsg(rf_press, 0x1E206627);
    sendCANMsg(rf_status, 0x1E216627);

    if (lr_status == 0) {
      setLED(LR_TIRE, RED);
    } 
    else {
      setLED(LR_TIRE, GREEN);
    }
    sendCANMsg(lr_press, 0x1E20A627);
    sendCANMsg(lr_status, 0x1E21A627);

    if (rr_status == 0) {
      setLED(RR_TIRE, RED);
    } 
    else {
      setLED(RR_TIRE, GREEN);
    }
    sendCANMsg(rr_press, 0x1E20E627);
    sendCANMsg(rr_status, 0x1E21E627);

    vTaskDelay(700 / portTICK_PERIOD_MS);
  }
}

void loop() {
}
