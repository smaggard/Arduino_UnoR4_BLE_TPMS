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
float rf_temp = 0.0;
float rf_battery = 0.0;
float lf_press = 0.0;
float lf_temp = 0.0;
float lf_battery = 0.0;
float rr_press = 0.0;
float rr_temp = 0.0;
float rr_battery = 0.0;
float lr_press = 0.0;
float lr_temp = 0.0;
float lr_battery = 0.0;
float celsius = 0.0;
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

std::tuple<float, float, float> processBLEDevice(BLEDevice peripheral) {
  // Function to process the devices found during scan and pull the proper
  // values from the Manufacturer data.
  if (peripheral.hasManufacturerData()) {
    int ManuDataLen = peripheral.manufacturerDataLength();
    uint8_t manuDataBuffer[ManuDataLen];
    // retrieve the data.... using manufacturerData etc.
    if (peripheral.manufacturerData(manuDataBuffer, ManuDataLen)) {
      float pressureInPSI = (((manuDataBuffer[10]) << 16 | (manuDataBuffer[9]) << 8 | manuDataBuffer[8]) / 100000.0) * 14.5037738;
      float tempInF = (((manuDataBuffer[13]) << 8 | manuDataBuffer[12]) / 100) * 1.8 + 32;
      float battery = manuDataBuffer[16];
      return std::make_tuple(pressureInPSI, tempInF, battery);
    } else {
      return std::make_tuple(0.00, 0.00, 0);
    }
  } else {
    return std::make_tuple(0.00, 0.00, 0);
  }
}

void setup() {

  Modulino.begin();
  leds.begin();
  Serial.begin(115200);
  while (!Serial) {
    leds.set(SERIAL_LED, RED, BRIGHTNESS);
    leds.show();
  }
  leds.set(SERIAL_LED, GREEN, BRIGHTNESS);
  leds.show();
  // Start Modulino devices
  //Modulino.begin();

  if (!thermo.begin()) {
    Serial.println("Thermo Start Failed.");
  }

  if (!matrix.begin()) {
    Serial.println("Matrix");
    leds.set(MATRIX_LED, RED, BRIGHTNESS);
    leds.show();
  } else {
    leds.set(MATRIX_LED, GREEN, BRIGHTNESS);
    leds.show();
  }
  if (!buzzer.begin()) {
    Serial.println("Buzzer failed to start");
  }

  //begin initialization
  //Start CAN
  if (!CAN.begin(CanBitRate::BR_1000k)) {
    Serial.println("CAN.begin(...) failed.");
    leds.set(CAN_LED, RED, BRIGHTNESS);
    leds.show();
    //while(1);
  } else {
    leds.set(CAN_LED, GREEN, BRIGHTNESS);
    leds.show();
  }

  // Start BLE
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");
    leds.set(BLE_LED, RED, BRIGHTNESS);
    leds.show();
    //while (1);
  } else {
    leds.set(BLE_LED, GREEN, BRIGHTNESS);
    leds.show();
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
    celsius = thermo.getTemperature();
    fahrenheit = (celsius * 9 / 5) + 32;
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
  for (;;) {
    BLEDevice peripheral = BLE.available();
    if (peripheral) {
      String address = peripheral.address();
      if (address == "80:ea:ca:50:2e:d8") {
        std::tuple<float, float, float> values = processBLEDevice(peripheral);
        lf_press = std::get<0>(values);
        lf_temp = std::get<1>(values);
        lf_battery = std::get<2>(values);
      } else if (address == "81:ea:ca:50:2e:61") {
        std::tuple<float, float, float> values = processBLEDevice(peripheral);
        rf_press = std::get<0>(values);
        rf_temp = std::get<1>(values);
        rf_battery = std::get<2>(values);
      } else if (address == "82:ea:ca:50:2d:18") {
        std::tuple<float, float, float> values = processBLEDevice(peripheral);
        lr_press = std::get<0>(values);
        lr_temp = std::get<1>(values);
        lr_battery = std::get<2>(values);
      } else if (address == "83:ea:ca:50:2d:34") {
        std::tuple<float, float, float> values = processBLEDevice(peripheral);
        rr_press = std::get<0>(values);
        rr_temp = std::get<1>(values);
        rr_battery = std::get<2>(values);
      } else {
        //Serial.println("Address " + address + " Didn't match");
      }
    }
    //Serial.println("No Devices Found");
    sendCANMsg(lf_press, 0x1E202627);
    sendCANMsg(lf_temp, 0x1E212627);
    sendCANMsg(lf_battery, 0x1E202628);
    if (lf_press < 30 && lf_press > 0) {
      leds.set(LF_TIRE, RED, BRIGHTNESS);
      leds.show();
    } else {
      leds.set(LF_TIRE, GREEN, BRIGHTNESS);
    }

    sendCANMsg(rf_press, 0x1E206627);
    sendCANMsg(rf_temp, 0x1E216627);
    sendCANMsg(rf_battery, 0x1E206628);
    if (rf_press < 30 && rf_press > 0) {
      
      
    } else {
      leds.set(RF_TIRE, GREEN, BRIGHTNESS);
    }

    sendCANMsg(lr_press, 0x1E20A627);
    sendCANMsg(lr_temp, 0x1E21A627);
    sendCANMsg(lr_battery, 0x1E20A628);
    if (lr_press < 30 && lf_press > 0) {
      leds.set(LR_TIRE, RED, BRIGHTNESS);
      leds.show();
    } else {
      leds.set(LR_TIRE, GREEN, BRIGHTNESS);
    }

    sendCANMsg(rr_press, 0x1E20E627);
    sendCANMsg(rr_temp, 0x1E21E627);
    sendCANMsg(rr_battery, 0x1E20E628);
    if (rr_press < 30 && rr_press > 0) {
      leds.set(RR_TIRE, RED, BRIGHTNESS);
      leds.show();
    } else {
      leds.set(RR_TIRE, GREEN, BRIGHTNESS);
    }

    vTaskDelay(700 / portTICK_PERIOD_MS);
  }
}

void loop() {
}