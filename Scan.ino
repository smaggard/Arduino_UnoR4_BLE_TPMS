/*
  TPMS CAN Sender
  This application scan BLE for the known TPMS sensors, 
  converts their values and sends them accross the CAN bus
  for use on a Holley Terminator X or Dominator as CAN expansion
  modules. 
*/
#include <Arduino_FreeRTOS.h>
#include <ArduinoBLE.h>
#include <Arduino_CAN.h>
#include <tuple>
#include <Arduino.h>

String LFTireAddress = "98:58:8A:18:65:44";
String RFTireAddress = "98:58:8A:18:61:05";
String LRTireAddress = "98:58:8A:18:64:F1";
String RRTireAddress = "98:58:8A:18:61:35";

String tireAddresses[] = { LFTireAddress, RFTireAddress, LRTireAddress, RRTireAddress };

void floatToHexArray(float value, unsigned char *hexArray) {
  // Function to convert float values to hex arrays to be sent as a CAN message.
  memcpy(hexArray, &value, sizeof(float));
  std::swap(hexArray[0], hexArray[3]);
  std::swap(hexArray[1], hexArray[2]);
}

void sendCANMsg(float value, uint32_t canId) {
  // Function to send CAN Messages containing the float value provided
  // on the CAN ID provided.
  unsigned char hexArray[4];
  floatToHexArray(value, hexArray);
  CanMsg const msg(CanExtendedId(canId), sizeof(hexArray), hexArray);
  int const rc = CAN.write(msg);
  if (rc <= 0) {
    CAN.end();
    CAN.begin(CanBitRate::BR_1000k);
  }
  //Serial.println(rc);
  delay(5);
}

std::tuple<float, float, int> processBLEDevice(BLEDevice peripheral) {
  // Function to process the devices found during scan and pull the proper
  // values from the Manufacturer data.
  if (peripheral.hasManufacturerData()) {
    int ManuDataLen = peripheral.manufacturerDataLength();
    uint8_t manuDataBuffer[ManuDataLen];
    int status = 1;
    // retrieve the data.... using manufacturerData etc.
    if (peripheral.manufacturerData(manuDataBuffer, ManuDataLen)) {
      float pressureInPSI = (((manuDataBuffer[20]) << 8 | manuDataBuffer[21]) / 10000.0) * 14.5037738;
      float tempInF = ((((manuDataBuffer[22]) << 8 | manuDataBuffer[23]) / 100) * 1.8) + 32;
      if (pressureInPSI < 30 && pressureInPSI > 0) {
        status = 0;
      }
      return std::make_tuple(pressureInPSI, tempInF, status);
    } else {
      return std::make_tuple(0.00, 0.00, 0);
    }
  } else {
    return std::make_tuple(0.00, 0.00, 0);
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  //Start CAN
  if (!CAN.begin(CanBitRate::BR_1000k)) {
    Serial.println("CAN.begin(...) failed.");
    while (1)
      ;
  } else {
    Serial.println("Can Started.");
  }

  // Start BLE
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");
    //while (1);
  } else {
    Serial.println("Bluetooth Started.");
  }
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

  vTaskStartScheduler();
}


void ReadSensor(void *parameters) {

  for (;;) {
    // Read the Amp sensor
    int sensorValue = analogRead(A0);
    int sensorValue2 = analogRead(A1);
    // Convert to 5.0V scale
    float voltage = sensorValue * (5.0 / 1023.0);
    float voltage2 = sensorValue2 * (5.0 / 1023.0);
    // Send Can messages
    sendCANMsg(voltage, 0x1E202629);
    sendCANMsg(voltage2, 0x1E206629);

    // Delay 50 milliseconds
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}


void TPMSScanner(void *parameters) {
  // check if a peripheral has been discovered
  // Serial.println("In TPMSSCanner");

  struct Tire {
    float pressure;
    float temp;
    int status;
  };

  Tire lfTire = { .pressure = 0.0, .temp = 0, .status = 1 };
  Tire rfTire = { .pressure = 0.0, .temp = 0, .status = 1 };
  Tire lrTire = { .pressure = 0.0, .temp = 0, .status = 1 };
  Tire rrTire = { .pressure = 0.0, .temp = 0, .status = 1 };

  for (;;) {
    BLEDevice peripheral = BLE.available();
    if (peripheral) {
      String bleAddress = peripheral.address();
      bleAddress.toUpperCase();
      // Serial.println(bleAddress);
      if (bleAddress == LFTireAddress) {
        std::tuple<float, float, int> values = processBLEDevice(peripheral);
        lfTire.pressure = std::get<0>(values);
        lfTire.temp = std::get<1>(values);
        lfTire.status = std::get<2>(values);
      } else if (bleAddress == RFTireAddress) {
        std::tuple<float, float, int> values = processBLEDevice(peripheral);
        rfTire.pressure = std::get<0>(values);
        rfTire.temp = std::get<1>(values);
        rfTire.status = std::get<2>(values);
      } else if (bleAddress == LRTireAddress) {
        std::tuple<float, float, int> values = processBLEDevice(peripheral);
        lrTire.pressure = std::get<0>(values);
        lrTire.temp = std::get<1>(values);
        lrTire.status = std::get<2>(values);
      } else if (bleAddress == RRTireAddress) {
        std::tuple<float, float, int> values = processBLEDevice(peripheral);
        rrTire.pressure = std::get<0>(values);
        rrTire.temp = std::get<1>(values);
        rrTire.status = std::get<2>(values);
      }
    }
    sendCANMsg(lfTire.pressure, 0x1E202627);
    sendCANMsg(lfTire.status, 0x1E212627);
    sendCANMsg(lfTire.temp, 0x1E202004);

    sendCANMsg(rfTire.pressure, 0x1E206627);
    sendCANMsg(rfTire.status, 0x1E216627);
    sendCANMsg(rfTire.temp, 0x1E206004);

    sendCANMsg(lrTire.pressure, 0x1E20A627);
    sendCANMsg(lrTire.status, 0x1E21A627);
    sendCANMsg(lrTire.temp, 0x1E20A004);

    sendCANMsg(rrTire.pressure, 0x1E20E627);
    sendCANMsg(rrTire.status, 0x1E21E627);
    sendCANMsg(rrTire.temp, 0x1E20E004);

    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void loop() {
}
