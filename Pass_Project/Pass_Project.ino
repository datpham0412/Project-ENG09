#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

// BME680 Setup
Adafruit_BME680 bme;

// SDI-12 Setup
#define DIRO 7

String command;
int sensorAddress = 0;
String deviceIdentification = "14ENG20009103171611xxx";

void setup() {
  // Arduino IDE Serial Monitor
  Serial.begin(9600);

  // ================ BME680 ================
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1)
      ;
  }
  // Set the temperature, pressure, and humidity oversampling
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setPressureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);

  // ================ SDI-12 ================
  Serial1.begin(1200, SERIAL_7E1);  // SDI-12 UART, configures serial port for 7 data bits, even parity, and 1 stop bit
  pinMode(DIRO, OUTPUT);            // DIRO Pin

  // HIGH to Receive from SDI-12
  digitalWrite(DIRO, HIGH);
  Serial.println("Arduino is ready. Send commands to control the sensors.");
}

void loop() {
  int byte;
  // Receive SDI-12 over UART and then print to Serial Monitor
  if (Serial1.available()) {
    byte = Serial1.read();  // Reads incoming communication in bytes
    // Serial.println(byte);
    if (byte == 33) {  // If byte is command terminator (!)
      SDI12Receive(command);
      command = "";  // reset command string
    } else {
      if (byte != 0) {          // do not add the start bit (0)
        command += char(byte);  // append byte to command string
      }
    }
  }
}

void SDI12Receive(String input) {
  // convert device address to string
  String address = String(sensorAddress);
  // Determines if the command is addressed for this device
  if (String(input.charAt(0)) == address) {
    // Repond to Start Measurement command "aTEST!"   **Notice: Not correctly implemented, this only demonstrates command and usage of I2C sensors
    if (String(input.charAt(1)) == "A") {
      sensorAddress = input.charAt(2) - '0';  // Convert character to integer
      SDI12Send(String(sensorAddress));
      Serial.println("Change Address");
    } else if (String(input.charAt(1)) == "M") {
        SDI12Send("0003");
        Serial.println("Start Measurement");
    } else if (String(input.charAt(1)) == "D" && (String(input.charAt(2)) == "0")) {
        bme.performReading();
        Serial.println("Temp(*C)");
        SDI12Send(String(bme.temperature));
        Serial.println("Pressure (hPa)");
        SDI12Send(String(bme.pressure / 100.0));
        Serial.println("Humidity (%)");
        SDI12Send(String(bme.humidity));
        Serial.println("Gas (Ohms)");
        SDI12Send(String(bme.gas_resistance));
        Serial.println("Send data");
    } else if (String(input.charAt(1)) == "I") {
        SDI12Send(deviceIdentification);
        Serial.println("Responding to identification command");
    }
  }
  if (String(input.charAt(0)) == "?") {
    SDI12Send(address);
    Serial.println("Responding to ?! command");
  }
}

void SDI12Send(String message) {
  // Serial.print("message: ");
  Serial.println(message);

  digitalWrite(DIRO, LOW);
  delay(100);
  Serial1.print(message + String("\r\n"));
  Serial1.flush();  // wait for print to finish
  Serial1.end();
  Serial1.begin(1200, SERIAL_7E1);
  digitalWrite(DIRO, HIGH);
}
