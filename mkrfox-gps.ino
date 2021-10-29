#include <SigFox.h>
#include <ArduinoLowPower.h>
#include <TinyGPS.h>

#define WAITING_TIME 0
#define GPS_PIN 2
#define GPS_INFO_BUFFER_SIZE 128

bool debug = false;

TinyGPS gps;

//GPS data variables
int year;
byte month, day, hour, minute, second, hundredths;
unsigned long chars;
unsigned short sentences, failed_checksum;
char GPS_info_char;
char GPS_info_buffer[GPS_INFO_BUFFER_SIZE];
unsigned int received_char;
bool message_started = false;
int i = 0;

// GPS coordinate structure, 12 bytes size on 32 bits platforms
struct gpscoord {
  float a_latitude;  // 4 bytes
  float a_longitude; // 4 bytes
  float a_altitude;  // 4 bytes
};

float latitude  = 0.0f;
float longitude = 0.0f;
float altitud = 0;


//////////////// Waiting function //////////////////
void Wait(int m, bool s) {
  //m minutes to wait
  //s slow led pulses
  if (debug) {
    Serial.print("Waiting: "); Serial.print(m); Serial.println(" min.");
  }

  digitalWrite(LED_BUILTIN, LOW);

  if (s) {
    int seg = m * 30;
    for (int i = 0; i < seg; i++) {
      digitalWrite(LED_BUILTIN, HIGH); //LED on
      delay(1000);
      digitalWrite(LED_BUILTIN, LOW); //LED off
      delay(1000);
    }
  } else {
    int seg = m * 15;
    for (int i = 0; i < seg; i++) {
      digitalWrite(LED_BUILTIN, HIGH); //LED on
      delay(1000);
      digitalWrite(LED_BUILTIN, LOW); //LED off
      delay(3000);
    }
  }
}

/////////////////// Sigfox Send Data function ////////////////
void SendSigfox(String data) {
  // Remove EOL
  data.trim();

  if (debug) {
    Serial.print("Sending: "); Serial.println(data);
    if (data.length() > 12) {
      Serial.println("Message too long, only first 12 bytes will be sent");
    }
  }

  // Start the module
  SigFox.begin();
  // Wait at least 30mS after first configuration (100mS before)
  delay(100);
  // Clears all pending interrupts
  SigFox.status();
  delay(1);
  // we need debug to disable greedy sleep if (debug) SigFox.debug();
  SigFox.debug();
  delay(100);

  SigFox.beginPacket();
  SigFox.print(data);


  if (debug) {
    int ret = SigFox.endPacket(true);  // send buffer to SIGFOX network and wait for a response
    if (ret > 0) {
      Serial.println("Transmission ok");
    } else {
      Serial.println("No transmission");
      Serial.println("Check the SigFox coverage in your area");
      Serial.println("If you are indoor, check the 20dB coverage or move near a window");
    }

    Serial.println(SigFox.status(SIGFOX));
    Serial.println(SigFox.status(ATMEL));

    if (SigFox.parsePacket()) {
      Serial.println("Response from server:");
      while (SigFox.available()) {
        Serial.print("0x");
        Serial.println(SigFox.read(), HEX);
      }
    } else {
      Serial.println("Could not get any response from the server");
    }
    Serial.println();
  } else {
    SigFox.endPacket();
  }
  SigFox.end();
}

//////////////////  Convert GPS function  //////////////////
/* Converts GPS float data to Char data */

String ConvertGPSdata(const void* data, uint8_t len) {
  uint8_t* bytes = (uint8_t*)data;
  String cadena ;
  if (debug) {
    Serial.print("Length: "); Serial.println(len);
  }

  for (uint8_t i = len - 1; i < len; --i) {
    if (bytes[i] < 12) {
      cadena.concat(byte(0)); // Not tested
    }
    cadena.concat(char(bytes[i]));
    if (debug) Serial.print(bytes[i], HEX);
  }

  if (debug) {
    Serial.println("");
    Serial.print("String to send: "); Serial.println(cadena);
  }

  return cadena;
}


////////////////////////// Get GPS position function/////////////////////
String GetGPSpositon() {

  int messages_count = 0;
  String pos;

  if (debug) Serial.println("GPS ON");
  digitalWrite(GPS_PIN, HIGH); //Turn GPS on
  Wait(1, false);
  while (messages_count < 500) {
    while (Serial1.available()) {

      int GPS_info_char = Serial1.read();

      if (GPS_info_char == '$') messages_count ++; // start of message. Counting messages.

      if (debug) {
        if (GPS_info_char == '$') { // start of message
          message_started = true;
          received_char = 0;
        } else if (GPS_info_char == '*') { // end of message
          for (i = 0; i < received_char; i++) {
            Serial.write(GPS_info_buffer[i]); // writes the message to the PC once it has been completely received
          }
          Serial.println();
          message_started = false; // ready for the new message
        } else if (message_started == true) { // the message is already started and I got a new character
          if (received_char <= GPS_INFO_BUFFER_SIZE) { // to avoid buffer overflow
            GPS_info_buffer[received_char] = GPS_info_char;
            received_char++;
          } else { // resets everything (overflow happened)
            message_started = false;
            received_char = 0;
          }
        }
      }

      if (gps.encode(GPS_info_char)) {
        gps.f_get_position(&latitude, &longitude);
        altitud = gps.altitude() / 100;

        // Store coordinates into dedicated structure
        gpscoord coords = {altitud, longitude, latitude};

        gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths);

        if (debug) {
          Serial.println();
          Serial.println();
          Serial.print("Latitud/Longitud: ");
          Serial.print(latitude, 5);
          Serial.print(", ");
          Serial.println(longitude, 5);
          Serial.println();
          Serial.print("Fecha: "); Serial.print(day, DEC); Serial.print("/");
          Serial.print(month, DEC); Serial.print("/"); Serial.print(year);
          Serial.print(" Hora: "); Serial.print(hour, DEC); Serial.print(":");
          Serial.print(minute, DEC); Serial.print(":"); Serial.print(second, DEC);
          Serial.print("."); Serial.println(hundredths, DEC);
          Serial.print("Altitud (metros): "); Serial.println(gps.f_altitude());
          Serial.print("Rumbo (grados): "); Serial.println(gps.f_course());
          Serial.print("Velocidad(kmph): "); Serial.println(gps.f_speed_kmph());
          Serial.print("Satelites: "); Serial.println(gps.satellites());
          Serial.println();
        }

        gps.stats(&chars, &sentences, &failed_checksum);
        if (debug) Serial.println("GPS turned off");
        digitalWrite(GPS_PIN, LOW); //GPS turned off
        pos = ConvertGPSdata(&coords, sizeof(gpscoord)); //Send data
        return pos;

      }

    }
  }
  pos = "No Signal";
}

//////////////////SETUP///////////////////

void setup() {
  if (debug) {
    Serial.begin(9600);
    while (!Serial) {}// wait for serial port to connect. Needed for native USB port only
    Serial.println("Serial Connected");
  }

  //Serial1 pins 13-14 for 3.3V connection to GPS.
  Serial1.begin(9600);
  while (!Serial1) {}
  if (debug) {
    Serial.println("GPS Connected");
  }

  pinMode(GPS_PIN, OUTPUT); //pin de interruptor del GPS

  if (!SigFox.begin()) {
    Serial.println("Shield error or not present!");
    return;
  }

  // Enable debug led and disable automatic deep sleep
  if (debug) {
    SigFox.debug();
  } else {
    SigFox.end(); // Send the module to the deepest sleep
  }
}

//////////////////////LOOP////////////////////////

void loop() {
  String position_data;

  position_data = GetGPSpositon();
  SendSigfox(position_data);

  //Wait(WAITING_TIME, false);
}
