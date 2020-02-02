// ESP32 WiFi <-> 3x UART Bridge
// by AlphaLima
// www.LK8000.com

// Disclaimer: Don't use  for life support systems
// or any other situations where system failure may affect
// user or environmental safety.

#include <ctype.h>
#include "config.h"
#include <esp_wifi.h>
#include <WiFi.h>

#ifdef BLUETOOTH
#include <BluetoothSerial.h>
BluetoothSerial SerialBT;
#endif

#ifdef OTA_HANDLER
#include <ArduinoOTA.h>
#endif // OTA_HANDLER

HardwareSerial Serial_one(1);
HardwareSerial Serial_two(2);
HardwareSerial* COM[NUM_COM] = { &Serial, &Serial_one, &Serial_two };

#define MAX_NMEA_CLIENTS 4
#ifdef PROTOCOL_TCP
#include <WiFiClient.h>
WiFiServer server_0(SERIAL0_TCP_PORT);
WiFiServer server_1(SERIAL1_TCP_PORT);
WiFiServer server_2(SERIAL2_TCP_PORT);
WiFiServer *server[NUM_COM] = { &server_0, &server_1, &server_2 };
WiFiClient TCPClient[NUM_COM][MAX_NMEA_CLIENTS];
int currentConnections = 0;
#endif

uint8_t buf1[NUM_COM][BUFFER_SIZE];
uint16_t i1[NUM_COM] = { 0, 0, 0 };

uint8_t buf2[NUM_COM][BUFFER_SIZE];
uint16_t i2[NUM_COM] = { 0, 0, 0 };

uint8_t BTbuf[BUFFER_SIZE];
uint16_t iBT = 0;

#ifdef PRINT_TS
byte hh = 0, mi = 0, ss = 0;
unsigned int dddd = 0;
unsigned long lastTick = 0;
#endif


void setup() {
  delay(500);

  COM[0]->begin(UART_BAUD0, SERIAL_PARAM0, SERIAL0_RXPIN, SERIAL0_TXPIN);
  COM[1]->begin(UART_BAUD1, SERIAL_PARAM1, SERIAL1_RXPIN, SERIAL1_TXPIN);
  COM[2]->begin(UART_BAUD2, SERIAL_PARAM2, SERIAL2_RXPIN, SERIAL2_TXPIN);

#ifdef LED_PIN
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, false);
#endif

  if(debug) COM[DEBUG_COM]->printf("\nWiFi serial bridge v%s\n", VERSION);

#ifdef MODE_AP
  if(debug) COM[DEBUG_COM]->println("Open ESP Access Point mode");
  // AP mode (phone connects directly to ESP) (no router)
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(ip, ip, netmask); // configure ip address for softAP
  WiFi.softAP(ssid, pw); // configure ssid and password for softAP
#endif

#ifdef MODE_STA
  if(debug) COM[DEBUG_COM]->println("Open ESP Station mode");
  // STATION mode (ESP connects to router and gets an IP)
  // Assuming phone is also connected to that router
  // from RoboRemo you must connect to the IP of the ESP
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pw);
  if(debug) COM[DEBUG_COM]->print("try to Connect to Wireless network: ");
  if(debug) COM[DEBUG_COM]->println(ssid);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    if(debug) COM[DEBUG_COM]->print(".");
  }
  if(debug) COM[DEBUG_COM]->println("\nWiFi connected");
#endif

#ifdef BLUETOOTH
  if(debug) COM[DEBUG_COM]->println("Open Bluetooth Server");
  SerialBT.begin(ssid); // Bluetooth device name
#endif

#ifdef OTA_HANDLER
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
  // if DNSServer is started with "*" for domain name, it will reply with
  // provided IP to all DNS request

  ArduinoOTA.begin();
#endif // OTA_HANDLER

#ifdef PROTOCOL_TCP
  COM[0]->println("Starting TCP Server 1");  
  if(debug) COM[DEBUG_COM]->println("Starting TCP Server 1");  
  server[0]->begin(); // start TCP server 
  server[0]->setNoDelay(true);
  COM[1]->println("Starting TCP Server 2");
  if(debug) COM[DEBUG_COM]->println("Starting TCP Server 2");  
  server[1]->begin(); // start TCP server 
  server[1]->setNoDelay(true);
  COM[2]->println("Starting TCP Server 3");
  if(debug) COM[DEBUG_COM]->println("Starting TCP Server 3");  
  server[2]->begin(); // start TCP server   
  server[2]->setNoDelay(true);
  #endif

  // lower WiFi Power
#if defined(ESP8266)
  esp_err_t esp_wifi_set_max_tx_power(50);
#elif defined(ESP32)
  WiFi.setTxPower(WIFI_POWER_7dBm);
#endif
}


void loop() 
{  
#ifdef OTA_HANDLER  
  ArduinoOTA.handle();
#endif // OTA_HANDLER
  
#ifdef BLUETOOTH
  // receive from Bluetooth:
  if(SerialBT.hasClient()) 
  {
    while(SerialBT.available())
    {
      BTbuf[iBT] = SerialBT.read(); // read char from client (LK8000 app)
      if(iBT <BUFFER_SIZE-1) iBT++;
    }          
    for(int num= 0; num < NUM_COM ; num++)
      COM[num]->write(BTbuf,iBT); // now send to UART(num):          
    iBT = 0;
  }  
#endif  
#ifdef PROTOCOL_TCP
  for(int num= 0; num < NUM_COM ; num++)
  {
    if (server[num]->hasClient())
    {
      for(byte i = 0; i < MAX_NMEA_CLIENTS; i++){
        //find free/disconnected spot
        if (!TCPClient[num][i] || !TCPClient[num][i].connected()){
          if(TCPClient[num][i]) TCPClient[num][i].stop();
          TCPClient[num][i] = server[num]->available();
          if(debug) COM[DEBUG_COM]->print("New client for COM"); 
          if(debug) COM[DEBUG_COM]->print(num); 
          if(debug) COM[DEBUG_COM]->println(i);
          continue;
        }
      }
      //no free/disconnected spot so reject
      WiFiClient TmpserverClient = server[num]->available();
      TmpserverClient.stop();
    }
  }

#ifdef MODE_AP
  if (WiFi.softAPgetStationNum() != currentConnections) {
    currentConnections = WiFi.softAPgetStationNum();
#ifdef LED_PIN
    digitalWrite(LED_PIN, currentConnections > 0 ? true : false);
#endif
  }
#endif

#endif

  for(int num= 0; num < NUM_COM ; num++)
  {
    if(COM[num] != NULL)
    {
      for(byte cln = 0; cln < MAX_NMEA_CLIENTS; cln++)
      {
        if(TCPClient[num][cln])
        {
          while(TCPClient[num][cln].available())
          {
            buf1[num][i1[num]] = TCPClient[num][cln].read(); // read char from client (LK8000 app)
            if(i1[num]<BUFFER_SIZE-1) i1[num]++;
          }

          COM[num]->write(buf1[num], i1[num]); // now send to UART(num):
#ifdef PRINT_TS
          if (debug && num != DEBUG_COM) {
            COM[DEBUG_COM]->printf(">> COM%d(%d): ", num, i1[num]);
            for (int i = 0; i < i1[num]; i++)
              COM[DEBUG_COM]->printf("%c", buf1[num][i]);
            COM[DEBUG_COM]->println();
          }
#endif
          i1[num] = 0;
        }
      }

      if(COM[num]->available())
      {
        while(COM[num]->available())
        {
          buf2[num][i2[num]] = COM[num]->read(); // read char from UART(num)
          if(i2[num]<BUFFER_SIZE-1) i2[num]++;
        }
        // now send to WiFi:
        for(byte cln = 0; cln < MAX_NMEA_CLIENTS; cln++)
        {
          if(TCPClient[num][cln])
            TCPClient[num][cln].write(buf2[num], i2[num]);
        }

#ifdef PRINT_TS
        if (debug && num != DEBUG_COM) {
          COM[DEBUG_COM]->printf("%dd/%02d:%02d:%02d << COM%d(%d): ", dddd, hh, mi, ss, num, i2[num]);
          int clen = buf2[num][i2[num]-1] == '\n' ? i2[num]-1 : i2[num];
          for (int i = 0; i < clen; i++) {
            COM[DEBUG_COM]->printf("%c", buf2[num][i]);
          }
          COM[DEBUG_COM]->println();
        }
#endif

#ifdef BLUETOOTH
        // now send to Bluetooth:
        if(SerialBT.hasClient())
          SerialBT.write(buf2[num], i2[num]);
#endif
        i2[num] = 0;
      }
    }
  }

#ifdef PRINT_TS
  if ((micros() - lastTick) >= 1000000UL) {
    lastTick += 1000000UL;
    ss++;
    if (ss >= 60) { ss-=60; mi++; }
    if (mi >= 60) { mi-=60; hh++; }
    if (hh >= 24) { hh-=24; dddd++; }
  }
#endif
}
