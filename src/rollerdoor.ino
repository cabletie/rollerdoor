#include <Particle.h>
#include <elapsedMillis.h>
#include <application.h>
// #include <stdarg.h>
#include <MQTT.h>
#include <mcp23s17.h>

#define RD_VERSION "1.7.2"
#define ARDUINOJSON_ENABLE_PROGMEM 0
#include <Arduino.h>
#include <ArduinoJson.h>

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#include <map>
#include <unordered_map>

// mcp23s17 gpio_x(mcp23s17::HardwareAddress::HW_ADDR_0); // All addressing pins set to GND

char gDeviceInfo[255]; // Device Status string - static version number, build date etc

// 
// Define IO pins used
// 
int doorUpPin = D0; // Input from door up hall effect sensor
int doorDownPin = D1; // Input from door down hall effect sensor
int boardLed = D7; // This is the LED that is already on your device.
int doorButtonPin = D2; // Output to push the door open/close button

//
// mcs23s17 pins
//
#define RELAY1_PIN 1 // Hopefully pin 1 is GPA1 whch is the first relay.
#define RELAY2_PIN 0 // Hopefully pin 1 is GPA1 whch is the first relay.

//
// System status vars
//
bool ledStatus;
int doorUp;
int doorDown;
int rssi = 0;
byte bssid[6];
String sBssid = "";

// minimum time between particle.publish calls
// 1 Second
#define INTER_PUBLISH_DELAY 1000
// period between sending continuous data readings
// 1 minute
#define SEND_DATA_PERIOD 60000UL
// period between sending system status messages
// 5 minutes
#define SEND_STATUS_PERIOD 300000UL
// Period between checks for MQTT connection
// 1 Minute
#define CHECK_MQTT_PERIOD 60000UL

elapsedMillis senddata_timeout = 0;
elapsedMillis sendstatus_timeout = 0;
elapsedMillis checkmqtt_timeout = 0;

// Roller door release pulse width in ms
#define RELEASE_PULSE_WIDTH 500

// Setup for remote reset
#define DELAY_BEFORE_REBOOT 2000
unsigned int rebootDelayMillis = DELAY_BEFORE_REBOOT;
unsigned long rebootSync = millis();
bool resetFlag = false;

// MQTT
#define ROLLERDOOR "RollerDoor"
#define UID_LENGTH 8
#define MQTT_USER "mosquitto"
#define MQTT_PASS "%0mosquitto"

bool published = false;

// Setup for remote AutoDiscovery
bool autoDiscoverFlag = false;
bool publishFlag = false;
bool sendStateFlag = false;
bool pushButton = false;

// socket and network parameters
int serverPort = 23;

// start TCP servers
TCPServer tServer = TCPServer(serverPort);
TCPClient tClient;

enum telnetStates {DISCONNECTED, CONNECTED};
telnetStates telnetState = DISCONNECTED;
char myIpString[ ] = "000.000.000.000";

// Operational status vars
enum doorStates {unknown,open,opening,closed,closing,ds_error,stopped,moving};
const char* stateNames[] = {"unknown","open","opening","closed","closing","error","stopped","moving"};
enum doorCommands {NONE, OPEN, CLOSE, STOP};
doorCommands doorCommand = NONE;
doorStates doorStatus = unknown;
doorStates oldDoorStatus = unknown;

// Times the period the door release pulse is active
Timer pulseTimer(RELEASE_PULSE_WIDTH, releaseTheButton,TRUE);

// Actually push the button to the roller door
// Duration is set by RELEASE_PULSE_WIDTH ms
void pushTheButton(){
	digitalWrite(doorButtonPin,HIGH);
  // gpio_x.digitalWrite(RELAY1_PIN, mcp23s17::PinLatchValue::HIGH);
	pulseTimer.start();
	Serial.println("Pushing the button");
  if (telnetState == CONNECTED) {
    // serializeJsonPretty(gDeviceInfo, tClient);
    tClient.println("Pushing the button");
  }
}

// Callback to release the button once the pulsewidth timer has expired
void releaseTheButton()
{
	digitalWrite(doorButtonPin,LOW);
  // gpio_x.digitalWrite(RELAY1_PIN, mcp23s17::PinLatchValue::LOW);
	Serial.println("Releasing the button");
  // Send update to TCP Client
  if (telnetState == CONNECTED) {
    // serializeJsonPretty(gDeviceInfo, tClient);
    tClient.println("Releasing the button");
  }

}

// Use the external antenna if available
STARTUP( WiFi.selectAntenna(ANT_AUTO));

// Setup doorStatus indicate LED update timer
void updateDoorMovingStatusLed()
{
    if((doorStatus==opening)|(doorStatus==closing))
    {
        ledStatus = !ledStatus;
        digitalWrite(boardLed,ledStatus);
    }
    else if (doorStatus == open)
        digitalWrite(boardLed,HIGH);
    else if (doorStatus == closed)
        digitalWrite(boardLed,LOW);
}

// Setup doorStatus indicate LED update timer
void updateDoorUnknownStatusLed()
{
    if((doorStatus==unknown))
        {
            ledStatus = !ledStatus;
            digitalWrite(boardLed,ledStatus);
        }
}

Timer moving_timer(300, updateDoorMovingStatusLed);
Timer unknown_timer(75, updateDoorUnknownStatusLed);

// System.deviceID()
String deviceID;
// Last 6 digits of deviceID
String miniDeviceID;
// Base MQTT topic
String baseTopic;
// Status topic
String statusTopic;

// Remote Reset Function
int cloudResetFunction(String command) {
  resetFlag = true;
  rebootSync = millis();
  return 0;
}
// Remote AutoDiscovery Command
int cloudAutoDiscoverFunction(String command) {
  autoDiscoverFlag = true;
  return 1;
}

// Remote Publish Command
int cloudPublishFunction(String command) {
  publishFlag = true;
  return 1;
}

// function called when an MQTT message is received
void mqttCallback(char* topic, byte* payload, unsigned int length) {
	Serial.println(topic);
	char p[length + 1];
	memcpy(p, payload, length);
	p[length] = 0;

  if(!strcmp(topic,"cmnd/rollerdoor/autodiscover")) {
    autoDiscoverFlag = true; // Send autodiscover information
    publishFlag = true;
  }
  if(!strcmp(topic,"cmnd/rollerdoor/state")) {
    sendStateFlag = true; // Send state information
    publishFlag = true;
  }
  if(!strcmp(topic,baseTopic+"cover/set")) {
    pushButton = true; // Push the door open/close button
		if (!strcmp(p, "OPEN"))
				doorCommand = OPEN;
		else if (!strcmp(p, "CLOSE"))
				doorCommand = CLOSE;
		else if (!strcmp(p, "STOP"))
				doorCommand = STOP;
		else
				doorCommand = NONE;
		Serial.println(p);
    if (telnetState == CONNECTED) {
      tClient.println(p);
    }
  }
}

// Create MQTT Object
MQTT mqttClient("192.168.86.87", 1883, MQTT_DEFAULT_KEEPALIVE, mqttCallback, 512);

//  #define MQTT_HOST_DISCOVERY
#ifdef MQTT_HOST_DISCOVERY
void MqttDiscoverServer(void)
{
  if (!Wifi.mdns_begun) { return; }

  int n = MDNS.queryService("mqtt", "tcp");  // Search for mqtt service

  AddLog_P2(LOG_LEVEL_INFO, PSTR(D_LOG_MDNS D_QUERY_DONE " %d"), n);

  if (n > 0) {
    uint32_t i = 0;            // If the hostname isn't set, use the first record found.
#ifdef MDNS_HOSTNAME
    for (i = n; i > 0; i--) {  // Search from last to first and use first if not found
      if (!strcmp(MDNS.hostname(i).c_str(), MDNS_HOSTNAME)) {
        break;                 // Stop at matching record
      }
    }
#endif  // MDNS_HOSTNAME
    SettingsUpdateText(SET_MQTT_HOST, MDNS.IP(i).toString().c_str());
    Settings.mqtt_port = MDNS.port(i);

    AddLog_P2(LOG_LEVEL_INFO, PSTR(D_LOG_MDNS D_MQTT_SERVICE_FOUND " %s, " D_IP_ADDRESS " %s, " D_PORT " %d"), MDNS.hostname(i).c_str(), SettingsText(SET_MQTT_HOST), Settings.mqtt_port);
  }
}
#endif  // MQTT_HOST_DISCOVERY


String tnetToString(int tnetState){
  String msg = "Unknown";
  switch (tnetState){
    case CONNECTED:
        msg = "Connected";
        break;
    case DISCONNECTED:
        msg = "Disconnected";
        break;    
  }
  return msg;
}

void updateStatus(){
    if (telnetState == CONNECTED) {
      tClient.println("Sending Status update");
    }

    // Update values first
    IPAddress myIP = WiFi.localIP();
    sprintf(myIpString, "%d.%d.%d.%d", myIP[0], myIP[1], myIP[2], myIP[3]);
    rssi = WiFi.RSSI();
    byte bssid[6];
    WiFi.BSSID(bssid);
    
    sBssid = String::format("%02X:%02X:%02X:%02X:%02X:%02X", bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5]);
    // Send some system version info
    snprintf(gDeviceInfo, sizeof(gDeviceInfo),
        "{\"Application\":\"%s\",\"Version\":\"%s\",\"Date\":\"%s\",\"Time\":\"%s\",\"Sysver\":\"%s\",\"RSSI\":\"%d\",\"IPAddress\":\"%s\",\"DoorStatus\":\"%s\" }"
        ,__FILENAME__
        ,RD_VERSION
        ,__DATE__
        ,__TIME__
        ,(const char*)System.version()  // cast required for String
        ,rssi
        ,myIpString
				,stateNames[doorStatus]
    );

    //Then publish the STATUS stuff
    if (mqttClient.isConnected()) {
        mqttClient.publish(baseTopic+"tele/LWT","Online",true);
        mqttClient.publish(baseTopic+"tele/HASS_STATE",gDeviceInfo,true);
    }
    // Send update to TCP Client
    if (telnetState == CONNECTED) {
        // serializeJsonPretty(gDeviceInfo, tClient);
        tClient.println(gDeviceInfo);
    }

}
void mqttConnectAndSubscribe()
{
  // connect to the MQTT broker if not already
  if(!mqttClient.isConnected()){
    if(mqttClient.connect(ROLLERDOOR,MQTT_USER,MQTT_PASS))
    {
      mqttClient.subscribe("cmnd/rollerdoor/state");
      mqttClient.subscribe("cmnd/rollerdoor/autodiscover");
      mqttClient.subscribe(baseTopic+"cover/set");
      sendAutoDiscover();
    }
  }
}
// Serialize JSON and publish to MQTT Client
void sendMqtt(const String &topic, const String &msg, bool retain){
  // Try three times 10 seconds apart
  for (size_t i = 0; i < 3; i++)
  {
    if (mqttClient.isConnected()) {
        mqttClient.publish(topic,msg,retain);
        // Particle.publish("Debug", "MQTT Published", 300, PRIVATE);
        return;
    }
    mqttConnectAndSubscribe();
    // mqttClient.connect(ROLLERDOOR,MQTT_USER,MQTT_PASS);
    delay(10000);
    Particle.publish("Debug", "Reconnect MQTT", 300, PRIVATE);
  }
}

// Create a JSON object to send as the MQTT auto discovery data
void sendAutoDiscover(){
  String msg;

  DynamicJsonDocument jsonDoc(800);
  // Sample JSON
  // homeassistant/switch/459293_RL_1/config = {
    // "name":"Sonoff-GPO-1",
    // "cmd_t":"~cmnd/POWER",
    // "stat_t":"~tele/STATE",
    // "val_tpl":"{{value_json.POWER}}",
    // "pl_off":"OFF","pl_on":"ON",
    // "avty_t":"~tele/LWT",
    // "pl_avail":"Online",
    // "pl_not_avail":"Offline",
    // "uniq_id":"459293_RL_1",
    // "device":{"identifiers":["459293"]},
    // "~":"gpo-4755/"
  // } (retained)
 
  // Define common bits to all sensors
  jsonDoc["stat_t"] = statusTopic;
  jsonDoc["avty_t"] = "~tele/LWT";
  jsonDoc["pl_avail"] = "Online";
  jsonDoc["pl_not_avail"] = "Offline";

  JsonObject device = jsonDoc.createNestedObject("device");
  JsonArray identifiers = device.createNestedArray("identifiers");
  identifiers.add(deviceID);
  JsonArray connections = device.createNestedArray("connections");
  JsonArray con_map = connections.createNestedArray(); 
  String myIp = myIpString;
  con_map.add("ip");
  con_map.add(myIp);
  jsonDoc["~"] =  baseTopic;

  // Bits just for each sensor
  jsonDoc["name"] = "Roller Door";
  jsonDoc["dev_cla"] = "garage";
  jsonDoc["val_tpl"] = "{{value_json.STATUS}}";
  jsonDoc["uniq_id"] = miniDeviceID+"_DS";
  jsonDoc["cmd_t"] = "~cover/set";
  jsonDoc["pl_cls"] = "CLOSE";
  jsonDoc["pl_open"] = "OPEN";
  serializeJson(jsonDoc,msg);
  sendMqtt("homeassistant/cover/"+jsonDoc["uniq_id"].as<String>()+"/config",msg,true);
  if (telnetState == CONNECTED) {
    serializeJsonPretty(jsonDoc, tClient);
    tClient.println();
  }

  // Prepare a /config JSON for the status entity
  jsonDoc["name"] = "Roller Door Controller Status";
  jsonDoc["stat_t"] = "~tele/HASS_STATE";
  jsonDoc["json_attributes_topic"] = "~tele/HASS_STATE";
  jsonDoc["val_tpl"] = "{{value_json[\'RSSI\']}}";
  jsonDoc["ic"] = "mdi:information-outline";

  jsonDoc.remove("pl_off");
  jsonDoc.remove("pl_on");
  jsonDoc.remove("dev_cla");
  jsonDoc.remove("cmd_t");
  jsonDoc.remove("pl_cls");
  jsonDoc.remove("pl_open");

  jsonDoc["unit_of_meas"] = "dBm";
  jsonDoc["uniq_id"] = miniDeviceID+"_status";
  device.remove("connections");
  device["name"] = ROLLERDOOR;
  device["mdl"] = "Particle Photon";
  device["sw"] = RD_VERSION;
  device["mf"] = "PW&A";

  msg = "";
  serializeJson(jsonDoc,msg);
  sendMqtt("homeassistant/sensor/"+jsonDoc["uniq_id"].as<String>()+"/config",msg,true);
}

void publishAll(uint32_t doorStatus) {
        DynamicJsonDocument jsonDoc(800);
        jsonDoc["STATUS"] = stateNames[doorStatus];
        // Send update to TCP Client
        if (telnetState == CONNECTED) {
            serializeJsonPretty(jsonDoc, tClient);
            tClient.println();
        }

        // Send update to MQTT Client
        String msg;
        serializeJson(jsonDoc,msg);
        sendMqtt(statusTopic,msg,true);
}

void setup() {
  // General GPIO setup
  pinMode(doorUpPin,INPUT); 
  pinMode(boardLed,OUTPUT); 
  pinMode(doorDownPin,INPUT);  
  pinMode(doorButtonPin,OUTPUT); 

  // mcp23s17 setup
  // gpio_x.pinMode(RELAY1_PIN, mcp23s17::PinMode::OUTPUT);

  // Set output high (not grounded) immediately
  digitalWrite(doorButtonPin,LOW);
  // gpio_x.digitalWrite(RELAY1_PIN, mcp23s17::PinLatchValue::LOW);

  doorStatus = unknown;
  Serial.begin();
  // Serial1.begin(4800,SERIAL_8N1); // open serial communications to the photon

  // Announce function
  Particle.function("reset", cloudResetFunction);
  Particle.function("mqtt", cloudAutoDiscoverFunction);
  Particle.function("publish", cloudPublishFunction);
  Particle.connect();

  // Initialise some static info
  deviceID = System.deviceID();
  // Pick out the middle bit of the ID 
  // - the RH end seems to not change sometimes and 
  // - the LH has too many zeroes.
  int didl = deviceID.length();
  int start = (didl-UID_LENGTH)<0?0:(didl-UID_LENGTH);
  start = (start>UID_LENGTH)?UID_LENGTH:start;
  miniDeviceID = deviceID.substring(start,start+UID_LENGTH);
  // miniDeviceID = deviceID.substring(8,15);
  Serial.printf("miniDeviceID: %s\n",miniDeviceID.c_str());
  baseTopic = "rd-"+miniDeviceID+"/";
  statusTopic = baseTopic+"tele/STATE";

  // Wait for cloud connection
  while(!Particle.connected())
    Particle.process();

  // connect to the MQTT broker
  mqttConnectAndSubscribe();
  // mqttClient.connect(ROLLERDOOR,MQTT_USER,MQTT_PASS);
  // if(mqttClient.isConnected()){
  //     mqttClient.subscribe("cmnd/rollerdoor/state");
  //     mqttClient.subscribe("cmnd/rollerdoor/autodiscover");
  // 		mqttClient.subscribe(baseTopic+"cover/set");
  //     sendAutoDiscover();
  // }

  // Find my IP address and make it and a bunch of variables publicly available via particle.io
  Particle.variable("devIP", myIpString, STRING);  // Var used to get IP for telnet client.
  Particle.variable("telnetState",telnetState);
  Particle.variable("rssi", rssi);
  Particle.variable("bssid", sBssid);

  // Operational variables
  Particle.variable("deviceInfo",gDeviceInfo);
  Particle.variable("doorStatus",doorStatus);

  updateStatus();

  tServer.begin(); // begin listening for TCP connections

  // Particle.variable("doorStatus",doorStatus);
  Particle.publish("doorStatus","unknown",60,PRIVATE);
  // Particle.publish("doorEvent","127",60,PRIVATE);

  // Serial.begin();
  digitalWrite(boardLed,LOW); // Turn off board LED
  ledStatus = LOW;
}

// Now for the loop.

void loop()
{
  // Create the main JSON document for sending
  DynamicJsonDocument jsonDoc(800);

  // Deal with setting up Telnet client
  if (tClient.connected()) {
    if(telnetState == DISCONNECTED){
      telnetState = CONNECTED;
      tClient.println("Hello!");
      Particle.publish("rollerdoor/tcp_connection","Connected",PRIVATE);
      published = false; // Trigger sending of data immediately
    }
	}
	else {
		// if no client is yet connected, check for a new connection
		if (telnetState == CONNECTED) {
			tClient.stop();
			telnetState = DISCONNECTED;
  		Particle.publish("rollerdoor/tcp_connection","Disconnected",PRIVATE);
		}
		tClient = tServer.available();
	}

  if((doorStatus == unknown) & !unknown_timer.isActive())
      unknown_timer.start();
  else if((doorStatus != unknown) & unknown_timer.isActive())
      unknown_timer.stop();

  // Get door endstop input statuses
  doorUp = !digitalRead(doorUpPin);
  doorDown = !digitalRead(doorDownPin);

  if (doorUp & doorDown) // Illegal state or there's a space-time continuum problem
  {
    if (doorStatus != ds_error) // Transition from not error to error
    {
      Particle.publish("doorStatus","error",60,PRIVATE);
      // Particle.publish("doorEvent","127",60,PRIVATE);
      moving_timer.stop();
      unknown_timer.start();
      doorStatus = ds_error;
      // Publish status if there's been a change
      if (oldDoorStatus != doorStatus){
          publishAll(doorStatus);
          oldDoorStatus = doorStatus;
      }
      Serial.println("Both open and closed sensors at the same time!");
      if (telnetState == CONNECTED) {
        // serializeJsonPretty(gDeviceInfo, tClient);
        tClient.println("Both open and closed sensors at the same time!");
      }
    }
  } 
  else if (doorUp) 
  {
    if ((doorStatus==opening) | (doorStatus==unknown) | (doorStatus==closing))
    {
      Particle.publish("doorStatus","open",60,PRIVATE);
      // Particle.publish("doorEvent","255",60,PRIVATE);
      moving_timer.stop();
      digitalWrite(boardLed,HIGH);
      Serial.println("Door now open");
    }
    doorStatus=open;
    if (oldDoorStatus != doorStatus){
      publishAll(doorStatus);
      oldDoorStatus = doorStatus;
    }
  }
  else if(doorStatus==open)
  {// was open
    Particle.publish("doorStatus","closing",60,PRIVATE);
    moving_timer.start();
    doorStatus=closing;
    if (oldDoorStatus != doorStatus){
      publishAll(doorStatus);
      oldDoorStatus = doorStatus;
    }
    Serial.println("Door now closing");
  }
  else if (doorDown)
  {
    if ((doorStatus==closing) | (doorStatus == unknown) | (doorStatus == opening))
    {
        Particle.publish("doorStatus","closed",60,PRIVATE);
        // Particle.publish("doorEvent","0",60,PRIVATE);
        moving_timer.stop();
        digitalWrite(boardLed,LOW);
        Serial.println("Door now closed");
    }
    doorStatus=closed;
    if (oldDoorStatus != doorStatus){
        publishAll(doorStatus);
        oldDoorStatus = doorStatus;
    }
  }
  else if(doorStatus==closed)
  { // was closed
    Particle.publish("doorStatus","opening",60,PRIVATE);
    // Particle.publish("doorEvent","170",60,PRIVATE);
    moving_timer.start();
    doorStatus=opening;
    if (oldDoorStatus != doorStatus){
        publishAll(doorStatus);
        oldDoorStatus = doorStatus;
    }
    Serial.println("Door now opening");
  }

  // Process the command to open/close the door
  if(pushButton){
    pushButton = false; // No more to do
    switch (doorCommand)
    {
      case NONE:
        break;
      case STOP:
        if (telnetState == CONNECTED) {
          tClient.println("Received STOP command");
        }
        if ((doorStatus == opening )|(doorStatus == closing))
          {
            pushTheButton();
            doorStatus = stopped;
          }
        break;
        // Otherwise do nothing i.e. STOP when open or closed does nothing
      case OPEN:
        if (telnetState == CONNECTED) {
          tClient.println("Received OPEN command");
        }
        if(doorStatus == closing)
        { 
          pushTheButton();
          doorStatus = stopped;
          pushButton = true;
        }
        if(doorStatus == closed) 
        { 
          pushTheButton();
        }
        if((doorStatus == unknown )|(doorStatus == stopped))
        { 
          doorStatus = moving;
          pushTheButton();
          pushButton = true;
        }
        break;
      case CLOSE:
        if (telnetState == CONNECTED) {
          tClient.println("Received CLOSE command");
        }
        if(doorStatus == opening)
          { 
            pushTheButton();
            doorStatus = stopped;
            pushButton = true;
          }
        if(doorStatus == open) 
          { 
            pushTheButton();
          }
        if((doorStatus == unknown )|(doorStatus == stopped))
          { 
            doorStatus = moving;
            pushTheButton();
            pushButton = true;
          }
        break;      
      default:
        break;
    }
  }

  rssi=WiFi.RSSI();

  // Process MQTT Stuff
  if (mqttClient.isConnected())
    mqttClient.loop();

  //  Remote Reset Function
  if ((resetFlag) && (millis() - rebootSync >=  rebootDelayMillis)) {
    if (telnetState == CONNECTED) {
      // serializeJsonPretty(gDeviceInfo, tClient);
      tClient.println("Received Particle RESET command");
    }

    Particle.publish("Debug", "Remote Reset Initiated", 300, PRIVATE);
    delay(INTER_PUBLISH_DELAY); // A little delay after each publish to not overload the quota of 4 per second
    if (mqttClient.isConnected()) {
    mqttClient.publish(baseTopic+"tele/LWT","Offline",true);
    }
    System.reset();
  }

  //  Remote AutoDiscovery Function
  if (autoDiscoverFlag) {
    if (telnetState == CONNECTED) {
      // serializeJsonPretty(gDeviceInfo, tClient);
      tClient.println("Received Particle AUTODISCOVER command");
    }
    autoDiscoverFlag = false;
    Particle.publish("Debug", "AutoDiscovery triggered", 300, PRIVATE);
    delay(INTER_PUBLISH_DELAY); // A little delay after each publish to not overload the quota of 4 per second
    sendAutoDiscover();
  }
  //  Remote publish Function
  if (publishFlag) {
    if (telnetState == CONNECTED) {
      // serializeJsonPretty(gDeviceInfo, tClient);
      tClient.println("Received Particle PUBLISH command");
    }
    publishFlag = false;
    Particle.publish("Debug", "Publish triggered", 300, PRIVATE);
    // Initiate a publish of current data
    published=false;
  }
  if((sendstatus_timeout >= SEND_STATUS_PERIOD) | sendStateFlag){
    // Publish vars to particle.io
    // Includes sending ~/tele/LWT Online to MQTT
    sendStateFlag = false;
    updateStatus();
    sendstatus_timeout = 0;
  }
  // checkmqtt_timeout - make sure MQTT is connected and subscribed
  if((checkmqtt_timeout >= CHECK_MQTT_PERIOD)){
    mqttConnectAndSubscribe();
    checkmqtt_timeout = 0;
  }
}
