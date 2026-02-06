#include <SPI.h>
#include <Ethernet2.h>
#include <EthernetUdp2.h>
#include <PubSubClient.h>
#include <ModbusMaster.h>
#include <EEPROM.h>
#include <avr/wdt.h>

/* =========================
   Constants
   ========================= */
#define NUM_CHANNELS     32
#define SLAVE_ID         1
#define MAX485_DE_RE     11

#define REG_INPUT_LOW    0x90
#define REG_INPUT_HIGH   0x91
#define REG_OUTPUT_LOW    0x80
#define REG_OUTPUT_HIGH   0x81
#define BAUDRATE_REG     0xFE
#define BAUDRATE_115K    0x07

#define EEPROM_MAGIC     0x42

const unsigned long POLL_OK_MS    = 20;
const unsigned long POLL_ERROR_MS = 250;

/* =========================
   Network
   ========================= */
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xE1 };

/* =========================
   Globals
   ========================= */
EthernetClient ethClient;
EthernetServer webServer(80);
EthernetUDP Udp;
PubSubClient mqtt(ethClient);
ModbusMaster node;

const unsigned int DISCOVERY_PORT = 8266;
IPAddress broadcastIP;

unsigned long lastPoll = 0;
unsigned long pollInterval = POLL_OK_MS;

unsigned long lastAlive = 0;
const unsigned long ALIVE_INTERVAL = 10000;  // 10 seconds

// Poll intervals
#define POLL_INTERVAL_OK    20    // normal polling in ms
#define POLL_INTERVAL_ERROR 250   // slowed polling on Modbus error

unsigned long currentPollInterval = POLL_INTERVAL_OK;

unsigned long lastDiscoverySend = 0;
uint32_t lastInputs = 0;
bool modbusOnline = false;
bool mqttConnected = false;
bool discoverySent = false;
bool FirstRegister = true;
bool mqttConfigured = false;

uint16_t DiscreteOutputs[] ={ 0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x1A,0x1B,0x1C,0x1D,0x1E,0x1F };

#define DISCOVERY_INTERVAL 5000  // 5 seconds
unsigned long lastDiscoveryMillis = 0;

#define WATCHDOG_TIMEOUT 50000  // 50 seconds
bool WatchdogDisabled = true;

unsigned long lastMQTTAttempt = 0;
const unsigned long MQTT_RETRY_INTERVAL = 30000; // retry every 30s

int contentLength = 0;

/* =========================
   Device info
   ========================= */
const char* DEVICE_MANUFACTURER = "iRulez";
const char* DEVICE_MODEL        = "32ch Modbus IO";
const char* FW_VERSION          = "1.0.1";
const char* HW_VERSION          = "Rev-A";
//const char* CONFIG_URL          = "https://irulez.be";
char CONFIG_URL[32];

char deviceName[32];
String DEVICE_ID;

/* =========================
   EEPROM config
   ========================= */
struct Config {
  uint8_t magic;
  bool useDHCP;
  IPAddress staticIP;
  IPAddress gateway;
  IPAddress subnet;
  char mqtt_server[32];
  uint16_t mqtt_port;
  char mqtt_user[32];
  char mqtt_pass[32];
  char deviceName[32];
} config;

/* =========================
   RS485
   ========================= */
void preTransmission()  { digitalWrite(MAX485_DE_RE, HIGH); }
void postTransmission() { digitalWrite(MAX485_DE_RE, LOW); }

/* =========================
   EEPROM
   ========================= */
void saveConfig() {
  config.magic = EEPROM_MAGIC;
  EEPROM.put(0, config);
}

void factoryReset() {
  for (unsigned int i = 0; i < sizeof(Config); i++)
    EEPROM.write(i, 0xFF);
  delay(500);
  asm volatile ("jmp 0");
}

void loadConfig() {
  EEPROM.get(0, config);
  if (config.magic != EEPROM_MAGIC) {
    config.magic = EEPROM_MAGIC;
    config.useDHCP = true;
    config.staticIP = IPAddress(192,168,1,200);
    config.gateway  = IPAddress(192,168,1,1);
    config.subnet   = IPAddress(255,255,255,0);
    strcpy(config.mqtt_server, "0.0.0.0");
    config.mqtt_port = 1883;
    strcpy(config.mqtt_user, "irulez");
    strcpy(config.mqtt_pass, "password");
    strcpy(config.deviceName, "iRulez_32IO");
    saveConfig();
  }

  strcpy(deviceName, config.deviceName);
  DEVICE_ID = String(deviceName);
}

/* =========================
   Web UI
   ========================= */
void handleWeb() {
  EthernetClient client = webServer.available();
  if (!client) return;
  while (client.connected() && !client.available()) delay(1);

  String req = client.readStringUntil('\r');
  client.flush();

  // Factory Reset
  if (req.indexOf("GET /reset") >= 0) {
    client.println(F("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n"));
    client.println(F("<!DOCTYPE html><html><head><meta charset='UTF-8'></head><body><h2>Factory Reset!</h2></body></html>"));
    client.stop();
    delay(1000);
    factoryReset();
  }

  // Save Settings
  if (req.indexOf("POST /save") >= 0) {
    while (client.available() == 0) delay(1);
    String postData = client.readString();

    Config newConfig = config;
    newConfig.useDHCP = (postData.indexOf("dhcp=") >= 0);

    auto getParam = [&](const char* name, const String& data) -> String {
      String key = String(name) + "=";
      int idx = data.indexOf(key);
      if (idx < 0) return "";
      int end = data.indexOf('&', idx);
      if (end < 0) end = data.length();
      return data.substring(idx + key.length(), end);
    };

    String ipStr = getParam("ip", postData);
    if (ipStr.length()) sscanf(ipStr.c_str(), "%hhu.%hhu.%hhu.%hhu",
      &newConfig.staticIP[0], &newConfig.staticIP[1], &newConfig.staticIP[2], &newConfig.staticIP[3]);

    String gwStr = getParam("gw", postData);
    if (gwStr.length()) sscanf(gwStr.c_str(), "%hhu.%hhu.%hhu.%hhu",
      &newConfig.gateway[0], &newConfig.gateway[1], &newConfig.gateway[2], &newConfig.gateway[3]);

    String snStr = getParam("sn", postData);
    if (snStr.length()) sscanf(snStr.c_str(), "%hhu.%hhu.%hhu.%hhu",
      &newConfig.subnet[0], &newConfig.subnet[1], &newConfig.subnet[2], &newConfig.subnet[3]);

    String s = getParam("mqtt", postData);
    if (s.length()) s.toCharArray(newConfig.mqtt_server, sizeof(newConfig.mqtt_server));

    s = getParam("port", postData);
    if (s.length()) newConfig.mqtt_port = s.toInt();

    s = getParam("user", postData);
    if (s.length()) s.toCharArray(newConfig.mqtt_user, sizeof(newConfig.mqtt_user));

    s = getParam("pass", postData);
    if (s.length()) s.toCharArray(newConfig.mqtt_pass, sizeof(newConfig.mqtt_pass));

    s = getParam("devname", postData);
    if (s.length()) {
      s.toCharArray(newConfig.deviceName, sizeof(newConfig.deviceName));
      strcpy(deviceName, newConfig.deviceName);
    }

    config = newConfig;
    saveConfig();

    client.println(F("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n"));
    client.println(F("<!DOCTYPE html><html><head><meta charset='UTF-8'><meta http-equiv='refresh' content='3;url=/'></head><body>"));
    client.println(F("<h2>✅ Settings saved successfully!</h2><p>Rebooting device in 3 seconds...</p></body></html>"));
    client.stop();
    delay(3000);
    asm volatile ("jmp 0");  // Soft reboot
    return;
  }

  // Main Page
  client.println(F("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n"));
  client.println(F("<!DOCTYPE html><html><head><meta charset='UTF-8'><title>iRulez Config</title>"));
client.println(F("<style>"));
client.println(F("body{font-family:Arial,Helvetica,sans-serif; background:#fff8dc; margin:0; padding:0;}"));
client.println(F(".container{max-width:520px; margin:18px auto; background:#fff3cc; padding:14px 18px; border-radius:14px; box-shadow:0 6px 18px rgba(0,0,0,0.18);}"));
client.println(F("h2{text-align:center; margin:4px 0 8px 0; color:#3a2c00;}"));
client.println(F("p{text-align:center; margin:6px 0;}"));
client.println(F("fieldset{border:1px solid #e6c24d; padding:10px; margin-bottom:10px; border-radius:10px;}"));
client.println(F("legend{padding:0 8px; font-weight:bold; color:#5a4600;}"));
client.println(F("table{width:100%; border-collapse:collapse;}"));
client.println(F("th,td{padding:4px 6px; text-align:left; vertical-align:middle;}"));
client.println(F("th{width:40%; color:#5a4600;}"));
client.println(F("input[type=text], input[type=password], input[type=number]{width:220px; padding:6px; border-radius:6px; border:1px solid #ccc;}"));
client.println(F("input[type=submit]{background:#f2c94c; border:none; border-radius:8px; padding:6px 16px; font-weight:bold; cursor:pointer;}"));
client.println(F("input[type=submit]:hover{background:#e6b800;}"));
client.println(F(".status{font-weight:bold;}"));
client.println(F(".ok{color:green;}"));
client.println(F(".bad{color:red;}"));
client.println(F("a{color:#b03a2e;font-weight:bold;text-decoration:none;} a:hover{text-decoration:underline;}"));
client.println(F(".switch{position:relative; display:inline-block; width:44px; height:24px;}"));
client.println(F(".switch input{opacity:0;width:0;height:0;}"));
client.println(F(".slider{position:absolute;cursor:pointer;top:0;left:0;right:0;bottom:0;background:#ccc;transition:.3s;border-radius:24px;}"));
client.println(F(".slider:before{position:absolute;content:'';height:18px;width:18px;left:3px;bottom:3px;background:white;transition:.3s;border-radius:50%;}"));
client.println(F("input:checked + .slider{background:#f2c94c;}"));
client.println(F("input:checked + .slider:before{transform:translateX(20px);}"));
client.println(F("</style></head><body><div class='container'>"));

  client.println(F("<h2>i<i>R</i>ulez 32IO Board Configuration</h2><hr>"));
client.print(F("<p>MQTT Status: <span class='status "));
client.print(mqttConnected ? "ok'>Connected" : "bad'>Disconnected");
client.println(F("</span></p>"));
  client.println(F("</b></p>"));

  client.println(F("<form action='/save' method='post'>"));

  // Network
client.println(F("<fieldset><legend>Network Settings</legend><table>"));

client.print(F("<tr><th>Use DHCP</th><td><label class='switch'><input type='checkbox' name='dhcp' "));
if (config.useDHCP) client.print(F("checked"));
client.println(F("><span class='slider'></span></label></td></tr>"));

client.print(F("<tr><th>Static IP</th><td><input type='text' name='ip' value='"));
client.print(config.staticIP);
client.println(F("'></td></tr>"));

client.print(F("<tr><th>Gateway</th><td><input type='text' name='gw' value='"));
client.print(config.gateway);
client.println(F("'></td></tr>"));

client.print(F("<tr><th>Subnet</th><td><input type='text' name='sn' value='"));
client.print(config.subnet);
client.println(F("'></td></tr>"));

client.println(F("</table></fieldset>"));

  // MQTT
 client.println(F("<fieldset><legend>MQTT Settings</legend><table>"));

client.print(F("<tr><th>Server</th><td><input type='text' name='mqtt' value='"));
client.print(config.mqtt_server);
client.println(F("'></td></tr>"));

client.print(F("<tr><th>Port</th><td><input type='number' name='port' value='"));
client.print(config.mqtt_port);
client.println(F("'></td></tr>"));

client.print(F("<tr><th>User</th><td><input type='text' name='user' value='"));
client.print(config.mqtt_user);
client.println(F("'></td></tr>"));

client.print(F("<tr><th>Password</th><td><input type='password' name='pass' value='"));
client.print(config.mqtt_pass);
client.println(F("'></td></tr>"));

client.println(F("</table></fieldset>"));

  // Device Name
client.println(F("<fieldset><legend>Device Name</legend>"));
client.print(F("<input type='text' name='devname' value='"));
client.print(deviceName);
client.println(F("'>"));
client.println(F("</fieldset>"));

client.println(F("<div style='text-align:center;margin-top:10px;'>"));
client.println(F("<input type='submit' value='Save Settings'>"));
client.println(F("</div><hr>"));
client.println(F("<p style='text-align:center;'><a href='/reset'>Factory Reset</a></p>"));

  client.stop();
}

/* =========================
   MQTT
   ========================= */
void mqttPub(const String& topic, const String& payload, bool retain=true) {
  mqtt.publish(topic.c_str(), payload.c_str(), retain);
}

void publishModbusHealthSensor() {
  char t[128], p[384];
  snprintf(t,sizeof(t),
    "homeassistant/binary_sensor/%s/modbus_health/config",
    DEVICE_ID.c_str());
  snprintf(p,sizeof(p),
    "{\"name\":\"Modbus Communication\","
    "\"unique_id\":\"%s_modbus_health\","
    "\"state_topic\":\"%s/status\","
    "\"payload_on\":\"offline\",\"payload_off\":\"online\","
    "\"device_class\":\"problem\","
    "\"device\":{\"identifiers\":[\"%s\"],\"name\":\"%s\"}}",
    DEVICE_ID.c_str(),
    DEVICE_ID.c_str(),
    DEVICE_ID.c_str(),
    deviceName
  );
  mqttPub(t,p);
}

void publishDiscovery() {
  mqttPub(DEVICE_ID + "/status", "online");

  for (int i=0;i<NUM_CHANNELS;i++) {
    char t[128], p[512];

    snprintf(t,sizeof(t),"homeassistant/switch/%s/out_%02d/config",DEVICE_ID.c_str(),i+1);
    snprintf(p,sizeof(p),
      "{\"name\":\"Output %d\",\"unique_id\":\"%s_out_%d\","
      "\"command_topic\":\"%s/switch/%d/set\","
      "\"state_topic\":\"%s/switch/%d/state\","
      "\"availability_topic\":\"%s/status\","
      "\"payload_on\":\"ON\",\"payload_off\":\"OFF\","
      "\"device\":{\"identifiers\":[\"%s\"],\"name\":\"%s\","
      "\"manufacturer\":\"%s\",\"model\":\"%s\","
      "\"sw_version\":\"%s\",\"hw_version\":\"%s\",\"configuration_url\":\"%s\"}}",
      i+1,DEVICE_ID.c_str(),i+1,
      DEVICE_ID.c_str(),i+1,
      DEVICE_ID.c_str(),i+1,
      DEVICE_ID.c_str(),
      DEVICE_ID.c_str(),DEVICE_ID.c_str(),
      DEVICE_MANUFACTURER, DEVICE_MODEL,
      FW_VERSION, HW_VERSION, CONFIG_URL);

    mqttPub(t,p);

    snprintf(t,sizeof(t),"homeassistant/binary_sensor/%s/in_%02d/config",DEVICE_ID.c_str(),i+1);
    snprintf(p,sizeof(p),
      "{\"name\":\"Input %d\",\"unique_id\":\"%s_in_%d\","
      "\"state_topic\":\"%s/input/%d/state\","
      "\"availability_topic\":\"%s/status\","
      "\"payload_on\":\"ON\",\"payload_off\":\"OFF\","
      "\"device\":{\"identifiers\":[\"%s\"]}}",
      i+1,DEVICE_ID.c_str(),i+1,
      DEVICE_ID.c_str(),i+1,
      DEVICE_ID.c_str(),
      DEVICE_ID.c_str());

    mqttPub(t,p);
  }
  publishModbusHealthSensor();
}


/* =========================
   MQTT callback
   ========================= */
void mqttCallback(char* topic, byte* payload, unsigned int len) {
  payload[len] = 0;
  String msg = (char*)payload;

  for (int i = 1; i <= NUM_CHANNELS; i++) {
    String t = DEVICE_ID + "/switch/" + String(i) + "/set";
    if (String(topic) == t) {
      bool on = (msg == "ON");
      uint8_t rc = node.writeSingleCoil(DiscreteOutputs[i-1], on);
      if (rc == node.ku8MBSuccess) {
        mqttPub(DEVICE_ID + "/switch/" + String(i) + "/state", on ? "ON" : "OFF");
      } else {
        Serial.print(F("Error writing coil "));
        Serial.println(i);
      }
      break;
    }
  }
}

/* =========================
   MQTT reconnect
   ========================= */
void reconnectMQTT() {
  
  if (!mqttConfigured) checkDiscoveryResponse();
  else {
     if (millis() - lastMQTTAttempt < MQTT_RETRY_INTERVAL) return;
     lastMQTTAttempt = millis();
     Serial.print(F("MQTT Connecting..."));
     wdt_reset();
     if (mqtt.connect(
          DEVICE_ID.c_str(),                // client ID
          config.mqtt_user,                 // user
          config.mqtt_pass,                 // pass
          (DEVICE_ID + "/status").c_str(),  // will topic
          1,                                // will QoS
          true,                             // will retain
          "offline"                         // will payload
        )) {

      Serial.println(F("connected"));
      mqttConnected = true;

      // 👉 Publish online state immediately
      mqttPub(DEVICE_ID + "/status", "online");

      // 👉 Subscribe to all switch command topics
      for (int i = 1; i <= NUM_CHANNELS; i++) {
        mqtt.subscribe((DEVICE_ID + "/switch/" + String(i) + "/set").c_str());
      }

    } else {
      Serial.print(F("failed, rc="));
      Serial.print(mqtt.state());
      Serial.println(F(" retry in 20s"));
      mqttConnected = false;
      //delay(5000);
    }

  }
}


/* =========================
   Modbus
   ========================= */

void handleModbusError(uint8_t rc) {
  if (modbusOnline) {
    Serial.print("Modbus ERROR rc=");
    Serial.println(rc);
    mqttPub(DEVICE_ID + "/status", "offline");
    modbusOnline = false;

    // Slow down polling
    currentPollInterval = POLL_INTERVAL_ERROR;
  }
}


void readModbus() {
  uint8_t rc;
  uint16_t high;
  uint16_t low;
  if (millis() - lastPoll < currentPollInterval) return;  // use currentPollInterval
  lastPoll = millis();
  if (FirstRegister){
    rc = node.readHoldingRegisters(REG_INPUT_LOW, 1);
    if (rc != node.ku8MBSuccess) { handleModbusError(rc); return; }
    low = node.getResponseBuffer(0);
    FirstRegister = false;
  }
  else {
    rc = node.readHoldingRegisters(REG_INPUT_HIGH, 1);
    if (rc != node.ku8MBSuccess) { handleModbusError(rc); return; }
    high = node.getResponseBuffer(0);
    FirstRegister = true;
  }
  uint32_t inputs = ((uint32_t)high << 16) | low;
  if (inputs != lastInputs) {
    uint32_t changed = inputs ^ lastInputs;
    for (int i = 0; i < NUM_CHANNELS; i++) {
      if (changed & (1UL << i)) {
        mqttPub(DEVICE_ID + "/input/" + String(i+1) + "/state",
                (inputs & (1UL << i)) ? "ON" : "OFF");
      }
    }
    lastInputs = inputs;
  }

  // Mark Modbus as online if it was previously offline
  if (!modbusOnline) {
    Serial.println("Modbus ONLINE");
    modbusOnline = true;
    mqttPub(DEVICE_ID + "/status", "online");
    currentPollInterval = POLL_INTERVAL_OK;   // reset to fast polling
  }
}

void readOutputsOnce() {
  uint8_t rc;
  rc = node.readHoldingRegisters(REG_OUTPUT_LOW, 1); // low register
  if (rc != node.ku8MBSuccess) {
    Serial.print("Modbus Failed reading output LOW, rc=");
    Serial.println(rc);
    return;
  }
  uint16_t low = node.getResponseBuffer(0);

  rc = node.readHoldingRegisters(REG_OUTPUT_HIGH, 1); // high register
  if (rc != node.ku8MBSuccess) {
    Serial.print("Modbus Failed reading output HIGH, rc=");
    Serial.println(rc);
    return;
  }
  uint16_t high = node.getResponseBuffer(0);

  uint32_t outputs = ((uint32_t)high << 16) | low;

  //Serial.println(F("Modbus Output status read"));

  // Publish each output
  for (int i = 0; i < NUM_CHANNELS; i++) {
    mqttPub(DEVICE_ID + "/switch/" + String(i+1) + "/state",
            (outputs & (1UL << i)) ? "ON" : "OFF");
  }
}

void readModbusOnce() {
  uint8_t rc;

  rc = node.readHoldingRegisters(REG_INPUT_LOW, 1);
  if (rc != node.ku8MBSuccess) return;
  uint16_t low = node.getResponseBuffer(0);

  rc = node.readHoldingRegisters(REG_INPUT_HIGH, 1);
  if (rc != node.ku8MBSuccess) return;
  uint16_t high = node.getResponseBuffer(0);

  uint32_t v = ((uint32_t)high << 16) | low;

  for (int i = 0; i < 32; i++) {
    mqttPub(
      String(DEVICE_ID) + "/input/" + String(i + 1) + "/state",
      (v & (1UL << i)) ? "ON" : "OFF"
    );
  }

  lastInputs = v;
  //firstReadDone = true;
}

void setupUdpDiscovery() {
  if (millis() - lastDiscoveryMillis < DISCOVERY_INTERVAL) return;
  lastDiscoveryMillis = millis();
  Udp.begin(12345);

  IPAddress ip = Ethernet.localIP();
  IPAddress mask = Ethernet.subnetMask();

  // Calculate broadcast IP
  for (int i = 0; i < 4; i++) {
    broadcastIP[i] = ip[i] | ~mask[i];
  }
  Serial.println("Discovering MQTT server");
  Serial.print(F("UDP broadcast to: "));
  Serial.println(broadcastIP);
  const char* msg = "DISCOVER_MQTT_SERVER";
  Udp.beginPacket(broadcastIP, DISCOVERY_PORT);
  Udp.write(msg);
  Udp.endPacket();
  Serial.println(F("UDP broadcast has been sent"));
}

void sendDiscovery() {
  const char* msg = "DISCOVER_MQTT_SERVER";
  Udp.beginPacket(broadcastIP, DISCOVERY_PORT);
  Udp.write(msg);
  Udp.endPacket();
  Serial.println(F("UDP broadcast has been sent"));
}

void checkDiscoveryResponse() {
  setupUdpDiscovery();
  int packetSize = Udp.parsePacket();
  wdt_reset();
  if (packetSize) {
    char buf[64];
    int len = Udp.read(buf, sizeof(buf)-1);
    buf[len] = 0;

    Serial.print(F("UDP response: "));
    Serial.println(buf);

    if (String(buf).startsWith("MQTT_SERVER=")) {
      String server = String(buf).substring(12);
      server.toCharArray(config.mqtt_server, sizeof(config.mqtt_server));
      mqttConfigured = true;
      //Serial.println((strlen(config.mqtt_server)));
      saveConfig();
    }
  }
}

/* =========================
   Setup
   ========================= */
void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);
  Serial.println();
  Serial.println("  +---------------------------+");
  Serial.println("  |          iRulez           |");
  Serial.println("  |                           |");
  Serial.println("  | Hold on, it can take some |");
  Serial.println("  | time before the board     |");
  Serial.println("  | connects and is visible   |"); 
  Serial.println("  |     in Home Assistant     |");
  Serial.println("  +---------------------------+");
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);
  Ethernet.init(10);
  pinMode(MAX485_DE_RE,OUTPUT);
  digitalWrite(MAX485_DE_RE,LOW);
  uint8_t resultBaudrate;
  loadConfig();
  //Serial.println((strlen(config.mqtt_server)));
  if (strlen(config.mqtt_server) > 7) {
    mqttConfigured = true;
  }
  Serial.print("initializing ethernet ");
  if(config.useDHCP) Ethernet.begin(mac);
  else Ethernet.begin(mac,config.staticIP,config.gateway,config.subnet);
  for (int i = 0; i < 12; i++) {
    Serial.print(".");
    delay(1500); // 500 ms between dots
  }
  Serial.println(".");
  Serial.println("initializing done");
  delay(1000);
  //setupUdpDiscovery();
  //sendDiscovery();

  webServer.begin();

  node.begin(SLAVE_ID,Serial2);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
  
  resultBaudrate = node.writeSingleRegister(BAUDRATE_REG,BAUDRATE_115K);
  if (resultBaudrate == node.ku8MBSuccess) {
    Serial.println("  +---------------------------+");
    Serial.println("  |        Attention!         |");
    Serial.println("  |                           |");
    Serial.println("  | Since this is the first   |");
    Serial.println("  | time you connect the      |");
    Serial.println("  | module, the baudrate has  |"); 
    Serial.println("  | changed to 115200 !!      |");
    Serial.println("  | Please unplug and         |");    
    Serial.println("  | plugin your PLC again!    |"); 
    Serial.println("  +---------------------------+");
  }
  else{
    Serial.println("Baudrate of the PLC already set to 115200");
  }
  Serial.println("Changing baudrate of RS485 to 115200");

  Serial2.begin(115200);
  node.begin(SLAVE_ID,Serial2);

  mqtt.setServer(config.mqtt_server,config.mqtt_port);
  mqtt.setCallback(mqttCallback);
  mqtt.setBufferSize(1024);
  IPAddress ip = Ethernet.localIP();
  Serial.print(F("IP Address: "));
  Serial.println(ip);
  snprintf(CONFIG_URL, sizeof(CONFIG_URL),
         "http://%u.%u.%u.%u",
         ip[0], ip[1], ip[2], ip[3]);
  //wdt_enable(WDTO_8S);  // Enable 8s watchdog
}

/* =========================
   Loop
   ========================= */
void loop() {
  wdt_reset();
  handleWeb();
  Ethernet.maintain();
  if(!mqtt.connected()) reconnectMQTT();
  else if(!discoverySent) {
    publishDiscovery();
    readOutputsOnce();
    readModbusOnce();
    discoverySent=true;
  }
  if (mqtt.connected()) {
    mqtt.loop();
    readModbus();
    if (millis() - lastAlive > ALIVE_INTERVAL) {
      if (!modbusOnline) {
        mqttPub(DEVICE_ID + "/status", "offline");
      }
      else {
        mqttPub(DEVICE_ID + "/status", "online");
      }
      lastAlive = millis();
    }    
  }
  if (millis() > WATCHDOG_TIMEOUT && WatchdogDisabled){
    Serial.println("Enable watchdog");
    wdt_enable(WDTO_8S);
    WatchdogDisabled = false;
  }
  
         
}
