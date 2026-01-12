// ===================== MoodLight UNO (Stable Bridge + Non-blocking ESP) =====================
#include <Adafruit_NeoPixel.h>
#include <SoftwareSerial.h>
#include "WiFiEsp.h"
#include "SoftwareSerial.h"
#include <TimerOne.h>

// wifi set up start
#define DEBUG
#define DEBUG_WIFI
#define AP_SSID "iotA"
#define AP_PASS "iotA1234"
#define SERVER_NAME "10.10.14.85"
#define SERVER_PORT 5000
#define LOGID "LDH_ARD"
#define PASSWD "PASSWD"

#define ONMODE 1
#define OFFMODE 2
#define AUTOMODE 3
#define COLORMODE 4

#define CMD_SIZE 120
#define ARR_CNT 20

char sendBuf[CMD_SIZE];
bool timerIsrFlag = false;

unsigned long secCount;
int Time = 0;
int isOn = 0;
int onTimemode= 0;

#define ESP_RX_PIN 10   // ESP-01 TX → D10
#define ESP_TX_PIN 11   // ESP-01 RX ← D11 (5V→3.3V 분압 권장)
SoftwareSerial wifiSerial(ESP_RX_PIN, ESP_TX_PIN);
WiFiEspClient client;
// wifi set up end

// ---------- User Config ----------
#define LED_PIN       6
#define NUMPIXELS     20
#define PIXEL_TYPE    (NEO_GRB + NEO_KHZ800)
#define DEFAULT_BRIGHTNESS  60

#define UNO_BAUD      115200
#define ESP_BAUD      9600
#define ESP_UPLOAD    0
// ----------------------------------

// ---------- NeoPixel ----------
Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, PIXEL_TYPE);

int saveRV, saveGV, saveBV, saveOnTime, saveMode, saveSpeed;
static inline void fillAll(uint8_t r, uint8_t g, uint8_t b, uint8_t w=0) {
  uint32_t c = pixels.Color(r,g,b,w);
  for (int i=0;i<NUMPIXELS;i++) pixels.setPixelColor(i,c);
  pixels.show();
}

// ---------- LED Modes ----------
uint8_t  demo = 0; // 0=RAINBOW, 1=TOGGLE, 2=SOLID
uint8_t  speed = 3;
uint8_t  solidR=255, solidG=80, solidB=20, solidW=0;
uint8_t  currentBrightness = DEFAULT_BRIGHTNESS;
bool     solidApplied = false;
int      imsiFlag = 0;
uint8_t  rbPos=0; uint32_t lastAnimMs=0;
const uint16_t TOGGLE_MS=1000; uint8_t stage=0; uint32_t lastToggleMs=0;

static inline uint16_t intervalFor(uint8_t s){ s=constrain(s,1,5); return 40*(6-s); }
static inline uint32_t wheel(byte pos){
  pos = 255 - pos;
  if (pos < 85) return pixels.Color(255 - pos*3, 0, pos*3, 0);
  else if (pos < 170){ pos -= 85; return pixels.Color(0, pos*3, 255 - pos*3, 0); }
  else { pos -= 170; return pixels.Color(pos*3, 255 - pos*3, 0, 0); }
}
static inline void stepRainbow(){
  uint16_t iv = intervalFor(speed);
  if (millis()-lastAnimMs < iv) return;
  lastAnimMs = millis();
  for (int i=0;i<NUMPIXELS;i++) pixels.setPixelColor(i, wheel((i+rbPos)&255));
  pixels.show(); rbPos++;
}
static inline void stepToggle(){
  if (millis()-lastToggleMs < TOGGLE_MS) return;
  lastToggleMs = millis();
  switch(stage){
    case 0: fillAll(255,0,0,0); break;
    case 1: fillAll(0,255,0,0); break;
    case 2: fillAll(0,0,255,0); break;
    case 3: fillAll(0,0,0,255); break;
    case 4: fillAll(0,0,0,0); break;
  }
  stage = (stage+1)%5;
}
static inline void applyModeNow(){ if (demo==2) solidApplied=false; }

// ---------- Parsing ----------
String inLine;

static inline void handleKV(const String& kRaw, const String& vRaw){
  String k=kRaw; k.trim(); k.toUpperCase();
  String v=vRaw; v.trim();

  if(k=="MODE"){
    String vv=v; vv.toUpperCase();
    if(vv=="OFF"){ demo=2; solidR=solidG=solidB=solidW=0; }
    else if(vv=="SOLID") demo=2;
    else if(vv=="RAINBOW") demo=0;
    else if(vv=="TOGGLE") demo=1;
    applyModeNow();
  }
  else if (k=="R")   solidR=constrain(v.toInt(),0,255);
  else if (k=="G")   solidG=constrain(v.toInt(),0,255);
  else if (k=="B")   solidB=constrain(v.toInt(),0,255);
  else if (k=="W")   solidW=constrain(v.toInt(),0,255);
  else if (k=="BRI"||k=="BRIGHTNESS"){
    currentBrightness = constrain(v.toInt(),0,255);
    pixels.setBrightness(currentBrightness);
  }
  else if (k=="SPEED") speed=constrain(v.toInt(),1,5);
}

// ---------- ESP (non-blocking CIPSEND) ----------
SoftwareSerial ESP(ESP_RX_PIN, ESP_TX_PIN);
#if ESP_UPLOAD
String lastSend;
bool   esp_waiting_prompt = false;
unsigned long esp_deadline = 0;
const unsigned long ESP_PROMPT_WAIT_MS = 120;
#endif

static inline void handleLine(String line){
  line.trim();

  if (line.length()==1 && (line[0]=='0'||line[0]=='1'||line[0]=='2')){
    demo = (uint8_t)(line[0]-'0'); applyModeNow();
  }
  else if (line.startsWith("SENSOR@")){
  #if ESP_UPLOAD
    lastSend = line + "\r\n";
    ESP.print("AT+CIPSEND=");
    ESP.println(lastSend.length());
    esp_waiting_prompt = true;
    esp_deadline = millis() + ESP_PROMPT_WAIT_MS;
  #endif
  }
  else{
    int start=0;
    while(start<(int)line.length()){
      int sep=line.indexOf(';',start);
      String token=(sep==-1)? line.substring(start): line.substring(start,sep);
      int p=token.indexOf(':'); if(p<0) p=token.indexOf('=');
      if(p>0) handleKV(token.substring(0,p), token.substring(p+1));
      if(sep==-1) break; start=sep+1;
    }
  }

  if(demo==2){ fillAll(solidR,solidG,solidB,solidW); solidApplied=true; }
}

static inline void readFrom(Stream& s){
  while(s.available()){
    char c = (char)s.read();
    if (c=='\r') continue;
    if (c=='\n'){
      if (inLine.length()){ handleLine(inLine); inLine=""; }
    } else {
      if (inLine.length()<220) inLine += c;
      else inLine = "";
    }
  }
}

#if ESP_UPLOAD
static inline void espPump(){
  if (!esp_waiting_prompt) {
    while (ESP.available()) ESP.read();
    return;
  }
  while (ESP.available()){
    if (ESP.read() == '>'){
      ESP.print(lastSend);
      esp_waiting_prompt = false;
      return;
    }
  }
  if (millis() > esp_deadline){
    esp_waiting_prompt = false;
  }
}
#endif

// ---------- Button (ON/OFF 토글 전용) ----------
#define BTN_PIN         3       // 택트 스위치: 다른 쪽은 GND
#define BTN_DEBOUNCE_MS 25

static inline void button_update(){
  static bool btnPrev = HIGH;                 // INPUT_PULLUP: HIGH(해제), LOW(눌림)
  static unsigned long lastChange = 0;
  bool raw = digitalRead(BTN_PIN);
  unsigned long now = millis();

  if (raw != btnPrev && (now - lastChange) >= BTN_DEBOUNCE_MS){
    btnPrev = raw;
    lastChange = now;

    if (raw == LOW){ // 눌림 감지(낙하 에지)
      if (isOn == 2){
        fillAll(0,0,0,0);
        isOn = 0;
        char temp[] = "[LDH_SQL]LAMP@OFF@LDH_SMP\n";
        client.print(temp);
      } else if(isOn == 0){
        // 저장된 색으로 켜기
        char temp[] = "[LDH_SQL]LAMP@ON@LDH_SMP\n";
        client.print(temp);
        isOn = 1;
      }
      else if(isOn == 1){
        fillAll(0,0,0,0);
        isOn = 2;
      }
    }
  }
}

// ---------- Arduino Core ----------
int firstFlag = 0;

void setup(){
  pixels.begin();
  pixels.setBrightness(DEFAULT_BRIGHTNESS);
  pixels.show();

  pinMode(BTN_PIN, INPUT_PULLUP);     // ★ 택트 스위치
  Serial.begin(115200);               // DEBUG
  wifi_Setup();

  Timer1.initialize(1000000);         // 1Sec
  Timer1.attachInterrupt(timerIsr);
}

void loop(){
  if (client.available()) {
    socketEvent();
  }

  // 버튼 토글 처리(논블록)
  button_update();
  if(isOn == 2){
    stepRainbow();
  }

  if (timerIsrFlag) {
    timerIsrFlag = false;
    if (!(secCount % 5)) {
      if (!client.connected()) {
        server_Connect();
      }
    }
  }
}

void socketEvent() {
  int i = 0;
  char* pToken;
  char* pArray[ARR_CNT] = { 0 };
  char recvBuf[CMD_SIZE] = { 0 };
  int len;

  len = client.readBytesUntil('\n', recvBuf, CMD_SIZE);
  recvBuf[len] = '\0';

  Serial.println(recvBuf);
  pToken = strtok(recvBuf, "[@]");
  while (pToken != NULL) {
    pArray[i] = pToken;
    if (++i >= ARR_CNT) break;
    pToken = strtok(NULL, "[@]");
  }

  if (!strcmp(pArray[1], "LAMP")) {
    if (!strcmp(pArray[2], "ON")) {
      saveRV = atoi(pArray[3]);
      saveGV = atoi(pArray[4]);
      saveBV = atoi(pArray[5]);
      saveOnTime = atoi(pArray[6]);
      saveMode = atoi(pArray[7]);
      saveSpeed = atoi(pArray[8]);
      char imsi[100];
      sprintf(imsi, "%d %d %d", saveRV, saveGV, saveBV);
    Serial.print(imsi);
      fillAll(saveRV, saveGV, saveBV,0);
      isOn = 1;
    }
    else if (!strcmp(pArray[2], "OFF")) {
      fillAll(0,0,0,0);
      isOn = 0;
    }
    else if(!strcmp(pArray[2], "GET") || !strcmp(pArray[2], "SET")){
      saveRV = atoi(pArray[3]);
      saveGV = atoi(pArray[4]);
      saveBV = atoi(pArray[5]);
      saveOnTime = atoi(pArray[6]);
      saveMode = atoi(pArray[7]);
      saveSpeed = atoi(pArray[8]);
    }

    if(!strcmp(pArray[2],"CHANGE")){
      Serial.println("in change");
      firstFlag = 1;
      char temp[] = "[LDH_SQL]LAMP@GET@LDH_SMP\n";
      client.print(temp);
      client.flush();
    }
  }
  else if (!strncmp(pArray[1], " New connected!", 4)) {
    Serial.write('\n');
    return;
  }

  client.write(sendBuf, strlen(sendBuf));
  client.flush();
#ifdef DEBUG
  Serial.print(", send : ");
  Serial.print(sendBuf);
#endif
}

void timerIsr() {
  timerIsrFlag = true;
  secCount++;
  if(isOn == 1){
    if(saveMode == AUTOMODE && saveOnTime != 0){
      onTimemode ++;
      if(onTimemode == saveOnTime){
        fillAll(0,0,0,0);
        isOn = 0;
        onTimemode = 0;
      }
    }
  }


  
}

void wifi_Setup() {
  wifiSerial.begin(38400);
  wifi_Init();
  server_Connect();
}
void wifi_Init() {
  do {
    WiFi.init(&wifiSerial);
    if (WiFi.status() == WL_NO_SHIELD) {
#ifdef DEBUG_WIFI
      Serial.println("WiFi shield not present");
#endif
    } else break;
  } while (1);

#ifdef DEBUG_WIFI
  Serial.print("Attempting to connect to WPA SSID: ");
  Serial.println(AP_SSID);
#endif
  while (WiFi.begin(AP_SSID, AP_PASS) != WL_CONNECTED) {
#ifdef DEBUG_WIFI
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(AP_SSID);
#endif
  }
#ifdef DEBUG_WIFI
  Serial.println("You're connected to the network");
  printWifiStatus();
#endif
}
int server_Connect() {
#ifdef DEBUG_WIFI
  Serial.println("Starting connection to server...");
#endif
  if (client.connect(SERVER_NAME, SERVER_PORT)) {
#ifdef DEBUG_WIFI
    Serial.println("Connected to server");
#endif
    client.print("[" LOGID ":" PASSWD "]");
  } else {
#ifdef DEBUG_WIFI
    Serial.println("server connection failure");
#endif
  }
  return 0;
}
void printWifiStatus() {
  Serial.print("SSID: "); Serial.println(WiFi.SSID());
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: "); Serial.println(ip);
  long rssi = WiFi.RSSI();
  Serial.print("Signal strength (RSSI):"); Serial.print(rssi); Serial.println(" dBm");
}
