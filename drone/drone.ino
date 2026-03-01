#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ===== CONFIGURATION =====
const char* ssid = "C3-DRONE";
const char* pass = "12345678";

// Pins
const int PIN_M1 = 21; // FL
const int PIN_M2 = 20; // FR
const int PIN_M3 = 2;  // RR (GPIO 7 is used for SDA, so moved to 2)
const int PIN_M4 = 10; // RL
const int I2C_SDA = 7;
const int I2C_SCL = 6;
const int PIN_LED = 8;

// PWM Settings
const int FREQ = 20000;
const int RES = 8;

Adafruit_MPU6050 mpu;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

volatile bool armed = false;
float throttle = 0, roll = 0, pitch = 0, yaw = 0;
uint32_t last_packet = 0;

// LED Status Logic
void updateLED() {
  static uint32_t last_ms = 0;
  static bool state = false;
  if (WiFi.softAPgetStationNum() > 0) {
    digitalWrite(PIN_LED, HIGH); // WiFi connected
  } else {
    if (millis() - last_ms > 500) { // Blink
      state = !state;
      digitalWrite(PIN_LED, state);
      last_ms = millis();
    }
  }
}

// Web UI HTML (Drone Controller Style)
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta name="viewport" content="width=device-width,initial-scale=1">
<style>
  body { background: #1a1a1a; color: white; font-family: sans-serif; text-align: center; margin: 0; overflow: hidden; }
  .grid { display: flex; justify-content: space-around; padding: 20px; height: 80vh; align-items: center; }
  .stick-area { width: 150px; height: 250px; background: #333; border-radius: 20px; position: relative; border: 2px solid #555; }
  #thr-knob { width: 100%; height: 50px; background: #00ff88; position: absolute; bottom: 0; border-radius: 10px; }
  .joy { width: 150px; height: 150px; background: #333; border-radius: 50%; position: relative; }
  .knob { width: 50px; height: 50px; background: white; border-radius: 50%; position: absolute; top: 50px; left: 50px; }
  .btn { padding: 15px 40px; font-size: 20px; margin: 10px; border-radius: 10px; border: none; font-weight: bold; }
  .arm { background: red; color: white; }
</style></head>
<body>
  <h2>C3 DRONE CTRL</h2>
  <div id="data">MPU: --</div>
  <button class="btn arm" onclick="toggleArm()" id="armBtn">DISARMED</button>
  <div class="grid">
    <div class="stick-area" id="thr-zone">
      <div id="thr-knob"></div>
    </div>
    <div class="joy" id="joy-zone">
      <div class="knob" id="joy-knob"></div>
    </div>
  </div>
<script>
  let ws = new WebSocket('ws://'+location.host+'/ws');
  let thr = 0, rl = 0, pt = 0;
  ws.onmessage = (e) => { document.getElementById('data').innerText = e.data; };
  
  function send() { ws.send(`C:${thr},${rl},${pt},0`); }
  function toggleArm() { ws.send("ARM"); }

  // Throttle (No auto-center)
  const thrZone = document.getElementById('thr-zone');
  thrZone.ontouchmove = (e) => {
    let r = thrZone.getBoundingClientRect();
    let v = 1 - (e.touches[0].clientY - r.top) / r.height;
    thr = Math.max(0, Math.min(100, Math.round(v * 100)));
    document.getElementById('thr-knob').style.height = thr + "%";
    send();
  };

  // Roll/Pitch (Auto-center)
  const joyZone = document.getElementById('joy-zone');
  const kn = document.getElementById('joy-knob');
  joyZone.ontouchmove = (e) => {
    let r = joyZone.getBoundingClientRect();
    rl = Math.round(((e.touches[0].clientX - r.left) / r.width - 0.5) * 200);
    pt = Math.round(((e.touches[0].clientY - r.top) / r.height - 0.5) * -200);
    kn.style.left = (rl/2 + 50) + "px"; kn.style.top = (-pt/2 + 50) + "px";
    send();
  };
  joyZone.ontouchend = () => { rl = 0; pt = 0; kn.style.left = "50px"; kn.style.top = "50px"; send(); };
</script></body></html>
)rawliteral";

void handleWebSocket(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) {
    last_packet = millis();
    String msg = "";
    for(size_t i=0; i<len; i++) msg += (char)data[i];
    if (msg == "ARM") armed = !armed;
    else if (msg.startsWith("C:")) {
      sscanf(msg.c_str(), "C:%f,%f,%f,%f", &throttle, &roll, &pitch, &yaw);
    }
  }
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!mpu.begin()) while(1);

  int pins[] = {PIN_M1, PIN_M2, PIN_M3, PIN_M4};
  for(int i=0; i<4; i++) {
    ledcSetup(i, FREQ, RES);
    ledcAttachPin(pins[i], i);
  }

  WiFi.softAP(ssid, pass);
  ws.onEvent(handleWebSocket);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *r){ r->send_P(200, "text/html", index_html); });
  server.begin();
}

void loop() {
  updateLED();
  ws.cleanupClients();

  // Failsafe
  if (millis() - last_packet > 1000) { throttle = 0; }

  // MPU Data send to Web
  static uint32_t last_send = 0;
  if (millis() - last_send > 200) {
    sensors_event_t a, g, t; mpu.getEvent(&a, &g, &t);
    String stat = "P:" + String(a.acceleration.x) + " R:" + String(a.acceleration.y) + (armed ? " [ARMED]" : " [OFF]");
    ws.textAll(stat);
    last_send = millis();
  }

  if (armed) {
    // Simple mixer logic
    float base = throttle * 2.55f;
    ledcWrite(0, constrain(base + pitch + roll, 0, 255)); // M1
    ledcWrite(1, constrain(base + pitch - roll, 0, 255)); // M2
    ledcWrite(2, constrain(base - pitch - roll, 0, 255)); // M3
    ledcWrite(3, constrain(base - pitch + roll, 0, 255)); // M4
  } else {
    for(int i=0; i<4; i++) ledcWrite(i, 0);
  }
}
