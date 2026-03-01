#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// ================= WIFI (SoftAP) =================
static const char* AP_SSID = "ESP32 DRONE";
static const char* AP_PASS = "12345678";

// ================= PINS ==========================
static const int PIN_M1 = 21;   
static const int PIN_M2 = 20;   
static const int PIN_M3 = 2;    
static const int PIN_M4 = 3;    
static const int I2C_SDA = 7;
static const int I2C_SCL = 6;
static const int PIN_LED = 8;
static const int LED_ON_LEVEL = 1;

// ================= PWM ===========================
static const int PWM_FREQ = 20000;
static const int PWM_RES_BITS = 8;
static const int PWM_MAX = (1 << PWM_RES_BITS) - 1;

static const int CH_M1 = 0;
static const int CH_M2 = 1;
static const int CH_M3 = 2;
static const int CH_M4 = 3;

static const int MOTOR_MIN_START = 35;
static const int MOTOR_MAX_LIMIT = 240;
static const int THR_RAMP_PER_LOOP = 3;

// ================= LOOP & FAILSAFE =================
static const float LOOP_HZ = 250.0f;
static const uint32_t LOOP_US = (uint32_t)(1000000.0f / LOOP_HZ);
static const float CF_ALPHA = 0.98f;
static const float EXPO = 0.35f;

// FIX: Timeout ကို ပိုတိုးထားပါတယ် (Connection မတည်ငြိမ်ရင်တောင် မရပ်သွားအောင်)
static const uint32_t CMD_TIMEOUT_MS    = 2000;  // 2 Seconds
static const uint32_t DISARM_TIMEOUT_MS = 60000; // 1 Minute idle

// ================= GLOBALS =======================
Adafruit_MPU6050 mpu;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

volatile bool armed = false;
volatile bool killSwitch = false;
enum FlightMode { MODE_ANGLE = 0, MODE_RATE = 1 };
volatile FlightMode mode = MODE_ANGLE;
volatile uint32_t lastCmdMs = 0;

volatile float in_throttle = 0.0f;
volatile float in_roll = 0.0f, in_pitch = 0.0f, in_yaw = 0.0f;
volatile float in_thrLimit = 1.0f;

static float roll_deg = 0, pitch_deg = 0;
static float gyro_bias_x = 0, gyro_bias_y = 0, gyro_bias_z = 0;
static float i_roll = 0, i_pitch = 0, i_yaw = 0;
static float last_err_r = 0, last_err_p = 0, last_err_y = 0;
static int thr_duty_smooth = 0;

// PID GAINS
static float Kp_angle = 4.5f;
static float Kp_rate_rp = 0.09f, Ki_rate_rp = 0.18f, Kd_rate_rp = 0.0025f;
static float Kp_rate_y = 0.12f, Ki_rate_y = 0.10f, Kd_rate_y = 0.0f;

static const float MAX_ANGLE_DEG = 30.0f;
static const float MAX_RATE_RP   = 220.0f;
static const float MAX_RATE_Y    = 180.0f;

// ================= UTIL ==========================
static inline float clampf(float x, float a, float b) { return (x < a) ? a : (x > b) ? b : x; }
static inline float expoCurve(float x, float expo) {
  float ax = fabsf(x);
  return clampf((1.0f - expo) * x + expo * x * ax * ax, -1.0f, 1.0f);
}
static void motorWrite(int ch, int duty) { ledcWrite(ch, constrain(duty, 0, PWM_MAX)); }
static void allMotorsOff() { for(int i=0; i<4; i++) motorWrite(i, 0); thr_duty_smooth = 0; }

// ================= CALIB =========================
static void calibrateGyro() {
  float sx = 0, sy = 0, sz = 0;
  sensors_event_t a, g, t;
  for (int i = 0; i < 400; i++) {
    mpu.getEvent(&a, &g, &t);
    sx += g.gyro.x; sy += g.gyro.y; sz += g.gyro.z;
    delay(2);
  }
  gyro_bias_x = sx / 400.0f; gyro_bias_y = sy / 400.0f; gyro_bias_z = sz / 400.0f;
}

// ================= WEB UI =======================
static const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>C3 DRONE</title>
<style>
:root{--bg:#0b0f14;--panel:#121a23;--line:#1f2a36;--acc:#34d399;--warn:#fb7185;--txt:#e5e7eb;--mut:#94a3b8;}
body{margin:0;background:var(--bg);color:var(--txt);font-family:system-ui;overflow:hidden;touch-action:none}
.top{display:flex;align-items:center;justify-content:space-between;padding:10px 12px;background:#0f172a;border-bottom:1px solid var(--line)}
.pill{padding:6px 10px;border-radius:999px;background:var(--panel);font-size:12px;color:var(--mut)}
.wrap{display:flex;gap:10px;padding:10px;height:calc(100vh - 58px);box-sizing:border-box}
.col{flex:1;display:flex;flex-direction:column;gap:10px}
.card{background:var(--panel);border:1px solid var(--line);border-radius:16px;padding:10px}
.row{display:flex;gap:10px;align-items:center}
button{border:none;border-radius:14px;padding:12px;font-weight:900;background:var(--acc);width:100%}
button.secondary{background:#60a5fa}
button.warn{background:var(--warn);color:white}
button.off{background:#334155;color:#e2e8f0}
.joyArea{display:flex;justify-content:space-between;gap:10px}
.joyBox{flex:1;display:flex;flex-direction:column;align-items:center;gap:5px}
.joyBase{width:160px;height:160px;border-radius:50%;background:rgba(255,255,255,0.05);border:2px solid rgba(52,211,153,0.3);position:relative}
.stick{width:50px;height:50px;border-radius:50%;background:var(--acc);position:absolute;top:55px;left:55px;touch-action:none}
.thrWrap{width:50px;height:220px;border-radius:15px;background:#000;position:relative;overflow:hidden;border:1px solid var(--line)}
.thrFill{position:absolute;bottom:0;width:100%;background:var(--acc);opacity:0.5}
.thrKnob{width:100%;height:40px;background:rgba(255,255,255,0.2);position:absolute;bottom:0;display:flex;align-items:center;justify-content:center;font-size:12px}
</style></head>
<body>
<div class="top"><div>C3 DRONE</div><div class="pill" id="stat">DISCONNECTED</div></div>
<div class="wrap"><div class="col">
  <div class="card">
    <div class="row">
      <button id="armBtn" class="warn" onclick="toggleArm()">ARM</button>
      <button id="killBtn" class="off" onclick="toggleKill()">KILL</button>
    </div>
    <div style="height:8px"></div>
    <button id="modeBtn" class="secondary" onclick="toggleMode()">MODE: ANGLE</button>
    <div style="height:8px"></div>
    <button class="off" onclick="sendCmd('CAL')">CALIBRATE GYRO</button>
  </div>
  <div class="card">
    <div class="joyArea">
      <div class="joyBox">
        <div class="thrWrap" id="thrWrap"><div class="thrFill" id="thrFill"></div><div class="thrKnob" id="thrKnob">0%</div></div>
        <div class="joyBase" id="yawBase"><div class="stick" id="yawStick"></div></div>
      </div>
      <div class="joyBox">
        <div class="joyBase" id="prBase"><div class="stick" id="prStick"></div></div>
        <div class="pill" id="dbg">Ready</div>
      </div>
    </div>
  </div>
</div></div>
<script>
let ws, armed=false, thr=0, roll=0, pitch=0, yaw=0;
function connect(){
  ws=new WebSocket(`ws://${location.host}/ws`);
  ws.onopen=()=>{document.getElementById('stat').innerText="CONNECTED";};
  ws.onclose=()=>{document.getElementById('stat').innerText="RECONNECTING...";setTimeout(connect,1000);};
  ws.onmessage=(e)=>{document.getElementById('dbg').innerText=e.data;};
}
connect();
function sendCtrl(){ if(ws&&ws.readyState==1) ws.send(`C:${Math.round(thr*1000)},${Math.round(roll*1000)},${Math.round(pitch*1000)},${Math.round(yaw*1000)},1000`); }
setInterval(sendCtrl, 40); // 25Hz - ပိုပြီး Stable ဖြစ်အောင်
function sendCmd(s){ if(ws&&ws.readyState==1) ws.send(s); }
function toggleArm(){ armed=!armed; document.getElementById('armBtn').className=armed?"":"warn"; document.getElementById('armBtn').innerText=armed?"ARMED":"ARM"; sendCmd(armed?"ARM":"DISARM"); }
function toggleKill(){ sendCmd("KILL1"); }
function toggleMode(){ sendCmd("MODE_TGL"); }

// Throttle Control
const tw=document.getElementById('thrWrap'), tk=document.getElementById('thrKnob'), tf=document.getElementById('thrFill');
tw.ontouchmove=(e)=>{
  e.preventDefault(); let r=tw.getBoundingClientRect();
  let y=1-((e.touches[0].clientY-r.top)/r.height);
  thr=Math.max(0,Math.min(1,y));
  tk.style.bottom=(thr*90)+"%"; tf.style.height=(thr*100)+"%"; tk.innerText=Math.round(thr*100)+"%";
};
// Joysticks Logic (simplified)
function setupJoy(id, cb){
  const b=document.getElementById(id), s=b.children[0];
  b.ontouchmove=(e)=>{
    e.preventDefault(); let r=b.getBoundingClientRect();
    let dx=(e.touches[0].clientX-r.left-80)/40, dy=(e.touches[0].clientY-r.top-80)/40;
    s.style.transform=`translate(${dx*10}px,${dy*10}px)`; cb(dx,dy);
  };
  b.ontouchend=()=>{ s.style.transform="translate(0,0)"; cb(0,0); };
}
setupJoy('yawBase', (x,y)=>{ yaw=clamp(x,-1,1); });
setupJoy('prBase', (x,y)=>{ roll=clamp(x,-1,1); pitch=clamp(-y,-1,1); });
function clamp(v,min,max){ return Math.max(min,Math.min(max,v)); }
</script></body></html>
)rawliteral";

// ================= WS HANDLER ====================
static void onWsEvent(AsyncWebSocket *server_, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) {
    lastCmdMs = millis(); 
    std::string msg((char*)data, len);
    if (msg == "ARM") { armed = true; killSwitch = false; }
    else if (msg == "DISARM") { armed = false; }
    else if (msg == "KILL1") { armed = false; killSwitch = true; }
    else if (msg == "CAL") { calibrateGyro(); }
    else if (msg.substr(0,2) == "C:") {
      int t, r, p, y, lim;
      if (sscanf(msg.c_str(), "C:%d,%d,%d,%d,%d", &t, &r, &p, &y, &lim) == 5) {
        in_throttle = t/1000.0f; in_roll = expoCurve(r/1000.0f, EXPO);
        in_pitch = expoCurve(p/1000.0f, EXPO); in_yaw = y/1000.0f;
      }
    }
  }
}

// ================= SETUP =========================
void setup() {
  pinMode(PIN_LED, OUTPUT);
  Wire.begin(I2C_SDA, I2C_SCL); Wire.setClock(400000);
  if (!mpu.begin()) while (1);
  
  for(int i=0; i<4; i++) {
    ledcSetup(i, PWM_FREQ, PWM_RES_BITS);
    ledcAttachPin((i==0?PIN_M1:i==1?PIN_M2:i==2?PIN_M3:PIN_M4), i);
  }
  allMotorsOff();

  WiFi.softAP(AP_SSID, AP_PASS);
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *r){ r->send_P(200, "text/html", index_html); });
  server.begin();
  calibrateGyro();
}

// ================= LOOP ==========================
void loop() {
  ws.cleanupClients();
  uint32_t nowMs = millis();

  // FIX: FAILSAFE - Connection ပျောက်သွားရင် ချက်ချင်းမရပ်ဘဲ ၂ စက္ကန့် စောင့်မယ်
  if (armed) {
    if (nowMs - lastCmdMs > CMD_TIMEOUT_MS) {
      in_throttle = 0; // ၂ စက္ကန့်ကျော်ရင်မှ ပိတ်မယ်
      if (nowMs - lastCmdMs > 5000) armed = false; // ၅ စက္ကန့်ကျော်ရင် Disarm လုပ်မယ်
    }
  }

  static uint32_t lastUs = micros();
  uint32_t nowUs = micros();
  if (nowUs - lastUs < LOOP_US) return;
  float dt = (nowUs - lastUs) / 1000000.0f;
  lastUs = nowUs;

  sensors_event_t a, g, temp; mpu.getEvent(&a, &g, &temp);
  float gx = (g.gyro.x - gyro_bias_x) * 180.0f / PI;
  float gy = (g.gyro.y - gyro_bias_y) * 180.0f / PI;
  float gz = (g.gyro.z - gyro_bias_z) * 180.0f / PI;

  float accRoll = atan2f(a.acceleration.y, a.acceleration.z) * 180.0f / PI;
  float accPitch = atan2f(-a.acceleration.x, sqrtf(a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z)) * 180.0f / PI;
  roll_deg = 0.98f * (roll_deg + gx * dt) + 0.02f * accRoll;
  pitch_deg = 0.98f * (pitch_deg + gy * dt) + 0.02f * accPitch;

  float target_r = (mode == MODE_ANGLE) ? clampf((in_roll * 30.0f - roll_deg) * 4.5f, -220, 220) : in_roll * 220.0f;
  float target_p = (mode == MODE_ANGLE) ? clampf((in_pitch * 30.0f - pitch_deg) * 4.5f, -220, 220) : in_pitch * 220.0f;
  float target_y = in_yaw * 180.0f;

  float err_r = target_r - gx; i_roll = clampf(i_roll + err_r * dt, -200, 200);
  float out_r = 0.09f * err_r + 0.18f * i_roll + 0.0025f * ((err_r - last_err_r)/dt); last_err_r = err_r;

  float err_p = target_p - gy; i_pitch = clampf(i_pitch + err_p * dt, -200, 200);
  float out_p = 0.09f * err_p + 0.18f * i_pitch + 0.0025f * ((err_p - last_err_p)/dt); last_err_p = err_p;

  float err_y = target_y - gz; i_yaw = clampf(i_yaw + err_y * dt, -200, 200);
  float out_y = 0.12f * err_y + 0.10f * i_yaw;

  int targetDuty = (armed && in_throttle > 0.01f) ? (MOTOR_MIN_START + (int)(in_throttle * (MOTOR_MAX_LIMIT - MOTOR_MIN_START))) : 0;
  if (targetDuty > thr_duty_smooth) thr_duty_smooth = fmin(targetDuty, thr_duty_smooth + 3);
  else thr_duty_smooth = fmax(targetDuty, thr_duty_smooth - 3);

  if (armed && !killSwitch) {
    float b = (float)thr_duty_smooth;
    motorWrite(CH_M1, (int)(b - out_p + out_r - out_y));
    motorWrite(CH_M2, (int)(b - out_p - out_r + out_y));
    motorWrite(CH_M3, (int)(b + out_p - out_r - out_y));
    motorWrite(CH_M4, (int)(b + out_p + out_r + out_y));
    digitalWrite(PIN_LED, LED_ON_LEVEL);
  } else {
    allMotorsOff();
    digitalWrite(PIN_LED, !LED_ON_LEVEL);
  }
}
