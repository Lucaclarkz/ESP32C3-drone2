/*
  READY-TO-FLY ESP32-C3 SuperMini Plus + MPU6050 Quad (X frame)
  - WiFi SoftAP + Web RC Remote UI:
      Left: Throttle (latching slider) + Yaw (auto-center stick)
      Right: Pitch/Roll (auto-center stick)
  - Flight Modes: ANGLE (self-level) / RATE
  - Safety: ARM, KILL, failsafe timeout, disarm timeout
  - MPU6050 gyro calibration on boot + CAL button
  - Motor mixing with headroom clamp (prevents saturation flip)
  - LEDC PWM for motors

  USER PINS:
    M1 GPIO21 (Front Left)
    M2 GPIO20 (Front Right)
    M3 GPIO2  (Rear Right)
    M4 GPIO3  (Rear Left)

  MPU6050:
    SDA GPIO7, SCL GPIO6, VCC 3.3V, GND GND
*/

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// =================== WIFI ===================
static const char* AP_SSID = "C3-DRONE";
static const char* AP_PASS = "12345678";

// =================== PINS ===================
static const int PIN_M1 = 21;   // Front Left
static const int PIN_M2 = 20;   // Front Right
static const int PIN_M3 = 2;    // Rear Right
static const int PIN_M4 = 3;    // Rear Left
static const int I2C_SDA = 7;
static const int I2C_SCL = 6;

// =================== PWM ===================
static const int PWM_FREQ = 20000;   // 20kHz
static const int PWM_RES_BITS = 8;   // 0..255
static const int PWM_MAX = (1 << PWM_RES_BITS) - 1;

static const int CH_M1 = 0;
static const int CH_M2 = 1;
static const int CH_M3 = 2;
static const int CH_M4 = 3;

// Motor behavior (coreless tune)
static const int MOTOR_MIN_START = 35;     // overcome friction
static const int MOTOR_MAX_LIMIT = 240;    // keep headroom under 255
static const int THR_RAMP_PER_LOOP = 3;    // smooth throttle ramp

// =================== LOOP ===================
static const float LOOP_HZ = 250.0f;
static const uint32_t LOOP_US = (uint32_t)(1000000.0f / LOOP_HZ);

// Angle filter
static const float CF_ALPHA = 0.98f;

// Stick expo
static const float EXPO = 0.35f;

// Safety timeouts
static const uint32_t CMD_TIMEOUT_MS = 350;       // cut throttle if no packets
static const uint32_t DISARM_TIMEOUT_MS = 2000;   // auto-disarm if no packets

// =================== FLIGHT MODES ===================
enum FlightMode { MODE_ANGLE = 0, MODE_RATE = 1 };

// =================== PID GAINS ===================
// Outer loop (angle -> desired rate)
static float Kp_angle = 4.5f;           // deg error -> deg/s target

// Inner loop (rate PID)
static float Kp_rate_rp = 0.09f;
static float Ki_rate_rp = 0.18f;
static float Kd_rate_rp = 0.0025f;

static float Kp_rate_y = 0.12f;
static float Ki_rate_y = 0.10f;
static float Kd_rate_y = 0.0f;

// Limits
static const float MAX_ANGLE_DEG = 30.0f;
static const float MAX_RATE_RP = 220.0f; // deg/s
static const float MAX_RATE_Y  = 180.0f; // deg/s

// =================== GLOBALS ===================
Adafruit_MPU6050 mpu;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

volatile bool armed = false;
volatile bool killSwitch = false;
volatile FlightMode mode = MODE_ANGLE;
volatile uint32_t lastCmdMs = 0;

// inputs
volatile float in_throttle = 0.0f;  // 0..1
volatile float in_roll = 0.0f;      // -1..1
volatile float in_pitch = 0.0f;     // -1..1
volatile float in_yaw = 0.0f;       // -1..1
volatile float in_thrLimit = 1.0f;  // 0.2..1.0

// state
static float roll_deg = 0, pitch_deg = 0;
static float gyro_bias_x = 0, gyro_bias_y = 0, gyro_bias_z = 0;

// PID integrators & prev error
static float i_roll = 0, i_pitch = 0, i_yaw = 0;
static float last_err_r = 0, last_err_p = 0, last_err_y = 0;

// throttle smoothing
static int thr_duty_smooth = 0;

// =================== UTIL ===================
static inline float clampf(float x, float a, float b) {
  return (x < a) ? a : (x > b) ? b : x;
}
static inline float expoCurve(float x, float expo) {
  float ax = fabsf(x);
  float y = (1.0f - expo) * x + expo * x * ax * ax;
  return clampf(y, -1.0f, 1.0f);
}

static void motorWrite(int ch, int duty) {
  duty = constrain(duty, 0, PWM_MAX);
  ledcWrite(ch, duty);
}
static void allMotorsOff() {
  motorWrite(CH_M1, 0);
  motorWrite(CH_M2, 0);
  motorWrite(CH_M3, 0);
  motorWrite(CH_M4, 0);
}

// =================== CALIBRATION ===================
static void calibrateGyro(uint16_t samples = 800) {
  float sx = 0, sy = 0, sz = 0;
  sensors_event_t a, g, t;
  for (uint16_t i = 0; i < samples; i++) {
    mpu.getEvent(&a, &g, &t);
    sx += g.gyro.x;
    sy += g.gyro.y;
    sz += g.gyro.z;
    delay(2);
  }
  gyro_bias_x = sx / samples;
  gyro_bias_y = sy / samples;
  gyro_bias_z = sz / samples;

  i_roll = i_pitch = i_yaw = 0;
  last_err_r = last_err_p = last_err_y = 0;
}

// =================== WEB UI (RC remote style) ===================
static const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
<title>C3 DRONE</title>
<style>
  :root{
    --bg:#0b0f14; --panel:#121a23; --line:#1f2a36; --acc:#34d399; --warn:#fb7185;
    --txt:#e5e7eb; --mut:#94a3b8;
  }
  body{margin:0;background:var(--bg);color:var(--txt);font-family:system-ui,-apple-system,Segoe UI,Roboto; overflow:hidden; touch-action:none;}
  .top{display:flex;align-items:center;justify-content:space-between;padding:10px 12px;background:linear-gradient(180deg,#0f172a,#0b0f14);border-bottom:1px solid var(--line);}
  .title{font-weight:800;letter-spacing:1px;}
  .pill{padding:6px 10px;border-radius:999px;background:var(--panel);border:1px solid var(--line);font-size:12px;color:var(--mut);}
  .wrap{display:flex;gap:10px;padding:10px;height:calc(100vh - 58px);box-sizing:border-box;}
  .col{flex:1;display:flex;flex-direction:column;gap:10px;}
  .card{background:var(--panel);border:1px solid var(--line);border-radius:16px;padding:10px;}
  .row{display:flex;gap:10px;align-items:center;justify-content:space-between;}
  button{border:none;border-radius:14px;padding:12px;font-weight:900;color:#0b0f14;background:var(--acc);width:100%;}
  button.secondary{background:#60a5fa;}
  button.warn{background:var(--warn);color:white;}
  button.off{background:#334155;color:#e2e8f0;}
  .small{font-size:12px;color:var(--mut);}

  .joyArea{display:flex;justify-content:space-between;gap:10px;}
  .joyBox{flex:1;display:flex;flex-direction:column;gap:10px;align-items:center;justify-content:center;}
  .joyBase{
    width:170px;height:170px;border-radius:50%;
    background:radial-gradient(circle at 30% 30%, rgba(255,255,255,0.06), rgba(255,255,255,0.02));
    border:2px solid rgba(52,211,153,0.35);
    position:relative;
  }
  .stick{
    width:60px;height:60px;border-radius:50%;
    background:radial-gradient(circle at 30% 30%, rgba(52,211,153,0.95), rgba(52,211,153,0.55));
    position:absolute;top:55px;left:55px;
    box-shadow:0 0 18px rgba(52,211,153,0.6);
    touch-action:none;
  }

  .thrWrap{width:54px;height:220px;border-radius:18px;background:#0b1220;border:1px solid var(--line);position:relative;overflow:hidden;}
  .thrFill{position:absolute;bottom:0;left:0;right:0;height:0;background:linear-gradient(180deg, rgba(52,211,153,0.1), rgba(52,211,153,0.6));}
  .thrKnob{
    width:54px;height:44px;border-radius:18px;background:rgba(255,255,255,0.10);border:1px solid rgba(255,255,255,0.16);
    position:absolute;left:0;bottom:0;display:flex;align-items:center;justify-content:center;color:var(--txt);font-weight:900;touch-action:none;
  }
  .grid{display:grid;grid-template-columns:1fr 1fr;gap:10px;}
  input[type="range"]{width:100%;}
</style>
</head>
<body>
  <div class="top">
    <div class="title">C3 DRONE</div>
    <div class="pill" id="stat">WS: --</div>
  </div>

  <div class="wrap">
    <div class="col">
      <div class="card">
        <div class="row">
          <button id="armBtn" class="warn" onclick="toggleArm()">ARM</button>
          <button id="killBtn" class="off" onclick="toggleKill()">KILL</button>
        </div>
        <div style="height:10px"></div>
        <div class="row">
          <button id="modeBtn" class="secondary" onclick="toggleMode()">MODE: ANGLE</button>
        </div>
        <div style="height:10px"></div>
        <div class="row">
          <button class="off" onclick="sendCmd('CAL')">CALIBRATE</button>
        </div>
        <div style="height:10px"></div>
        <div class="grid">
          <div class="card">
            <div class="small">Throttle Limit</div>
            <input id="thrLim" type="range" min="20" max="100" value="100" oninput="thrLimitChanged(this.value)">
            <div class="small"><span id="thrLimVal">100</span>%</div>
          </div>
          <div class="card">
            <div class="small">Status</div>
            <div class="small" id="dbg">---</div>
          </div>
        </div>
      </div>

      <div class="card">
        <div class="joyArea">
          <div class="joyBox">
            <div class="small">THROTTLE</div>
            <div class="thrWrap" id="thrWrap">
              <div class="thrFill" id="thrFill"></div>
              <div class="thrKnob" id="thrKnob">0%</div>
            </div>

            <div class="small">YAW</div>
            <div class="joyBase" id="yawBase">
              <div class="stick" id="yawStick"></div>
            </div>
          </div>

          <div class="joyBox">
            <div class="small">PITCH / ROLL</div>
            <div class="joyBase" id="prBase">
              <div class="stick" id="prStick"></div>
            </div>
            <div class="small">Right stick auto-centers</div>
          </div>
        </div>
      </div>
    </div>
  </div>

<script>
let ws;
let armed=false, killed=false, mode=0;
let thr=0; // 0..1 latching
let yaw=0, roll=0, pitch=0; // -1..1
let thrLimit=1.0;

function connectWS(){
  ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onopen = ()=>{ document.getElementById('stat').innerText="WS: OK"; };
  ws.onclose= ()=>{ document.getElementById('stat').innerText="WS: RECONNECT"; setTimeout(connectWS,700); };
  ws.onmessage = (e)=>{ document.getElementById('dbg').innerText = e.data; };
}
connectWS();

function sendCmd(s){
  if(ws && ws.readyState===1) ws.send(s);
}
function sendControl(){
  if(ws && ws.readyState===1){
    ws.send(`C:${Math.round(thr*1000)},${Math.round(roll*1000)},${Math.round(pitch*1000)},${Math.round(yaw*1000)},${Math.round(thrLimit*1000)}`);
  }
}

function toggleArm(){
  armed=!armed;
  const b=document.getElementById('armBtn');
  if(armed){ b.innerText="ARMED"; b.className=""; sendCmd("ARM"); }
  else { b.innerText="ARM"; b.className="warn"; sendCmd("DISARM"); }
}
function toggleKill(){
  killed=!killed;
  const b=document.getElementById('killBtn');
  if(killed){ b.innerText="KILL ON"; b.className="warn"; sendCmd("KILL1"); }
  else { b.innerText="KILL"; b.className="off"; sendCmd("KILL0"); }
}
function toggleMode(){
  mode = (mode===0)?1:0;
  const b=document.getElementById('modeBtn');
  if(mode===0){ b.innerText="MODE: ANGLE"; sendCmd("MODE0"); }
  else { b.innerText="MODE: RATE"; sendCmd("MODE1"); }
}
function thrLimitChanged(v){
  document.getElementById('thrLimVal').innerText=v;
  thrLimit = Math.max(0.2, Math.min(1.0, v/100));
  sendControl();
}

/* Throttle slider (latching) */
(function(){
  const wrap = document.getElementById('thrWrap');
  const knob = document.getElementById('thrKnob');
  const fill = document.getElementById('thrFill');

  function setThrFromY(clientY){
    const r = wrap.getBoundingClientRect();
    let y = clientY - r.top;
    y = Math.max(0, Math.min(r.height, y));
    thr = 1.0 - (y / r.height);
    thr = Math.max(0, Math.min(1, thr));
    const h = Math.round(thr * r.height);
    fill.style.height = `${h}px`;
    knob.style.bottom = `${h - knob.offsetHeight/2}px`;
    knob.innerText = `${Math.round(thr*100)}%`;
    sendControl();
  }

  wrap.addEventListener('touchstart', (e)=>{ setThrFromY(e.touches[0].clientY); }, {passive:false});
  wrap.addEventListener('touchmove',  (e)=>{ e.preventDefault(); setThrFromY(e.touches[0].clientY); }, {passive:false});
})();

/* Yaw joystick (auto-center) */
(function(){
  const base=document.getElementById('yawBase');
  const stick=document.getElementById('yawStick');
  const R=50;

  function setYawFromTouch(t){
    const r=base.getBoundingClientRect();
    let x=t.clientX - r.left - r.width/2;
    x=Math.max(-R, Math.min(R, x));
    stick.style.transform=`translate(${x}px, 0px)`;
    yaw = (x/R);
    sendControl();
  }
  base.ontouchstart=(e)=>{ setYawFromTouch(e.touches[0]); };
  base.ontouchmove=(e)=>{ e.preventDefault(); setYawFromTouch(e.touches[0]); };
  base.ontouchend=()=>{ stick.style.transform=`translate(0px,0px)`; yaw=0; sendControl(); };
})();

/* Pitch/Roll joystick (auto-center) */
(function(){
  const base=document.getElementById('prBase');
  const stick=document.getElementById('prStick');
  const R=55;

  function setPR(t){
    const r=base.getBoundingClientRect();
    let x=t.clientX - r.left - r.width/2;
    let y=t.clientY - r.top  - r.height/2;
    const mag=Math.sqrt(x*x+y*y);
    if(mag>R){ x = x*(R/mag); y = y*(R/mag); }
    stick.style.transform=`translate(${x}px, ${y}px)`;
    roll = x/R;
    pitch = -y/R;
    sendControl();
  }
  base.ontouchstart=(e)=>{ setPR(e.touches[0]); };
  base.ontouchmove=(e)=>{ e.preventDefault(); setPR(e.touches[0]); };
  base.ontouchend=()=>{ stick.style.transform=`translate(0px,0px)`; roll=0; pitch=0; sendControl(); };
})();
</script>
</body>
</html>
)rawliteral";

// =================== WS HANDLER ===================
static void onWsEvent(AsyncWebSocket *server_, AsyncWebSocketClient *client,
                      AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type != WS_EVT_DATA || len == 0) return;

  String msg;
  msg.reserve(len + 1);
  for (size_t i = 0; i < len; i++) msg += (char)data[i];

  lastCmdMs = millis();

  if (msg == "ARM") {
    killSwitch = false;
    armed = true;
    i_roll = i_pitch = i_yaw = 0;
    last_err_r = last_err_p = last_err_y = 0;
    thr_duty_smooth = 0;
    client->text("ARMED");
    return;
  }
  if (msg == "DISARM") {
    armed = false;
    client->text("DISARMED");
    return;
  }
  if (msg == "KILL1") {
    killSwitch = true;
    armed = false;
    client->text("KILL ON");
    return;
  }
  if (msg == "KILL0") {
    killSwitch = false;
    client->text("KILL OFF");
    return;
  }
  if (msg == "MODE0") { mode = MODE_ANGLE; client->text("MODE ANGLE"); return; }
  if (msg == "MODE1") { mode = MODE_RATE;  client->text("MODE RATE");  return; }

  if (msg == "CAL") {
    armed = false;
    client->text("CAL...");
    calibrateGyro();
    client->text("CAL OK");
    return;
  }

  if (msg.startsWith("C:")) {
    int t, r, p, y, lim;
    if (sscanf(msg.c_str(), "C:%d,%d,%d,%d,%d", &t, &r, &p, &y, &lim) == 5) {
      float thr = clampf(t / 1000.0f, 0.0f, 1.0f);
      float rr  = clampf(r / 1000.0f, -1.0f, 1.0f);
      float pp  = clampf(p / 1000.0f, -1.0f, 1.0f);
      float yy  = clampf(y / 1000.0f, -1.0f, 1.0f);
      float ll  = clampf(lim / 1000.0f, 0.2f, 1.0f);

      rr = expoCurve(rr, EXPO);
      pp = expoCurve(pp, EXPO);
      yy = expoCurve(yy, EXPO);

      in_throttle = thr;
      in_roll = rr;
      in_pitch = pp;
      in_yaw = yy;
      in_thrLimit = ll;
    }
  }
}

// =================== SETUP ===================
void setup() {
  Serial.begin(115200);
  delay(100);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  if (!mpu.begin()) {
    Serial.println("MPU6050 init failed!");
    while (1) delay(100);
  }

  // IMU settings for quad
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // LEDC PWM
  ledcSetup(CH_M1, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(CH_M2, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(CH_M3, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(CH_M4, PWM_FREQ, PWM_RES_BITS);

  ledcAttachPin(PIN_M1, CH_M1);
  ledcAttachPin(PIN_M2, CH_M2);
  ledcAttachPin(PIN_M3, CH_M3);
  ledcAttachPin(PIN_M4, CH_M4);

  allMotorsOff();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
    req->send_P(200, "text/html", index_html);
  });
  server.begin();

  delay(400);
  calibrateGyro(); // boot calibration (keep still)

  lastCmdMs = millis();
}

// =================== LOOP ===================
void loop() {
  ws.cleanupClients();

  uint32_t nowMs = millis();
  if (armed && (nowMs - lastCmdMs > CMD_TIMEOUT_MS)) {
    in_throttle = 0.0f;
    in_roll = in_pitch = in_yaw = 0.0f;
  }
  if (armed && (nowMs - lastCmdMs > DISARM_TIMEOUT_MS)) {
    armed = false;
  }
  if (killSwitch) armed = false;

  static uint32_t lastUs = micros();
  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastUs) < LOOP_US) return;
  float dt = (nowUs - lastUs) / 1000000.0f;
  lastUs = nowUs;
  if (dt <= 0) dt = 0.004f;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Gyro corrected (deg/s)
  float gx = (g.gyro.x - gyro_bias_x) * 180.0f / PI;
  float gy = (g.gyro.y - gyro_bias_y) * 180.0f / PI;
  float gz = (g.gyro.z - gyro_bias_z) * 180.0f / PI;

  // Acc angles
  float accRoll  = atan2f(a.acceleration.y, a.acceleration.z) * 180.0f / PI;
  float accPitch = atan2f(-a.acceleration.x,
                          sqrtf(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0f / PI;

  // Complementary filter
  roll_deg  = CF_ALPHA * (roll_deg  + gx * dt) + (1.0f - CF_ALPHA) * accRoll;
  pitch_deg = CF_ALPHA * (pitch_deg + gy * dt) + (1.0f - CF_ALPHA) * accPitch;

  // Desired rates
  float des_rate_roll = 0, des_rate_pitch = 0, des_rate_yaw = 0;

  if (mode == MODE_ANGLE) {
    float des_ang_roll  = in_roll  * MAX_ANGLE_DEG;
    float des_ang_pitch = in_pitch * MAX_ANGLE_DEG;

    des_rate_roll  = clampf((des_ang_roll  - roll_deg)  * Kp_angle, -MAX_RATE_RP, MAX_RATE_RP);
    des_rate_pitch = clampf((des_ang_pitch - pitch_deg) * Kp_angle, -MAX_RATE_RP, MAX_RATE_RP);
  } else {
    des_rate_roll  = in_roll  * MAX_RATE_RP;
    des_rate_pitch = in_pitch * MAX_RATE_RP;
  }
  des_rate_yaw = in_yaw * MAX_RATE_Y;

  // Rate PID (roll)
  float err_r = des_rate_roll - gx;
  i_roll = clampf(i_roll + err_r * dt, -200.0f, 200.0f);
  float d_r = (err_r - last_err_r) / dt; last_err_r = err_r;
  float out_r = Kp_rate_rp * err_r + Ki_rate_rp * i_roll + Kd_rate_rp * d_r;

  // Rate PID (pitch)
  float err_p = des_rate_pitch - gy;
  i_pitch = clampf(i_pitch + err_p * dt, -200.0f, 200.0f);
  float d_p = (err_p - last_err_p) / dt; last_err_p = err_p;
  float out_p = Kp_rate_rp * err_p + Ki_rate_rp * i_pitch + Kd_rate_rp * d_p;

  // Rate PID (yaw)
  float err_y = des_rate_yaw - gz;
  i_yaw = clampf(i_yaw + err_y * dt, -200.0f, 200.0f);
  float d_y = (err_y - last_err_y) / dt; last_err_y = err_y;
  float out_y = Kp_rate_y * err_y + Ki_rate_y * i_yaw + Kd_rate_y * d_y;

  // Throttle -> duty
  float thr = clampf(in_throttle, 0.0f, 1.0f) * clampf(in_thrLimit, 0.2f, 1.0f);

  int targetDuty = 0;
  if (armed && thr > 0.001f) {
    int usable = MOTOR_MAX_LIMIT - MOTOR_MIN_START;
    targetDuty = MOTOR_MIN_START + (int)(thr * usable);
  } else {
    targetDuty = 0;
  }

  // Throttle ramp
  if (targetDuty > thr_duty_smooth) thr_duty_smooth = min(targetDuty, thr_duty_smooth + THR_RAMP_PER_LOOP);
  else thr_duty_smooth = max(targetDuty, thr_duty_smooth - THR_RAMP_PER_LOOP);

  float base = (float)thr_duty_smooth;

  // If yaw direction is reversed on your build, change yawSign to -1.0f
  const float yawSign = 1.0f;
  float yawTerm = yawSign * out_y;

  // X-frame mix (M1 FL, M2 FR, M3 RR, M4 RL)
  float m1 = base - out_p + out_r - yawTerm;
  float m2 = base - out_p - out_r + yawTerm;
  float m3 = base + out_p - out_r - yawTerm;
  float m4 = base + out_p + out_r + yawTerm;

  // Prevent saturation: shift down if any exceeds MOTOR_MAX_LIMIT
  float maxOut = fmaxf(fmaxf(m1, m2), fmaxf(m3, m4));
  if (maxOut > MOTOR_MAX_LIMIT) {
    float shift = maxOut - MOTOR_MAX_LIMIT;
    m1 -= shift; m2 -= shift; m3 -= shift; m4 -= shift;
  }

  m1 = clampf(m1, 0, PWM_MAX);
  m2 = clampf(m2, 0, PWM_MAX);
  m3 = clampf(m3, 0, PWM_MAX);
  m4 = clampf(m4, 0, PWM_MAX);

  if (!armed || killSwitch) {
    allMotorsOff();
  } else {
    motorWrite(CH_M1, (int)m1);
    motorWrite(CH_M2, (int)m2);
    motorWrite(CH_M3, (int)m3);
    motorWrite(CH_M4, (int)m4);
  }

  // Debug
  static uint32_t dbgMs = 0;
  if (nowMs - dbgMs > 200) {
    dbgMs = nowMs;
    char buf[96];
    snprintf(buf, sizeof(buf), "ARM:%d MODE:%c R:%.1f P:%.1f",
             armed ? 1 : 0, (mode == MODE_ANGLE) ? 'A' : 'R', roll_deg, pitch_deg);
    ws.textAll(buf);
  }
}
