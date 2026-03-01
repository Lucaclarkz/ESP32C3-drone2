/*
  READY-TO-FLY (Brushed) ESP32-C3 SuperMini Plus + MPU6050 Quad (X frame)
  Web RC Control (AsyncWebServer + WebSocket) + Live IMU telemetry
  PINS:
    M1 GPIO21 (Front Left)
    M2 GPIO20 (Front Right)
    M3 GPIO10 (Back Right)
    M4 GPIO5  (Back Left)
    MPU6050: SCL GPIO6, SDA GPIO7
    LED: GPIO8  (Power blink, WiFi connected solid)
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
static const int PIN_M1 = 21;     // Front Left
static const int PIN_M2 = 20;     // Front Right
static const int PIN_M3 = 10;     // Back Right
static const int PIN_M4 = 5;      // Back Left
static const int I2C_SCL = 6;
static const int I2C_SDA = 7;
static const int PIN_LED = 8;     // Status LED

// =================== PWM ===================
static const int PWM_FREQ = 20000;      // 20kHz for brushed
static const int PWM_RES_BITS = 8;      // 0..255
static const int PWM_MAX = (1 << PWM_RES_BITS) - 1;
static const int CH_M1 = 0;
static const int CH_M2 = 1;
static const int CH_M3 = 2;
static const int CH_M4 = 3;

// Brushed tune
static const int MOTOR_MIN_START = 35;  // start spinning threshold
static const int MOTOR_MAX_LIMIT = 240; // headroom below 255
static const int THR_RAMP_PER_LOOP = 3; // throttle smoothing step

// =================== LOOP ===================
static const float LOOP_HZ = 250.0f;
static const uint32_t LOOP_US = (uint32_t)(1000000.0f / LOOP_HZ);

// Complementary filter
static const float CF_ALPHA = 0.98f;

// Stick expo
static const float EXPO = 0.35f;

// Safety
static const uint32_t CMD_TIMEOUT_MS    = 350;   // if no WS control packets -> neutral
static const uint32_t DISARM_TIMEOUT_MS = 2000;  // auto disarm if no packets
static const uint32_t LED_BLINK_MS      = 350;

// Flight modes
enum FlightMode { MODE_ANGLE = 0, MODE_RATE = 1 };

// =================== PID GAINS (START VALUES; tune for your frame) ===================
// ANGLE outer loop: angle error -> rate target
static float Kp_angle = 4.5f;

// RATE inner loop (deg/s)
static float Kp_rate_rp = 0.09f;
static float Ki_rate_rp = 0.18f;
static float Kd_rate_rp = 0.0025f;

static float Kp_rate_y  = 0.12f;
static float Ki_rate_y  = 0.10f;
static float Kd_rate_y  = 0.0f;

// Limits
static const float MAX_ANGLE_DEG = 30.0f;
static const float MAX_RATE_RP   = 220.0f;
static const float MAX_RATE_Y    = 180.0f;

// =================== GLOBALS ===================
Adafruit_MPU6050 mpu;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// RC state
volatile bool armed = false;
volatile bool killSwitch = false;
volatile FlightMode mode = MODE_ANGLE;
volatile uint32_t lastCmdMs = 0;

volatile float in_throttle = 0.0f; // 0..1
volatile float in_roll     = 0.0f; // -1..1
volatile float in_pitch    = 0.0f; // -1..1
volatile float in_yaw      = 0.0f; // -1..1
volatile float in_thrLimit = 1.0f; // 0.2..1.0

// IMU state
static float roll_deg = 0, pitch_deg = 0;
static float gyro_bias_x = 0, gyro_bias_y = 0, gyro_bias_z = 0;

// PID integrators & prev errors
static float i_roll = 0, i_pitch = 0, i_yaw = 0;
static float last_err_r = 0, last_err_p = 0, last_err_y = 0;

// throttle smoothing duty
static int thr_duty_smooth = 0;

// telemetry (for UI)
static float t_gx=0, t_gy=0, t_gz=0;
static float t_ax=0, t_ay=0, t_az=0;

// =================== UTILS ===================
static inline float clampf(float x, float a, float b) { return (x < a) ? a : (x > b) ? b : x; }
static inline float expoCurve(float x, float expo) {
  float ax = fabsf(x);
  float y = (1.0f - expo) * x + expo * x * ax * ax;
  return clampf(y, -1.0f, 1.0f);
}
static void motorWrite(int ch, int duty) { duty = constrain(duty, 0, PWM_MAX); ledcWrite(ch, duty); }
static void allMotorsOff() { motorWrite(CH_M1,0); motorWrite(CH_M2,0); motorWrite(CH_M3,0); motorWrite(CH_M4,0); }

// =================== LED STATUS ===================
static void ledPowerBlinkTask() {
  static uint32_t last = 0;
  static bool s = false;
  if (millis() - last >= LED_BLINK_MS) {
    last = millis();
    s = !s;
    digitalWrite(PIN_LED, s ? HIGH : LOW);
  }
}
static void ledWifiConnectedOn() { digitalWrite(PIN_LED, HIGH); }

// =================== CALIBRATION ===================
static void calibrateGyro(uint16_t samples = 900) {
  float sx=0, sy=0, sz=0;
  sensors_event_t a,g,t;
  for (uint16_t i=0;i<samples;i++){
    mpu.getEvent(&a,&g,&t);
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

  roll_deg = 0;
  pitch_deg = 0;
  thr_duty_smooth = 0;
}

// =================== WEB UI ===================
static const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>C3 DRONE</title>
<style>
:root{--bg:#0b0f14;--panel:#121a23;--line:#1f2a36;--acc:#34d399;--warn:#fb7185;--txt:#e5e7eb;--mut:#94a3b8;}
body{margin:0;background:var(--bg);color:var(--txt);font-family:system-ui;overflow:hidden;touch-action:none}
.top{display:flex;align-items:center;justify-content:space-between;padding:10px 12px;background:linear-gradient(180deg,#0f172a,#0b0f14);border-bottom:1px solid var(--line)}
.title{font-weight:900;letter-spacing:1px}
.pill{padding:6px 10px;border-radius:999px;background:var(--panel);border:1px solid var(--line);font-size:12px;color:var(--mut)}
.wrap{display:flex;gap:10px;padding:10px;height:calc(100vh - 58px);box-sizing:border-box}
.col{flex:1;display:flex;flex-direction:column;gap:10px}
.card{background:var(--panel);border:1px solid var(--line);border-radius:16px;padding:10px}
.row{display:flex;gap:10px;align-items:center;justify-content:space-between}
button{border:none;border-radius:14px;padding:12px;font-weight:900;color:#0b0f14;background:var(--acc);width:100%}
button.secondary{background:#60a5fa}
button.warn{background:var(--warn);color:white}
button.off{background:#334155;color:#e2e8f0}
.small{font-size:12px;color:var(--mut)}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:10px}
input[type="range"]{width:100%}
.joyArea{display:flex;justify-content:space-between;gap:10px}
.joyBox{flex:1;display:flex;flex-direction:column;gap:10px;align-items:center;justify-content:center}
.joyBase{width:170px;height:170px;border-radius:50%;background:radial-gradient(circle at 30% 30%,rgba(255,255,255,.06),rgba(255,255,255,.02));border:2px solid rgba(52,211,153,.35);position:relative}
.stick{width:60px;height:60px;border-radius:50%;background:radial-gradient(circle at 30% 30%,rgba(52,211,153,.95),rgba(52,211,153,.55));position:absolute;top:55px;left:55px;box-shadow:0 0 18px rgba(52,211,153,.6);touch-action:none}
.thrWrap{width:54px;height:220px;border-radius:18px;background:#0b1220;border:1px solid var(--line);position:relative;overflow:hidden}
.thrFill{position:absolute;bottom:0;left:0;right:0;height:0;background:linear-gradient(180deg,rgba(52,211,153,.1),rgba(52,211,153,.6))}
.thrKnob{width:54px;height:44px;border-radius:18px;background:rgba(255,255,255,.10);border:1px solid rgba(255,255,255,.16);position:absolute;left:0;bottom:0;display:flex;align-items:center;justify-content:center;color:var(--txt);font-weight:900;touch-action:none}
pre{margin:0;font-size:12px;white-space:pre-wrap;word-break:break-word;color:#cbd5e1}
</style></head>
<body>
<div class="top"><div class="title">C3 DRONE</div><div class="pill" id="stat">WS: --</div></div>
<div class="wrap"><div class="col">
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
        <div class="small">IMU / Status</div>
        <pre id="imu">--</pre>
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
        <div class="joyBase" id="yawBase"><div class="stick" id="yawStick"></div></div>
      </div>

      <div class="joyBox">
        <div class="small">PITCH / ROLL</div>
        <div class="joyBase" id="prBase"><div class="stick" id="prStick"></div></div>
        <div class="small">Right stick auto-centers</div>
      </div>
    </div>
  </div>
</div></div>

<script>
let ws; let armed=false,killed=false,mode=0;
let thr=0, yaw=0, roll=0, pitch=0, thrLimit=1.0;

function connectWS(){
  ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onopen = ()=>{ document.getElementById('stat').innerText="WS: OK"; };
  ws.onclose= ()=>{ document.getElementById('stat').innerText="WS: RECONNECT"; setTimeout(connectWS,700); };
  ws.onmessage = (e)=>{
    if(e.data.startsWith("T:")) document.getElementById('imu').innerText = e.data.substring(2);
  };
}
connectWS();

function sendCmd(s){ if(ws && ws.readyState===1) ws.send(s); }
function sendControl(){
  if(ws && ws.readyState===1){
    ws.send(`C:${Math.round(thr*1000)},${Math.round(roll*1000)},${Math.round(pitch*1000)},${Math.round(yaw*1000)},${Math.round(thrLimit*1000)}`);
  }
}
function toggleArm(){
  armed=!armed; const b=document.getElementById('armBtn');
  if(armed){ b.innerText="ARMED"; b.className=""; sendCmd("ARM"); }
  else{ b.innerText="ARM"; b.className="warn"; sendCmd("DISARM"); }
}
function toggleKill(){
  killed=!killed; const b=document.getElementById('killBtn');
  if(killed){ b.innerText="KILL ON"; b.className="warn"; sendCmd("KILL1"); }
  else{ b.innerText="KILL"; b.className="off"; sendCmd("KILL0"); }
}
function toggleMode(){
  mode = (mode===0)?1:0; const b=document.getElementById('modeBtn');
  if(mode===0){ b.innerText="MODE: ANGLE"; sendCmd("MODE0"); }
  else{ b.innerText="MODE: RATE"; sendCmd("MODE1"); }
}
function thrLimitChanged(v){
  document.getElementById('thrLimVal').innerText=v;
  thrLimit = Math.max(0.2, Math.min(1.0, v/100));
  sendControl();
}

// throttle slider (latching)
(()=>{
  const wrap=document.getElementById('thrWrap');
  const knob=document.getElementById('thrKnob');
  const fill=document.getElementById('thrFill');
  function setThr(clientY){
    const r=wrap.getBoundingClientRect();
    let y=clientY-r.top; y=Math.max(0,Math.min(r.height,y));
    thr = 1.0-(y/r.height); thr=Math.max(0,Math.min(1,thr));
    const h=Math.round(thr*r.height);
    fill.style.height=`${h}px`;
    knob.style.bottom=`${h - knob.offsetHeight/2}px`;
    knob.innerText=`${Math.round(thr*100)}%`;
    sendControl();
  }
  wrap.addEventListener('touchstart',e=>setThr(e.touches[0].clientY),{passive:false});
  wrap.addEventListener('touchmove',e=>{e.preventDefault();setThr(e.touches[0].clientY)}, {passive:false});
})();

// yaw stick auto-center
(()=>{
  const base=document.getElementById('yawBase');
  const stick=document.getElementById('yawStick');
  const R=50;
  function setYaw(t){
    const r=base.getBoundingClientRect();
    let x=t.clientX-r.left-r.width/2; x=Math.max(-R,Math.min(R,x));
    stick.style.transform=`translate(${x}px,0px)`;
    yaw=x/R; sendControl();
  }
  base.ontouchstart=e=>setYaw(e.touches[0]);
  base.ontouchmove=e=>{e.preventDefault();setYaw(e.touches[0]);};
  base.ontouchend=()=>{stick.style.transform=`translate(0px,0px)`; yaw=0; sendControl();};
})();

// pitch/roll stick auto-center
(()=>{
  const base=document.getElementById('prBase');
  const stick=document.getElementById('prStick');
  const R=55;
  function setPR(t){
    const r=base.getBoundingClientRect();
    let x=t.clientX-r.left-r.width/2;
    let y=t.clientY-r.top -r.height/2;
    const mag=Math.sqrt(x*x+y*y);
    if(mag>R){ x=x*(R/mag); y=y*(R/mag); }
    stick.style.transform=`translate(${x}px,${y}px)`;
    roll=x/R; pitch=-y/R; sendControl();
  }
  base.ontouchstart=e=>setPR(e.touches[0]);
  base.ontouchmove=e=>{e.preventDefault();setPR(e.touches[0]);};
  base.ontouchend=()=>{stick.style.transform=`translate(0px,0px)`; roll=0; pitch=0; sendControl();};
})();
</script></body></html>
)rawliteral";

// =================== WS HANDLER ===================
static void wsSendOne(AsyncWebSocketClient* c, const char* s){ if(c && c->canSend()) c->text(s); }

static void onWsEvent(AsyncWebSocket *server_, AsyncWebSocketClient *client,
                      AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type != WS_EVT_DATA || len == 0) return;

  // safe copy
  char buf[96];
  size_t n = (len < sizeof(buf)-1) ? len : (sizeof(buf)-1);
  memcpy(buf, data, n);
  buf[n] = 0;
  String msg(buf);

  lastCmdMs = millis();

  if (msg == "ARM") {
    killSwitch = false;
    armed = true;
    i_roll=i_pitch=i_yaw=0;
    last_err_r=last_err_p=last_err_y=0;
    thr_duty_smooth=0;
    wsSendOne(client,"T:ARMED");
    return;
  }
  if (msg == "DISARM") { armed=false; wsSendOne(client,"T:DISARMED"); return; }
  if (msg == "KILL1")   { killSwitch=true; armed=false; wsSendOne(client,"T:KILL ON"); return; }
  if (msg == "KILL0")   { killSwitch=false; wsSendOne(client,"T:KILL OFF"); return; }
  if (msg == "MODE0")   { mode=MODE_ANGLE; wsSendOne(client,"T:MODE ANGLE"); return; }
  if (msg == "MODE1")   { mode=MODE_RATE;  wsSendOne(client,"T:MODE RATE");  return; }

  if (msg == "CAL") {
    armed = false;
    wsSendOne(client,"T:CAL...");
    calibrateGyro();
    wsSendOne(client,"T:CAL OK");
    return;
  }

  if (msg.startsWith("C:")) {
    int t,r,p,y,lim;
    if (sscanf(msg.c_str(), "C:%d,%d,%d,%d,%d", &t,&r,&p,&y,&lim) == 5) {
      float thr = clampf(t/1000.0f,  0.0f, 1.0f);
      float rr  = clampf(r/1000.0f, -1.0f, 1.0f);
      float pp  = clampf(p/1000.0f, -1.0f, 1.0f);
      float yy  = clampf(y/1000.0f, -1.0f, 1.0f);
      float ll  = clampf(lim/1000.0f, 0.2f, 1.0f);

      in_throttle = thr;
      in_roll  = expoCurve(rr, EXPO);
      in_pitch = expoCurve(pp, EXPO);
      in_yaw   = expoCurve(yy, EXPO);
      in_thrLimit = ll;
    }
  }
}

// =================== SETUP ===================
void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  Serial.begin(115200);
  delay(80);

  // IMU
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  if (!mpu.begin()) {
    Serial.println("MPU6050 init failed!");
    while(1){ ledPowerBlinkTask(); delay(5); }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // PWM
  ledcSetup(CH_M1, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(CH_M2, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(CH_M3, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(CH_M4, PWM_FREQ, PWM_RES_BITS);

  ledcAttachPin(PIN_M1, CH_M1);
  ledcAttachPin(PIN_M2, CH_M2);
  ledcAttachPin(PIN_M3, CH_M3);
  ledcAttachPin(PIN_M4, CH_M4);

  allMotorsOff();

  // WiFi AP
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  // Web
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req){
    req->send_P(200, "text/html", index_html);
  });
  server.begin();

  // Boot gyro calibration (keep still/flat)
  for(int i=0;i<8;i++){ ledPowerBlinkTask(); delay(60); }
  calibrateGyro();

  lastCmdMs = millis();

  // WiFi connected (AP ready) => solid LED
  ledWifiConnectedOn();

  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());
}

// =================== MAIN LOOP ===================
void loop() {
  ws.cleanupClients();

  // failsafe: if no cmd packets -> neutral; after longer -> disarm
  uint32_t nowMs = millis();
  if (armed && (nowMs - lastCmdMs > CMD_TIMEOUT_MS)) {
    in_throttle = 0.0f;
    in_roll = in_pitch = in_yaw = 0.0f;
  }
  if (armed && (nowMs - lastCmdMs > DISARM_TIMEOUT_MS)) {
    armed = false;
  }
  if (killSwitch) armed = false;

  // fixed-rate control loop
  static uint32_t lastUs = micros();
  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastUs) < LOOP_US) return;
  float dt = (nowUs - lastUs) / 1000000.0f;
  lastUs = nowUs;
  if (dt <= 0) dt = 0.004f;

  sensors_event_t a,g,t;
  mpu.getEvent(&a,&g,&t);

  // Save raw telemetry
  t_ax = a.acceleration.x; t_ay = a.acceleration.y; t_az = a.acceleration.z;

  // gyro corrected (deg/s)
  float gx = (g.gyro.x - gyro_bias_x) * 180.0f / PI;
  float gy = (g.gyro.y - gyro_bias_y) * 180.0f / PI;
  float gz = (g.gyro.z - gyro_bias_z) * 180.0f / PI;
  t_gx=gx; t_gy=gy; t_gz=gz;

  // accel angles (deg)
  float accRoll  = atan2f(a.acceleration.y, a.acceleration.z) * 180.0f / PI;
  float accPitch = atan2f(-a.acceleration.x,
                          sqrtf(a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z)) * 180.0f / PI;

  // complementary filter
  roll_deg  = CF_ALPHA * (roll_deg  + gx * dt) + (1.0f - CF_ALPHA) * accRoll;
  pitch_deg = CF_ALPHA * (pitch_deg + gy * dt) + (1.0f - CF_ALPHA) * accPitch;

  // Desired rates
  float des_rate_roll=0, des_rate_pitch=0, des_rate_yaw=0;

  if (mode == MODE_ANGLE) {
    float des_ang_roll  = in_roll  * MAX_ANGLE_DEG;
    float des_ang_pitch = in_pitch * MAX_ANGLE_DEG;

    des_rate_roll  = clampf((des_ang_roll  - roll_deg ) * Kp_angle, -MAX_RATE_RP, MAX_RATE_RP);
    des_rate_pitch = clampf((des_ang_pitch - pitch_deg) * Kp_angle, -MAX_RATE_RP, MAX_RATE_RP);
  } else {
    des_rate_roll  = in_roll  * MAX_RATE_RP;
    des_rate_pitch = in_pitch * MAX_RATE_RP;
  }
  des_rate_yaw = in_yaw * MAX_RATE_Y;

  // RATE PID Roll
  float err_r = des_rate_roll - gx;
  i_roll = clampf(i_roll + err_r * dt, -200.0f, 200.0f);
  float d_r = (err_r - last_err_r) / dt; last_err_r = err_r;
  float out_r = Kp_rate_rp*err_r + Ki_rate_rp*i_roll + Kd_rate_rp*d_r;

  // RATE PID Pitch
  float err_p = des_rate_pitch - gy;
  i_pitch = clampf(i_pitch + err_p * dt, -200.0f, 200.0f);
  float d_p = (err_p - last_err_p) / dt; last_err_p = err_p;
  float out_p = Kp_rate_rp*err_p + Ki_rate_rp*i_pitch + Kd_rate_rp*d_p;

  // RATE PID Yaw
  float err_y = des_rate_yaw - gz;
  i_yaw = clampf(i_yaw + err_y * dt, -200.0f, 200.0f);
  float d_y = (err_y - last_err_y) / dt; last_err_y = err_y;
  float out_y = Kp_rate_y*err_y + Ki_rate_y*i_yaw + Kd_rate_y*d_y;

  // Throttle -> base duty
  float thr = clampf(in_throttle, 0.0f, 1.0f) * clampf(in_thrLimit, 0.2f, 1.0f);

  int targetDuty = 0;
  if (armed && thr > 0.001f) {
    int usable = MOTOR_MAX_LIMIT - MOTOR_MIN_START;
    targetDuty = MOTOR_MIN_START + (int)(thr * usable);
  } else {
    targetDuty = 0;
  }

  // Throttle ramp smoothing
  if (targetDuty > thr_duty_smooth) thr_duty_smooth = min(targetDuty, thr_duty_smooth + THR_RAMP_PER_LOOP);
  else                             thr_duty_smooth = max(targetDuty, thr_duty_smooth - THR_RAMP_PER_LOOP);

  float base = (float)thr_duty_smooth;

  // If yaw is reversed, set yawSign=-1
  const float yawSign = 1.0f;
  float yawTerm = yawSign * out_y;

  // X mix: M1 FL, M2 FR, M3 BR, M4 BL
  float m1 = base - out_p + out_r - yawTerm;
  float m2 = base - out_p - out_r + yawTerm;
  float m3 = base + out_p - out_r - yawTerm;
  float m4 = base + out_p + out_r + yawTerm;

  // Saturation protection: shift down if any exceeds MOTOR_MAX_LIMIT
  float maxOut = fmaxf(fmaxf(m1,m2), fmaxf(m3,m4));
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

  // Telemetry to UI ~10Hz
  static uint32_t tms=0;
  if (nowMs - tms > 100) {
    tms = nowMs;
    char line[180];
    snprintf(line, sizeof(line),
      "T:ARM:%d KILL:%d MODE:%c\nROLL:%.1f  PITCH:%.1f\nGX:%.0f GY:%.0f GZ:%.0f\nAX:%.2f AY:%.2f AZ:%.2f",
      armed?1:0, killSwitch?1:0, (mode==MODE_ANGLE)?'A':'R',
      roll_deg, pitch_deg, t_gx, t_gy, t_gz, t_ax, t_ay, t_az);
    ws.textAll(line);
  }
}
