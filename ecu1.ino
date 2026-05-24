/*
 * ================================================================
 *  ECU #1 — CONTROL SYSTEM  (UART SENDER)
 *  Board   : STM32F401RE  Nucleo-64
 *  IDE     : Arduino IDE + STM32duino core
 *  Serial Monitor baud : 115200
 *
 *  UART DATA LINK (proven D8/D2):
 *    D8 = PA9  = UART1 TX  ──→  ECU2 D2 (PA10)
 *    D2 = PA10 = UART1 RX  ←──  ECU2 D8 (PA9)
 *    GND ────────────────────── GND  (MUST connect!)
 *
 *  IR REMOTE: Uses IRremote library (NEC protocol)
 *    Install: Arduino Library Manager → "IRremote" by shirriff/z3t0
 *    RED   button sends NEC code → LOCK
 *    GREEN button sends NEC code → UNLOCK
 *    (Press each button once, note the code printed in Serial Monitor,
 *     then set IR_CODE_LOCK and IR_CODE_UNLOCK below)
 *
 *  ECU1 COMPLETE PIN MAP:
 *  ┌─────────────────────┬────────┬──────────────────────────┐
 *  │ Component           │ Pin    │ Interface                │
 *  ├─────────────────────┼────────┼──────────────────────────┤
 *  │ Joystick1 VRx       │ PA0    │ ADC – Steering           │
 *  │ Joystick2 VRx       │ PA1    │ ADC – Accel/Brake        │
 *  │ Joystick1 SW (Horn) │ PA4    │ GPIO INPUT_PULLUP        │
 *  │ Potentiometer       │ PB0    │ ADC – Wiper speed        │
 *  │ IR Receiver TSOP    │ PA8    │ GPIO INPUT (IRremote)    │
 *  │ Ultrasonic TRIG     │ PB10   │ GPIO OUTPUT              │
 *  │ Ultrasonic ECHO     │ PB8    │ GPIO INPUT (5V divider!) │
 *  │ Push Button 1       │ PB3    │ Fwd/Rev toggle           │
 *  │ Push Button 2       │ PB4    │ Parking toggle           │
 *  │ Push Button 3       │ PB5    │ Headlight toggle         │
 *  │ UART1 TX  [D8]      │ PA9    │ → ECU2 D2(PA10)          │
 *  │ UART1 RX  [D2]      │ PA10   │ ← ECU2 D8(PA9)           │
 *  └─────────────────────┴────────┴──────────────────────────┘
 *
 *  WIRING NOTES:
 *  • Joysticks/Pot: VCC→3.3V, GND→GND, signal→ADC pin
 *  • Push buttons: one pin→GPIO, other pin→GND (PULLUP used)
 *  • Ultrasonic ECHO: HC-SR04 outputs 5V → use voltage divider:
 *    ECHO → 1kΩ → PB8 → 2kΩ → GND
 *  • IR receiver TSOP1738: VCC→3.3V, GND→GND, OUT→PA8
 *  • Add 100µF capacitor across TSOP VCC–GND
 *
 *  PACKET FORMAT (14 bytes @ 57600 baud):
 *  [0xAA][0x55][B0..B10][CHK][0xFF]
 *  B0  Steering angle  0–180
 *  B1  Motor speed     0–100 %
 *  B2  Direction       0=stop 1=fwd 2=rev
 *  B3  Brake PWM       0–255
 *  B4  Horn            0/1
 *  B5  Indicator       0=off 1=left 2=right
 *  B6  Wiper speed     0–255
 *  B7  Parking lights  0/1
 *  B8  Headlight       0/1
 *  B9  Lock/Unlock     0=none 1=lock 2=unlock
 *  B10 ABS auto-brake  0–255
 * ================================================================
 */

#include <IRremote.hpp>   // IRremote v3.x  (IRremote by shirriff)

HardwareSerial DataSerial(PA10, PA9);   // RX=PA10(D2), TX=PA9(D8)

// ── PINS ─────────────────────────────────────────────────────
#define PIN_JOY1_VRX    PA0
#define PIN_JOY2_VRX    PA1
#define PIN_JOY1_SW     PA4
#define PIN_WIPER_POT   PB0
#define PIN_IR_RECV     PA8
#define PIN_ULTRA_TRIG  PB10
#define PIN_ULTRA_ECHO  PB8
#define PIN_BTN1        PB3
#define PIN_BTN2        PB4
#define PIN_BTN3        PB5

// ── IR REMOTE CODES ──────────────────────────────────────────
// After first upload, press each button and read the code from
// Serial Monitor (it prints "IR RAW: 0xXXXXXXXX"). Set them here.
#define IR_CODE_LOCK    0xBC43FF00   // change to your RED   button code
#define IR_CODE_UNLOCK  0xF609FF00   // change to your GREEN button code

// ── PROTOCOL ─────────────────────────────────────────────────
#define UART_BAUD    57600
#define START1       0xAA
#define START2       0x55
#define END_BYTE     0xFF
#define PAYLOAD_LEN  11
#define PACKET_LEN   14

// ── STATE ────────────────────────────────────────────────────
bool    fwdState    = true;
bool    parkingOn   = false;
bool    headlightOn = false;
uint8_t indicState  = 0;
uint8_t lockCmd     = 0;
uint32_t txCount    = 0;
uint8_t  absVal     = 0;
long     lastDist   = 999;

// ── DEBOUNCE ─────────────────────────────────────────────────
struct Btn { uint8_t pin; bool last; uint32_t t; };
Btn btn1    = { PIN_BTN1,    HIGH, 0 };
Btn btn2    = { PIN_BTN2,    HIGH, 0 };
Btn btn3    = { PIN_BTN3,    HIGH, 0 };
Btn hornBtn = { PIN_JOY1_SW, HIGH, 0 };
#define DB_MS 50

bool pressed(Btn &b) {
  bool cur = digitalRead(b.pin);
  uint32_t now = millis();
  if (cur == LOW && b.last == HIGH && (now - b.t) > DB_MS) {
    b.t = now; b.last = cur; return true;
  }
  if (cur != b.last) b.last = cur;
  return false;
}
bool held(Btn &b) { return digitalRead(b.pin) == LOW; }

// ── ULTRASONIC ───────────────────────────────────────────────
void ultrasonicPoll() {
  digitalWrite(PIN_ULTRA_TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(PIN_ULTRA_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_ULTRA_TRIG, LOW);
  long dur = pulseIn(PIN_ULTRA_ECHO, HIGH, 6000); // 6ms = ~100cm max
  lastDist  = dur ? dur / 58 : 999;
  if      (lastDist <= 10) absVal = 255;
  else if (lastDist <= 80) absVal = (uint8_t)map(lastDist, 80, 10, 0, 255);
  else                     absVal = 0;
}

// ── SEND PACKET ──────────────────────────────────────────────
void sendPacket(uint8_t p[]) {
  uint8_t chk = 0;
  for (int i = 0; i < PAYLOAD_LEN; i++) chk ^= p[i];
  DataSerial.write(START1);
  DataSerial.write(START2);
  DataSerial.write(p, PAYLOAD_LEN);
  DataSerial.write(chk);
  DataSerial.write(END_BYTE);
  txCount++;
}

// ── PRINT HELPERS ────────────────────────────────────────────
void pp(int v, int w) {
  char buf[12]; itoa(v,buf,10);
  int l=strlen(buf); for(int i=l;i<w;i++) Serial.print(' ');
  Serial.print(buf);
}
void bar(int v, int mx, int w) {
  int f=map(constrain(v,0,mx),0,mx,0,w);
  Serial.print('['); for(int i=0;i<w;i++) Serial.print(i<f?'#':'.'); Serial.print(']');
}

// ── SETUP ────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(400);
  analogReadResolution(12);

  pinMode(PIN_JOY1_SW,    INPUT_PULLUP);
  pinMode(PIN_BTN1,       INPUT_PULLUP);
  pinMode(PIN_BTN2,       INPUT_PULLUP);
  pinMode(PIN_BTN3,       INPUT_PULLUP);
  pinMode(PIN_ULTRA_TRIG, OUTPUT);
  pinMode(PIN_ULTRA_ECHO, INPUT);
  digitalWrite(PIN_ULTRA_TRIG, LOW);

  // IRremote v3 init
  IrReceiver.begin(PIN_IR_RECV, DISABLE_LED_FEEDBACK);

  DataSerial.begin(UART_BAUD);

  Serial.println("[ECU1] Booted | TX=PA9(D8) RX=PA10(D2) @ 57600");
  Serial.println("[ECU1] Press IR buttons to see their codes");
}

// ── LOOP ─────────────────────────────────────────────────────
void loop() {
  static uint32_t txTimer    = 0;
  static uint32_t ultraTimer = 0;
  static uint32_t dashTimer  = 0;

  static uint8_t s_angle=90,s_speed=0,s_dir=0,s_brake=0;
  static uint8_t s_wiper=0, s_indic=0,s_lock=0, s_abs=0;
  static bool    s_horn=false,s_park=false,s_hdlt=false;

  uint32_t now = millis();

  // ─ Ultrasonic every 100 ms ────────────────────────────────
  if (now - ultraTimer >= 100) { ultraTimer = now; ultrasonicPoll(); }

  // ─ Toggle buttons ─────────────────────────────────────────
  if (pressed(btn1)) fwdState    = !fwdState;
  if (pressed(btn2)) parkingOn   = !parkingOn;
  if (pressed(btn3)) headlightOn = !headlightOn;

  // ─ IR Remote (IRremote library) ───────────────────────────
  if (IrReceiver.decode()) {
    uint32_t code = IrReceiver.decodedIRData.decodedRawData;
    Serial.print("[ECU1] IR RAW: 0x"); Serial.println(code, HEX);
    if (code == IR_CODE_LOCK)   lockCmd = 1;
    if (code == IR_CODE_UNLOCK) lockCmd = 2;
    IrReceiver.resume();  // ready for next code
  }

  // ─ Build & send packet every 20 ms ────────────────────────
  if (now - txTimer >= 20) {
    txTimer = now;
    uint8_t p[PAYLOAD_LEN] = {0};

    // B0 Steering
    int rawLR = analogRead(PIN_JOY1_VRX);
    uint8_t angle = (uint8_t)map(rawLR, 0, 4095, 0, 180);
    if (angle > 82 && angle < 98) angle = 90;
    p[0] = angle;

    // Auto-indicator from steering
    if      (angle <= 75)  indicState = 1;
    else if (angle >= 105) indicState = 2;
    else                   indicState = 0;

    // B1 B2 Speed + Direction
    int rawFB = analogRead(PIN_JOY2_VRX);
    // Joystick centre ≈ 2048 with ±150 dead-band
    const int CTR=2048, DEAD=150;
    uint8_t speed=0, dir=0, brakeVal=0;
    if (rawFB > CTR + DEAD) {
      speed = (uint8_t)map(rawFB, CTR+DEAD, 4095, 10, 100); // min 10% to overcome stall
      dir   = fwdState ? 1 : 2;
    } else if (rawFB < CTR - DEAD) {
      brakeVal = (uint8_t)map(rawFB, CTR-DEAD, 0, 0, 255);
      dir = 0; speed = 0;
    }

    p[1]  = speed;
    p[2]  = dir;
    p[3]  = max(brakeVal, absVal);
    p[4]  = held(hornBtn) ? 1 : 0;
    p[5]  = indicState;
    p[6]  = (uint8_t)map(analogRead(PIN_WIPER_POT), 0, 4095, 0, 255);
    p[7]  = parkingOn   ? 1 : 0;
    p[8]  = headlightOn ? 1 : 0;
    p[9]  = lockCmd;
    p[10] = absVal;

    s_angle=p[0]; s_speed=p[1]; s_dir=p[2]; s_brake=p[3];
    s_horn=p[4];  s_indic=p[5]; s_wiper=p[6]; s_park=p[7];
    s_hdlt=p[8];  s_lock=p[9];  s_abs=p[10];

    lockCmd = 0;
    sendPacket(p);
  }

  // ─ Dashboard every 500 ms ─────────────────────────────────
  if (now - dashTimer >= 500) {
    dashTimer = now;
    Serial.print("\033[2J\033[H");
    Serial.println("+==================================================+");
    Serial.println("|    ECU #1  --  CONTROL SYSTEM  (UART SENDER)    |");
    Serial.println("|  TX=PA9(D8)--ECU2   RX=PA10(D2)--ECU2  @57600  |");
    Serial.println("+==================================================+");
    Serial.print("|  Packets sent: "); pp(txCount,8); Serial.println("                    |");
    Serial.println("+--------------------------------------------------+");
    Serial.println("|  INPUT              Value   Bar                  |");
    Serial.println("+--------------------------------------------------+");
    Serial.print("|  Steer angle(deg)   "); pp(s_angle,5); Serial.print("   "); bar(s_angle,180,18); Serial.println("  |");
    Serial.print("|  Motor speed(%)     "); pp(s_speed,5); Serial.print("   "); bar(s_speed,100,18); Serial.println("  |");
    Serial.print("|  Brake PWM          "); pp(s_brake,5); Serial.print("   "); bar(s_brake,255,18); Serial.println("  |");
    Serial.print("|  ABS auto-brake     "); pp(s_abs,5);   Serial.print("   "); bar(s_abs,255,18);   Serial.println("  |");
    Serial.print("|  Wiper speed        "); pp(s_wiper,5); Serial.print("   "); bar(s_wiper,255,18); Serial.println("  |");
    Serial.print("|  Ultrasonic(cm)     ");
    if (lastDist>=999) { Serial.print(" ----   [  out of range      ]"); }
    else { pp((int)lastDist,5); Serial.print("   "); bar((int)constrain(lastDist,0,80),80,18); }
    Serial.println("  |");
    Serial.println("+--------------------------------------------------+");
    Serial.print("|  Direction : ");
    if(s_dir==1) Serial.print("FORWARD  "); else if(s_dir==2) Serial.print("REVERSE  "); else Serial.print("STOP     ");
    Serial.print("   Indicator: ");
    if(s_indic==1) Serial.print("LEFT <<  |"); else if(s_indic==2) Serial.print("RIGHT >> |"); else Serial.print("OFF      |");
    Serial.println();
    Serial.print("|  Horn      : "); Serial.print(s_horn ?"BEEPING  ":"silent   ");
    Serial.print("   Parking  : "); Serial.print(s_park ?"ON       |":"OFF      |"); Serial.println();
    Serial.print("|  Headlight : "); Serial.print(s_hdlt ?"ON       ":"OFF      ");
    Serial.print("   IR Lock  : ");
    if(s_lock==1) Serial.print("LOCK     |"); else if(s_lock==2) Serial.print("UNLOCK   |"); else Serial.print("idle     |");
    Serial.println();
    Serial.println("+==================================================+");
    Serial.print("  Uptime: "); Serial.print(now/1000);
    Serial.print("s   TX pkts: "); Serial.println(txCount);
  }
}
