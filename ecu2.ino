/*
 * ================================================================
 *  ECU #2 — ACTUATOR SYSTEM  (UART RECEIVER)
 *  Board   : STM32F401RE  Nucleo-64
 *  IDE     : Arduino IDE + STM32duino core
 *  Serial Monitor baud : 115200
 *
 *  UART DATA LINK:
 *    D2 = PA10 = UART1 RX  ←──  ECU1 D8 (PA9  TX)
 *    D8 = PA9  = UART1 TX  ──→  ECU1 D2 (PA10 RX)
 *    GND ─────────────────────── GND  (MUST connect!)
 *
 *  MOTOR FIX: ENA/ENB moved to PA15/PB3 (confirmed PWM on F401)
 *  PARKING FIX: LED bar moved to PC4–PC9 (safe pins, no conflicts)
 *
 *  ECU2 COMPLETE PIN MAP:
 *  ┌──────────────────────┬─────────┬───────────────────────────┐
 *  │ Component            │ Pin     │ Interface                 │
 *  ├──────────────────────┼─────────┼───────────────────────────┤
 *  │ Servo1 Steering      │ PA0     │ PWM (TIM2_CH1)            │
 *  │ Servo2 Wiper         │ PA1     │ PWM (TIM2_CH2)            │
 *  │ L298N IN1 (Mot-A)    │ PA5     │ GPIO OUTPUT               │
 *  │ L298N IN2 (Mot-A)    │ PA6     │ GPIO OUTPUT               │
 *  │ L298N IN3 (Mot-B)    │ PA7     │ GPIO OUTPUT               │
 *  │ L298N IN4 (Mot-B)    │ PB0     │ GPIO OUTPUT               │
 *  │ L298N ENA (speed)    │ PA15    │ PWM (TIM2_CH1) ← FIXED    │
 *  │ L298N ENB (speed)    │ PB3     │ PWM (TIM2_CH2) ← FIXED    │
 *  │ Buzzer Horn          │ PA8     │ tone()                    │
 *  │ Buzzer Indicator     │ PC6     │ tone()                    │
 *  │ Left Indicator LED   │ PC0     │ GPIO OUTPUT               │
 *  │ Right Indicator LED  │ PC1     │ GPIO OUTPUT               │
 *  │ Brake LED (PWM)      │ PC2     │ analogWrite               │
 *  │ Headlight Relay      │ PC3     │ GPIO OUTPUT               │
 *  │ TM1637 Speed CLK     │ PC7     │ bit-bang                  │
 *  │ TM1637 Speed DIO     │ PA11    │ bit-bang (shared)         │
 *  │ TM1637 RPM   CLK     │ PA12    │ bit-bang                  │
 *  │ TM1637 RPM   DIO     │ PA11    │ bit-bang (shared)         │
 *  │ LED Bar #1 Parking   │ PC4     │ GPIO OUTPUT ← FIXED       │
 *  │ LED Bar #2           │ PC5     │ GPIO OUTPUT ← FIXED       │
 *  │ LED Bar #3           │ PC8     │ GPIO OUTPUT ← FIXED       │
 *  │ LED Bar #4           │ PC9     │ GPIO OUTPUT ← FIXED       │
 *  │ LED Bar #5           │ PC10    │ GPIO OUTPUT ← FIXED       │
 *  │ LED Bar #6           │ PC11    │ GPIO OUTPUT ← FIXED       │
 *  │ UART1 RX  [D2]       │ PA10    │ ← ECU1 D8 (PA9)           │
 *  │ UART1 TX  [D8]       │ PA9     │ → ECU1 D2 (PA10)          │
 *  └──────────────────────┴─────────┴───────────────────────────┘
 *
 *  L298N WIRING:
 *  • 12V  → L298N 12V pin
 *  • GND  → L298N GND (shared with STM32 GND)
 *  • ENA  → PA15   (PWM speed control for Motor A)
 *  • IN1  → PA5    (Motor A direction)
 *  • IN2  → PA6    (Motor A direction)
 *  • ENB  → PB3    (PWM speed control for Motor B)
 *  • IN3  → PA7    (Motor B direction)
 *  • IN4  → PB0    (Motor B direction)
 *  • OUT1/OUT2 → Motor 1
 *  • OUT3/OUT4 → Motor 2
 *
 *  LED BAR GRAPH WIRING (each LED):
 *  • Pin → 330Ω resistor → LED anode → LED cathode → GND
 * ================================================================
 */

#include <Servo.h>
#include <TM1637Display.h>

HardwareSerial DataSerial(PA10, PA9);   // RX=PA10(D2), TX=PA9(D8)

// ── PINS ─────────────────────────────────────────────────────
#define PIN_SERVO_STEER   PA0
#define PIN_SERVO_WIPER   PA1
bool isImmobilized = false; // New variable to track if car is disabled

// L298N direction pins
#define PIN_IN1           PA5
#define PIN_IN2           PA6
#define PIN_IN3           PA7
#define PIN_IN4           PB0

// L298N enable pins — FIXED to actual PWM-capable pins on F401
#define PIN_ENA           PA15   // TIM2_CH1 — confirmed PWM on F401
#define PIN_ENB           PB3    // TIM2_CH2 — confirmed PWM on F401

// Change this line in the PINS section
#define PIN_BUZZ_HORN     PB4
#define PIN_BUZZ_INDIC    PC6

#define PIN_LED_LEFT      PC0
#define PIN_LED_RIGHT     PC1
#define PIN_BRAKE_LED     PB10
#define PIN_RELAY         PC3

// TM1637 — shared DIO on PA11
#define PIN_SPD_CLK       PC7
#define PIN_SPD_DIO       PA11
#define PIN_RPM_CLK       PA12
#define PIN_RPM_DIO       PA11

// Parking LED bar — FIXED to safe pins (no SWD/USB conflict)
const uint8_t PARK_PINS[] = { PC4, PC5, PC8, PC9, PC10, PC11 };
#define PARK_COUNT 6

// ── PROTOCOL ─────────────────────────────────────────────────
#define UART_BAUD    57600
#define START1       0xAA
#define START2       0x55
#define END_BYTE     0xFF
#define PAYLOAD_LEN  11
#define PACKET_LEN   14

// ── PACKET PARSER STATE MACHINE ──────────────────────────────
enum ParseState { WAIT_S1, WAIT_S2, READ_PAYLOAD, READ_CHK, WAIT_END };
ParseState pState = WAIT_S1;
uint8_t pBuf[PAYLOAD_LEN];
uint8_t pIdx = 0, pChk = 0;
uint32_t rxOK = 0, rxBad = 0;
bool newFrame = false;

// ── DECODED COMMANDS ─────────────────────────────────────────
uint8_t cmd_angle     = 90;
uint8_t cmd_speed     = 0;
uint8_t cmd_dir       = 0;
uint8_t cmd_brake     = 0;
bool    cmd_horn      = false;
uint8_t cmd_indic     = 0;
uint8_t cmd_wiper     = 0;
bool    cmd_parking   = false;
bool    cmd_headlight = false;
uint8_t cmd_lock      = 0;
uint8_t cmd_abs       = 0;

// ── ACTUATOR STATE ───────────────────────────────────────────
int  act_motorPWM  = 0;
int  act_brakePWM  = 0;
int  act_dispSpd   = 0;
int  act_dispRPM   = 0;
int  wiperPos      = 0, wiperDir = 1;
uint8_t lastServoAngle = 255;

// ── BLINK TIMERS ─────────────────────────────────────────────
uint32_t indicTimer=0; bool indicLed=false;
uint32_t parkTimer=0;  bool parkLed=false;
uint32_t wiperTimer=0, dispTimer=0, lockTimer=0;
uint8_t  lockCount=0;  bool lockSeq=false;

// ── OBJECTS ──────────────────────────────────────────────────
Servo         servoSteer, servoWiper;
TM1637Display dispSpd(PIN_SPD_CLK, PIN_SPD_DIO);
TM1637Display dispRPM(PIN_RPM_CLK, PIN_RPM_DIO);

// ── PACKET PARSER ────────────────────────────────────────────
void parseByte(uint8_t b) {
  switch (pState) {
    case WAIT_S1:  if (b==START1) pState=WAIT_S2; break;
    case WAIT_S2:
      if (b==START2) { pIdx=0; pChk=0; pState=READ_PAYLOAD; }
      else pState=WAIT_S1;
      break;
    case READ_PAYLOAD:
      pBuf[pIdx]=b; pChk^=b; pIdx++;
      if (pIdx>=PAYLOAD_LEN) pState=READ_CHK;
      break;
    case READ_CHK:
      if (b==pChk) pState=WAIT_END;
      else { rxBad++; pState=WAIT_S1; }
      break;
    case WAIT_END:
      if (b==END_BYTE) { rxOK++; newFrame=true; }
      else rxBad++;
      pState=WAIT_S1;
      break;
  }
}

// ── APPLY PAYLOAD ────────────────────────────────────────────
void applyPayload() {
  cmd_angle     = pBuf[0];
  cmd_speed     = pBuf[1];
  cmd_dir       = pBuf[2];
  cmd_brake     = pBuf[3];
  cmd_horn      = (pBuf[4]!=0);
  cmd_indic     = pBuf[5];
  cmd_wiper     = pBuf[6];
  cmd_parking   = (pBuf[7]!=0);
  cmd_headlight = (pBuf[8]!=0);
  if (pBuf[9]!=0) { cmd_lock=pBuf[9]; lockSeq=true; lockCount=0; lockTimer=millis(); }
  cmd_abs       = pBuf[10];
  if (pBuf[9] == 1) { 
    isImmobilized = true;  // LOCK received
    lockSeq = true; 
    lockCount = 0; 
    lockTimer = millis(); 
  }
  else if (pBuf[9] == 2) { 
    isImmobilized = false; // UNLOCK received
    lockSeq = true; 
    lockCount = 0; 
    lockTimer = millis(); 
  }
}

// ── MOTOR DRIVE ──────────────────────────────────────────────
// FIX: ENA/ENB are now on real PWM pins (PA15, PB3)
// FIX: Motor runs when speed>0 regardless of exact dir value
// FIX: Enable pins HIGH first, then set direction
void driveMotors() {
  uint8_t effBrk = max(cmd_brake, cmd_abs);

  // Hard brake or ABS — cut all power immediately
  if (effBrk > 180) {
    analogWrite(PIN_ENA, 0);
    analogWrite(PIN_ENB, 0);
    digitalWrite(PIN_IN1,LOW); digitalWrite(PIN_IN2,LOW);
    digitalWrite(PIN_IN3,LOW); digitalWrite(PIN_IN4,LOW);
    act_motorPWM = 0;
    return;
  }

  uint8_t pwm = (uint8_t)map(constrain(cmd_speed, 0, 100), 0, 100, 0, 255);

  if (cmd_speed > 0 && cmd_dir == 1) {
    // Forward
    digitalWrite(PIN_IN1, HIGH); digitalWrite(PIN_IN2, LOW);
    digitalWrite(PIN_IN3, HIGH); digitalWrite(PIN_IN4, LOW);
    analogWrite(PIN_ENA, pwm);
    analogWrite(PIN_ENB, pwm);
    act_motorPWM = pwm;
  } else if (cmd_speed > 0 && cmd_dir == 2) {
    // Reverse
    digitalWrite(PIN_IN1, LOW);  digitalWrite(PIN_IN2, HIGH);
    digitalWrite(PIN_IN3, LOW);  digitalWrite(PIN_IN4, HIGH);
    analogWrite(PIN_ENA, pwm);
    analogWrite(PIN_ENB, pwm);
    act_motorPWM = pwm;
  } else {
    // Stop — cut PWM, hold brake
    analogWrite(PIN_ENA, 0);
    analogWrite(PIN_ENB, 0);
    // Short brake: both inputs same = motor brake
    digitalWrite(PIN_IN1, HIGH); digitalWrite(PIN_IN2, HIGH);
    digitalWrite(PIN_IN3, HIGH); digitalWrite(PIN_IN4, HIGH);
    act_motorPWM = 0;
  }
}

// ── PRINT HELPERS ────────────────────────────────────────────
void pp(int v, int w) {
  char buf[12]; itoa(v,buf,10);
  int l=strlen(buf); for(int i=l;i<w;i++) Serial.print(' '); Serial.print(buf);
}
void bar(int v, int mx, int w) {
  int f=map(constrain(v,0,mx),0,mx,0,w);
  Serial.print('['); for(int i=0;i<w;i++) Serial.print(i<f?'#':'.'); Serial.print(']');
}

// ── SETUP ────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(200);

  // Servos
  servoSteer.attach(PIN_SERVO_STEER, 544, 2400);
  servoWiper.attach(PIN_SERVO_WIPER, 544, 2400);
  servoSteer.write(90); servoWiper.write(0);
  lastServoAngle = 90;

  // Motor driver — set all LOW first, then enable
  pinMode(PIN_IN1,OUTPUT); pinMode(PIN_IN2,OUTPUT);
  pinMode(PIN_IN3,OUTPUT); pinMode(PIN_IN4,OUTPUT);
  pinMode(PIN_ENA,OUTPUT); pinMode(PIN_ENB,OUTPUT);
  digitalWrite(PIN_IN1,LOW); digitalWrite(PIN_IN2,LOW);
  digitalWrite(PIN_IN3,LOW); digitalWrite(PIN_IN4,LOW);
  analogWrite(PIN_ENA, 0);   analogWrite(PIN_ENB, 0);

  // Buzzers
  pinMode(PIN_BUZZ_HORN, OUTPUT);  digitalWrite(PIN_BUZZ_HORN, LOW);
  pinMode(PIN_BUZZ_INDIC,OUTPUT);  digitalWrite(PIN_BUZZ_INDIC,LOW);

  // LEDs
  pinMode(PIN_LED_LEFT, OUTPUT);  digitalWrite(PIN_LED_LEFT, LOW);
  pinMode(PIN_LED_RIGHT,OUTPUT);  digitalWrite(PIN_LED_RIGHT,LOW);

  // Brake + relay
  pinMode(PIN_BRAKE_LED,OUTPUT); analogWrite(PIN_BRAKE_LED, 0);
  pinMode(PIN_RELAY,    OUTPUT); digitalWrite(PIN_RELAY,    LOW);

  // Parking LED bar
  for (int i=0;i<PARK_COUNT;i++) {
    pinMode(PARK_PINS[i],OUTPUT);
    digitalWrite(PARK_PINS[i],LOW);
  }

  // TM1637 displays
  dispSpd.setBrightness(4); dispSpd.showNumberDec(0,true); delay(5);
  dispRPM.setBrightness(4); dispRPM.showNumberDec(0,true);

  DataSerial.begin(UART_BAUD);

  Serial.println("[ECU2] Booted | RX=PA10(D2) TX=PA9(D8) @ 57600");
  Serial.println("[ECU2] Waiting for ECU1 packets...");

  // Startup LED test — flash all parking LEDs once so you can verify wiring
  Serial.println("[ECU2] LED bar test...");
  for (int i=0;i<PARK_COUNT;i++) digitalWrite(PARK_PINS[i],HIGH);
  delay(300);
  for (int i=0;i<PARK_COUNT;i++) digitalWrite(PARK_PINS[i],LOW);

  // Motor test pulse — brief forward blip so you can verify motors wired correctly
  Serial.println("[ECU2] Motor test pulse (0.4s fwd)...");
  digitalWrite(PIN_IN1,HIGH); digitalWrite(PIN_IN2,LOW);
  digitalWrite(PIN_IN3,HIGH); digitalWrite(PIN_IN4,LOW);
  analogWrite(PIN_ENA, 150); analogWrite(PIN_ENB, 150);
  delay(400);
  analogWrite(PIN_ENA, 0);   analogWrite(PIN_ENB, 0);
  digitalWrite(PIN_IN1,LOW); digitalWrite(PIN_IN2,LOW);
  digitalWrite(PIN_IN3,LOW); digitalWrite(PIN_IN4,LOW);
  Serial.println("[ECU2] Startup tests done. Running...");
}

// ── LOOP ─────────────────────────────────────────────────────
void loop() {
  uint32_t now = millis();

  // ─ 1. Parse incoming UART bytes ───────────────────────────
  while (DataSerial.available()) parseByte((uint8_t)DataSerial.read());

  // ─ 2. Apply frame when complete ───────────────────────────
  if (newFrame) { newFrame=false; applyPayload(); }

  // ─ 3. Steering servo (write only on change) ───────────────
  if (cmd_angle != lastServoAngle) {
    servoSteer.write(cmd_angle);
    lastServoAngle = cmd_angle;
  }
  // ─ 3. Steering servo ───────────────────────────────
  if (isImmobilized) {
    servoSteer.write(90); // Force wheels straight when locked
  } else if (cmd_angle != lastServoAngle) {
    servoSteer.write(cmd_angle);
    lastServoAngle = cmd_angle;
  }

  // ─ 4. DC Motors ───────────────────────────────────────────
  if (isImmobilized) {
    // Force motors to stop completely
    analogWrite(PIN_ENA, 0);
    analogWrite(PIN_ENB, 0);
    digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, LOW);
    digitalWrite(PIN_IN3, LOW); digitalWrite(PIN_IN4, LOW);
  } else {
    driveMotors(); // Only drive if NOT immobilized
  }

  // ─ 4. DC Motors ───────────────────────────────────────────
  
// --- Update Section 5 in the loop() ---
// ─ 5. Horn ────────────────────────────────────────────────
static bool hornIsActive = false; 

if (cmd_horn) {
  if (!hornIsActive) {
    tone(PIN_BUZZ_HORN, 1000); // Start tone once [cite: 147]
    hornIsActive = true;
  }
} 
else {
  if (hornIsActive) {
    noTone(PIN_BUZZ_HORN);     // Stop tone once [cite: 149]
    digitalWrite(PIN_BUZZ_HORN, LOW); 
    hornIsActive = false;
  }
}//iper servo ─────────────────────────────────────────

  if (cmd_wiper > 10) {

    uint32_t iv=(uint32_t)map(cmd_wiper,10,255,1800,60);

    if (now-wiperTimer>=iv) {

      wiperTimer=now; wiperPos+=wiperDir*4;

      if(wiperPos>=90){wiperPos=90;wiperDir=-1;}

      if(wiperPos<=0) {wiperPos=0; wiperDir= 1;}

      servoWiper.write(wiperPos);

    }

  } else {

    if (wiperPos>0 && (now-wiperTimer>=25)) {

      wiperTimer=now; wiperPos-=3;

      if(wiperPos<0) wiperPos=0;

      servoWiper.write(wiperPos);

    }

  }

// ─ 7. Indicators + beep (Tone Version) ────────────────────
 if (cmd_indic != 0) {
    if (now - indicTimer >= 500) {
        indicTimer = now;
        indicLed = !indicLed;
        
        digitalWrite(PIN_LED_LEFT,  (cmd_indic == 1) ? indicLed : LOW);
        digitalWrite(PIN_LED_RIGHT, (cmd_indic == 2) ? indicLed : LOW);
        
        // Trigger the tone ONLY if the horn is NOT being pressed
        // This prevents the "Timer Crash" that silences both
        if (indicLed && !cmd_horn) {
            tone(PIN_BUZZ_INDIC, 850, 100); 
        }
    }
} else {
    digitalWrite(PIN_LED_LEFT, LOW); 
    digitalWrite(PIN_LED_RIGHT, LOW);
    if (!cmd_horn) noTone(PIN_BUZZ_INDIC); // Only stop tone if horn isn't using it
    indicLed = false;
}
  // ─ 9. Headlight relay ─────────────────────────────────────
  digitalWrite(PIN_RELAY, cmd_headlight?HIGH:LOW);

  // ─ 10. Brake light PWM ────────────────────────────────────
  uint8_t effBrk=max(cmd_brake,cmd_abs);
  analogWrite(PIN_BRAKE_LED,effBrk);
  act_brakePWM=effBrk;

  // ─ 11. Lock/Unlock beep + LED sequence ────────────────────
  // LOCK   = 1 beep  + both LEDs flash together
  // UNLOCK = 2 beeps + LEDs alternate left/right
  if (lockSeq) {
    uint8_t total=(cmd_lock==1)?2:4;
    if (lockCount<total) {
      if (now-lockTimer>=250) {
        lockTimer=now;
        bool on=(lockCount%2==0);
        if(on) tone(PIN_BUZZ_HORN,2000,120);
        if(cmd_lock==1) {
          digitalWrite(PIN_LED_LEFT, on?HIGH:LOW);
          digitalWrite(PIN_LED_RIGHT,on?HIGH:LOW);
        } else {
          digitalWrite(PIN_LED_LEFT,   on?HIGH:LOW);
          digitalWrite(PIN_LED_RIGHT, !on?HIGH:LOW);
        }
        lockCount++;
      }
    } else {
      lockSeq=false; cmd_lock=0;
      digitalWrite(PIN_LED_LEFT,LOW); digitalWrite(PIN_LED_RIGHT,LOW);
    }
  }

  // ─ 12. Speed + RPM displays ───────────────────────────────
  if (now-dispTimer>=500) {
    dispTimer=now;
    float rpm    =(float)cmd_speed/100.0f*200.0f;
    float spdKMH =(rpm/60.0f)*(2.0f*3.14159f*0.03f)*3.6f;
    act_dispSpd=(int)spdKMH; act_dispRPM=(int)rpm;
    dispSpd.showNumberDec(act_dispSpd,false);
    delay(2);
    dispRPM.showNumberDec(act_dispRPM,false);
  }

  // ─ 13. Dashboard ──────────────────────────────────────────
  static uint32_t dashTimer=0;
  if (now-dashTimer>=500) {
    dashTimer=now;
    Serial.print("\033[2J\033[H");
    Serial.println("+==================================================+");
    Serial.println("|   ECU #2  --  ACTUATOR SYSTEM  (UART RECEIVER)  |");
    Serial.println("|  RX=PA10(D2)--ECU1   TX=PA9(D8)--ECU1  @57600  |");
    Serial.println("+==================================================+");
    Serial.print("|  Packets OK: "); pp(rxOK,7);
    Serial.print("   BAD: ");       pp(rxBad,5); Serial.println("                 |");

    if (rxOK==0 && now>4000) {
      Serial.println("|  !! NO PACKETS RECEIVED — Check:               |");
      Serial.println("|     ECU1 D8(PA9) TX --> ECU2 D2(PA10) RX      |");
      Serial.println("|     GND <--> GND  (critical!)                  |");
    }

    Serial.println("+--------------------------------------------------+");
    Serial.print("|  Raw payload: ");
    for(int i=0;i<PAYLOAD_LEN;i++){if(pBuf[i]<0x10)Serial.print('0'); Serial.print(pBuf[i],HEX); Serial.print(' ');}
    Serial.println("  |");
    Serial.println("+--------------------------------------------------+");
    Serial.println("|  DECODED              Value   Bar                |");
    Serial.println("+--------------------------------------------------+");
    Serial.print("|  Servo angle(deg)     "); pp(cmd_angle,5); Serial.print("   "); bar(cmd_angle,180,16); Serial.println("  |");
    Serial.print("|  Motor speed(%)       "); pp(cmd_speed,5); Serial.print("   "); bar(cmd_speed,100,16); Serial.println("  |");
    Serial.print("|  Motor PWM(0-255)     "); pp(act_motorPWM,5); Serial.print("   "); bar(act_motorPWM,255,16); Serial.println("  |");
    Serial.print("|  Brake PWM(effective) "); pp(act_brakePWM,5); Serial.print("   "); bar(act_brakePWM,255,16); Serial.println("  |");
    Serial.print("|  ABS brake            "); pp(cmd_abs,5); Serial.print("   "); bar(cmd_abs,255,16); Serial.println("  |");
    Serial.print("|  Wiper speed          "); pp(cmd_wiper,5); Serial.print("   "); bar(cmd_wiper,255,16); Serial.println("  |");
    Serial.print("|  Wiper position(deg)  "); pp(wiperPos,5); Serial.print("   "); bar(wiperPos,90,16); Serial.println("  |");
    Serial.println("+--------------------------------------------------+");
    Serial.print("|  Direction : ");
    if(cmd_dir==1)Serial.print("FORWARD  "); else if(cmd_dir==2)Serial.print("REVERSE  "); else Serial.print("STOP     ");
    Serial.print("   Motor PWM: "); pp(act_motorPWM,3); Serial.println("           |");
    Serial.print("|  Indicator : ");
    if(cmd_indic==1){Serial.print("LEFT  LED=");Serial.print(indicLed?"ON ":"OFF");}
    else if(cmd_indic==2){Serial.print("RIGHT LED=");Serial.print(indicLed?"ON ":"OFF");}
    else Serial.print("OFF          ");
    Serial.print("   Horn     : "); Serial.print(cmd_horn?"SOUNDING |":"silent   |"); Serial.println();
    Serial.print("|  Parking   : "); Serial.print(cmd_parking?"BLINKING ":"OFF      ");
    Serial.print("   Headlight: "); Serial.print(cmd_headlight?"RELAY ON |":"OFF      |"); Serial.println();
    Serial.print("|  Lock seq  : "); Serial.print(lockSeq?"ACTIVE   ":"idle     ");
    Serial.print("   Lock cmd : ");
    if(cmd_lock==1)Serial.print("LOCK     |"); else if(cmd_lock==2)Serial.print("UNLOCK   |"); else Serial.print("none     |");
    Serial.println();
    Serial.println("+--------------------------------------------------+");
    Serial.print("|  Speed disp: "); pp(act_dispSpd,4); Serial.print(" km/h");
    Serial.print("   RPM disp: "); pp(act_dispRPM,4); Serial.println(" RPM            |");
    Serial.println("+==================================================+");
    Serial.print("  Uptime: "); Serial.print(now/1000);
    Serial.print("s  OK:"); Serial.print(rxOK);
    Serial.print("  BAD:"); Serial.println(rxBad);
  }
  
}
