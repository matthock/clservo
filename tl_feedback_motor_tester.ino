#include <TimedPID.h>
#include <PinChangeInterrupt.h>
#include "fastpwm.h"

#define MOTOR_DIR 4
#define MOTOR_PWM 6
#define MOTOR_BRK 5
#define MOTOR_CURRENT A0
#define MOTOR_ENCODER_A 8
#define MOTOR_ENCODER_B 9

long pos = 5000;
long target = 2500;
boolean forward = true;
unsigned long current_time = 0;
unsigned long time_since_eval = 0;
unsigned long time_per_eval = 1000;
unsigned long time_since_led = 0;
unsigned long time_per_led = 1000000;
float current = 0;
float err_avg = 0;
long move_target = 0;
TimedPID pid = TimedPID(100, 1, 0.1);

float cpr = 20;
float gear_ratio = 31.25 * 4;
float cpr_out = cpr * gear_ratio;
float cps = cpr_out / 200;

void set_pwm_4d(unsigned int value);

void on_encoder_a() {
  forward = digitalRead(MOTOR_ENCODER_A) == digitalRead(MOTOR_ENCODER_B);
  pos += forward ? 1 : -1;
}

void on_encoder_b() {
  pos += forward ? 1 : -1;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);  
  pinMode(MOTOR_DIR, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_BRK, OUTPUT);
  pinMode(MOTOR_ENCODER_A, INPUT_PULLUP);
  pinMode(MOTOR_ENCODER_B, INPUT_PULLUP);
  analogRead(MOTOR_CURRENT);
  //pinMode(MOTOR_CURRENT, INPUT);
  attachPCINT(digitalPinToPCINT(MOTOR_ENCODER_A), &on_encoder_a, CHANGE);
  attachPCINT(digitalPinToPCINT(MOTOR_ENCODER_B), &on_encoder_b, CHANGE);
  digitalWrite(MOTOR_DIR, HIGH);
  pid.setCmdRange(-1024, 1024);
  current_time = millis();

  // Set up faster PWM on Timer 4 (using for Pin 6/OC4D)
  TCCR4A = 0; //This is for outputs A and B, we are just using D
  TCCR4B = TCCR4B & (!0b00001111) | _BV(CS42) | _BV(PWM4X); // 8x clock divider, Timer 4 PWM Mode
  TCCR4C = TCCR4C | _BV(PWM4D) | _BV(COM4D1) | !_BV(COM4D0); // Normal PWM on 4D
  TCCR4D = TCCR4D | !_BV(WGM41) | !_BV(WGM40); // Fast PWM
  PLLFRQ = (PLLFRQ&0xCF)|0x30; // PLL/2
  set_pwm_4d(0);
  //analogWrite(MOTOR_PWM, 0);


  // Finished with setup!
  Serial.print("Startup\n");
}

void set_pwm_4d(unsigned int value) {
  TC4H = value >> 8 & 0x11;
  TCNT4 = value & 0xFF;
}

long loop_in_window = 0;

void loop() {
  unsigned long new_time = micros();
  long elapsed = new_time - current_time;
  current_time = new_time;
  
  time_since_eval += elapsed;
  if (time_since_eval >= time_per_eval) {
    time_since_eval = 0;
    float this_current = analogRead(MOTOR_CURRENT);
    target += move_target;
    this_current = this_current * 5.0 /* vref */ * 500.0 /* mA/V */ / 1024.0 /* 10 bit ADC */;
    current = current * 24.0 / 25.0 + this_current / 25.0;
    err_avg = err_avg * 24.0 / 25.0  + abs(target - pos) / 25.0;
  }

  float move_speed = pid.getCmdAutoStep(target, pos);
  
  if (move_speed != 0) {
    bool target_direction = move_speed < 0 ? HIGH : LOW;
    set_pwm_4d(min(abs(move_speed) / 4, 255) * 4);
    digitalWrite(MOTOR_DIR, target_direction);
    digitalWrite(MOTOR_BRK, 0);
  } else {
    set_pwm_4d(0);
    //analogWrite(MOTOR_PWM, 0);
    digitalWrite(MOTOR_BRK, 1);
  }

  time_since_led += elapsed;
  if (time_since_led >= time_per_led) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    float this_current = analogRead(MOTOR_CURRENT);
    Serial.print(current);
    Serial.print("mA, Distance: ");
    Serial.print(target - pos);
    Serial.print(" encoder steps, ");
    Serial.print((target - pos) / cps);
    Serial.print(" microsteps, Microstep Err Average: ");
    Serial.print(err_avg / cps);
    Serial.print(" steps\n");
    time_since_led = 0;
    if (move_target == 0) {
      move_target = 1;
    } else {
      target -= 500;
      move_target = 0;
    }
  }
}
