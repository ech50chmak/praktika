#include <Arduino.h>
#include <Servo.h>

Servo s, ss;

volatile long degL = 0, degR = 0;

int inpins[] = {12, 14, 15, 16, 8, 9};
int outpins[] = {11, 4, 5, 6, 7};

int s1min = 0, s2min = 0, s1max = 0, s2max = 0;

int err_old = 0;

int ik = 0;

void setup() {
  attachInterrupt(0, []{ degL += digitalRead(8) ? -1 : 1; }, FALLING);
  attachInterrupt(1, []{ degR += digitalRead(9) ? 1 : -1; }, FALLING);

  for (int i : inpins) pinMode(i, INPUT);
  for (int i : outpins) pinMode(i, OUTPUT);

  s.attach(10);
  ss.attach(13);
  Serial.begin(9600);
}

void move(int speedL, int speedR){
  digitalWrite(4, speedL<=0);
  digitalWrite(7, speedR<=0);
  analogWrite(6, constrain(abs(speedL), 0, 255));
  analogWrite(5, constrain(abs(speedR), 0, 255));
}

void stop() {
  degL = degR = 0;
  long mil = millis()+200;
  while(mil>millis()) { move(-degL*10, -degR*10); }
  move(0, 0);
}

void ser(Servo &servo, int deg, int dly) {
  for (int i = servo.read(); deg > i ? i < deg : i > deg; deg > i ? i++ : i--) {
    servo.write(i); delay(dly);
  }
}

void callibration(int deg, int sign){
  degL = degR = 0;
  move(60*sign, -60*sign);
  while (abs(degL)<deg) {
    s1min = min(s1min, analogRead(14)); s2min = min(s2min, analogRead(15));
    s1max = max(s1max, analogRead(14)); s2max = max(s2max, analogRead(15));
  }
  stop();
}

int RLS() { return map(analogRead(14), s1min, s1max, 300, 100); }
int LLS() { return map(analogRead(15), s2min, s2max, 300, 100); }

void pd(int v) {
  float kp = 0.15 * v / 70;
  float kd = 1 * v / 70;
  int err = RLS() - LLS();
  int u = err*kp + kd*(err-err_old);
  move(v-u, v+u);
  err_old = err;
}

void pdARC(int v) {
  int err = degL - degR;
  int u = err*0.15 + 1*(err-err_old);
  move(v-u, v+u);
  err_old = err;
}

void arc(int v, int deg) {
  err_old = 0;
  degL = degR = 0;
  while (abs(degL)<deg) { pdARC(v); }
  stop();
}

void LFcross(int s, int cnt, int scan=1) {
  degL = degR = 0;
  err_old = 0;  
  float cur_deg = 0, aim_deg = 600;
  int v_min = 60, v_max = s;
  for (int i = 0; i < cnt; i++) {
    while (1) {
      int v = min(1, cur_deg/aim_deg) * (v_max-v_min) + v_min;
      pd(v);
      cur_deg = (abs(degL)+abs(degR))/2;
      if (RLS() < 200 && LLS() < 200) { break; }
    }
    degL = degR = 0;
    ik = 0;
    int ik_scan = 0;
    while (abs(degL) < 210) { 
      pd(60); 
      if (scan == 1){
        if (IK() < 15) { ik_scan++; }
      } 
    }
    if (ik_scan > 1) {
      ik = 1;
      is_cube();
    }
  }
}

void LFenc(int s, int deg) {
  degL = degR = 0;
  while (abs(degL)<deg) { pd(s); }
  stop();
}

void turn(int side) {    
  stop();
  degL = degR = 0;
  move(140*side, -140*side);
  while (abs(degL)<200) {}
  move(80*side, -80*side);
  while (side == 1 ? RLS() > 200 : LLS() > 200) {}
  move(60*side, -60*side);
  degL = degR = 0;
  while (abs(degL)<35) {}
  stop();
}

int IK() { return analogRead(17) ? 32 * pow(analogRead(17)*5/1024.0, -1.1) : 0; }

void is_cube() {
  if (ik == 1) {
    stop();
    delay(500);
  }
}

void loop() {

}
