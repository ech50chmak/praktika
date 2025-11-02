# PraktikaVsOSH Firmware Notes

Прошивка на Arduino/PlatformIO для двухколёсной платформы с линейными датчиками, энкодерами и сервоприводами. Описание с акцентом на синтаксис и алгоритмы `src/main.cpp`.

## Сборка
- `pio run` — компиляция.
- `pio run -t upload` — прошивка.
- `pio device monitor` — мониторинг UART.

## Функции и алгоритмы

### `setup()`
```cpp
void setup() {
  attachInterrupt(0, []{ degL += digitalRead(8) ? -1 : 1; }, FALLING);
  attachInterrupt(1, []{ degR += digitalRead(9) ? 1 : -1; }, FALLING);
  for (int i : inpins) pinMode(i, INPUT);
  for (int i : outpins) pinMode(i, OUTPUT);
  s.attach(10);
  ss.attach(13);
  Serial.begin(9600);
}
```
- Синтаксис: лямбда без захвата для обработчиков прерываний, диапазонный `for` для инициализации пинов.
- Алгоритм: 1) привязка энкодеров на спадающий фронт, 2) перевод входов/выходов в нужные режимы, 3) привязка серв, 4) запуск UART.

### `move(int speedL, int speedR)`
```cpp
void move(int speedL, int speedR){
  digitalWrite(4, speedL<=0);
  digitalWrite(7, speedR<=0);
  analogWrite(6, constrain(abs(speedL), 0, 255));
  analogWrite(5, constrain(abs(speedR), 0, 255));
}
```
- Синтаксис: тернарное сравнение в аргументе `digitalWrite`, стандартная функция `constrain`.
- Алгоритм: знак скорости задаёт направление, модуль — PWM-скважность.

### `stop()`
```cpp
void stop() {
  degL = degR = 0;
  long mil = millis()+200;
  while(mil>millis()) { move(-degL*10, -degR*10); }
  move(0, 0);
}
```
- Синтаксис: множественное присваивание и цикл без тела кроме вызова `move`.
- Алгоритм: 1) сброс энкодеров, 2) 200 мс обратная коррекция по отклонению, 3) остановка.

### `ser(Servo &servo, int deg, int dly)`
```cpp
void ser(Servo &servo, int deg, int dly) {
  for (int i = servo.read(); deg > i ? i < deg : i > deg; deg > i ? i++ : i--) {
    servo.write(i); delay(dly);
  }
}
```
- Синтаксис: условные выражения внутри заголовка цикла `for`.
- Алгоритм: итеративно увеличивает/уменьшает угол сервопривода, задерживая каждый шаг.

### `callibration(int deg, int sign)`
```cpp
void callibration(int deg, int sign){
  degL = degR = 0;
  move(60*sign, -60*sign);
  while (abs(degL)<deg) {
    s1min = min(s1min, analogRead(14));
    s2min = min(s2min, analogRead(15));
    s1max = max(s1max, analogRead(14));
    s2max = max(s2max, analogRead(15));
  }
  stop();
}
```
- Синтаксис: использование `min/max` из `<Arduino.h>` и вложенного чтения аналоговых пинов.
- Алгоритм: поворот платформы на `deg` тиков с накоплением экстремальных значений датчиков.

### `RLS()` / `LLS()`
```cpp
int RLS() { return map(analogRead(15), s1min, s1max, 300, 100); }
int LLS() { return map(analogRead(16), s2min, s2max, 300, 100); }
```
- Синтаксис: однострочные функции, `map` для линейной нормализации.
- Алгоритм: преобразование аналоговых значений в рабочий диапазон 100–300 (инверсия яркости).

### `pd(int v)`
```cpp
void pd(int v) {
  float kp = 0.15 * v / 70;
  float kd = 1 * v / 70;
  int err = RLS() - LLS();
  int u = err*kp + kd*(err-err_old);
  move(v-u, v+u);
  err_old = err;
}
```
- Синтаксис: плавающие коэффициенты `kp`, `kd`, последующее преобразование в целочисленное управление.
- Алгоритм: ПД-регулятор линии — текущий перекос (`err`) и дифференциальная составляющая корректируют левый/правый мотор.

### `pdARC(int s, int deg)` и `arc(int s, int deg)`
```cpp
void pdARC(int s, int deg) {
  int err = degL - degR;
  int u = err*0.15 + 1*(err-err_old);
  move(s-u, s+u);
  err_old = err;
}

void arc(int s, int deg) {
  err_old = 0;
  degL = degR = 0;
  while (abs(degL)<deg) { pdARC(s, deg); }
  stop();
}
```
- Синтаксис: повторное использование регулятора внутри цикла, передача параметров по значению.
- Алгоритм: 1) сброс ошибок, 2) удержание заданной скорости и выравнивание энкодеров, 3) остановка по достижении `deg`.

### `LFcross(int s, int cnt, int scan)`
```cpp
void LFcross(int s, int cnt, int scan) {
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
```
- Синтаксис: вложенные `while`, целочисленное `min`, условные блоки с инкрементом.
- Алгоритм: 1) плавный разгон до перекрёстка, 2) дополнительный проезд фиксированной длины, 3) повторное сканирование `IK()` и вызов `is_cube()` при множественных срабатываниях.

### `LFenc(int s, int deg)`
```cpp
void LFenc(int s, int deg) {
  degL = degR = 0;
  while (abs(degL)<deg) { pd(s); }
  stop();
}
```
- Синтаксис: цикл с условием по энкодеру и вызов `pd`.
- Алгоритм: движение по линии на заданное количество тиков с постоянной скоростью.

### `turn(int side)`
```cpp
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
```
- Синтаксис: тернарный оператор внутри условия `while`, серийные вызовы `move` с разными коэффициентами.
- Алгоритм: ступенчатое замедление при повороте на месте с контролем по энкодерам и линейному датчику.

### `IK()` и `is_cube()`
```cpp
int IK() { return 32 * pow(analogRead(17)*5/1024.0, -1.1); }

void is_cube() {
  if (ik == 1) {
    stop();
    delay(500);
  }
}
```
- Синтаксис: функция высшего порядка `pow` для обратной аппроксимации, условный вызов `stop`.
- Алгоритм: оценка расстояния по ИК-датчику и реакция при установленном флаге захвата.

### `loop()`
```cpp
void loop() {

}
```
- Синтаксис: пустой каркас главного цикла.
- Алгоритм: пользователь должен заполнить последовательностью калибровок и манёвров под задачу.

## Замечания для доработки
- Инициализировать `s1min/s2min` и `s1max/s2max` безопасными значениями и откорректировать пины в `callibration`.
- Обеспечить атомарность доступа к `degL/degR` (`noInterrupts()` / `interrupts()`).
- Добавить таймауты в бесконечные `while`.
- Ограничить результат `IK()` при нулевом входе.
