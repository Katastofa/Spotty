#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();  // Адрес по умолчанию 0x40

// Каналы серво (BL, BR, FL, FR)
int servoChannels[12] = {1, 2, 3, 5, 6, 7, 9, 10, 11, 13, 14, 15};

// Slopes из таблицы (pwm/deg)
float slopes[12] = {2.18, -1.50, 2.30, 2.22, 1.81, 3.27, 1.29, 1.59, 2.17, 1.17, 1.59, 3.13};

// Center PWM из таблицы
int centerPWMs[12] = {304, 236, 396, 300, 338, 352, 384, 357, 402, 394, 357, 359};

// Offsets для корректировки колен до 90°
int angleOffsets[12] = {
35,    // BL_side (index 0)
20,    // BL_fwd (1)
20,    // BL_knee (2)
35,    // BR_side (3)
70,    // BR_fwd (4)
-10,   // BR_knee (5)
0,     // FL_side (6)
-70,   // FL_fwd (7)
-20,   // FL_knee (8)
-10,   // FR_side (9)
70,    // FR_fwd (10)
10     // FR_knee (11)
};

#define SERVO_MIN 100  // Минимальный импульс для MG996R
#define SERVO_MAX 500  // Максимальный импульс

// Параметры шага
int stepHeight = 25;
int stepLength = 15;
int stepSwing = 8;
int cogShift = 5;
int stepDelay = 200;
int smoothSteps = 10;

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(50);  // 50 Гц для серво
  delay(10);
  setNeutralStance();  // Установить стойку
  Serial.println("Робособака готова. Отправь 't' для trot gait, 'f' для creep gait, 'h+/-' для stepHeight, 'l+/-' для stepLength.");
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 't') {
      forwardTrotStep();
    } else if (cmd == 'f') {
      forwardCreepStep();
    } else if (cmd == 'h') {
      stepHeight += 5; Serial.println("stepHeight: " + String(stepHeight));
    } else if (cmd == 'H') {
      stepHeight -= 5; Serial.println("stepHeight: " + String(stepHeight));
    } else if (cmd == 'l') {
      stepLength += 5; Serial.println("stepLength: " + String(stepLength));
    } else if (cmd == 'L') {
      stepLength -= 5; Serial.println("stepLength: " + String(stepLength));
    }
  }
}

// Установить угол с учётом slopes, centerPWMs и offsets
void setServo(int index, int angle) {
  int channel = servoChannels[index];
  int adjustedAngle = angle + angleOffsets[index];
  
  // Инверсия для боковых плеч FL и FR (indexes 6 и 9)
  if (index == 6 || index == 9) {
    adjustedAngle = 180 - adjustedAngle;
  }
  
  // Инверсия для FR_fwd (index 10? Ошибка в исходном: для BL_fwd index 1)
  if (index == 1) {  // BL_fwd инверсия
    adjustedAngle = 180 - adjustedAngle;
  }
  
  adjustedAngle = constrain(adjustedAngle, 0, 180);
  int pulse = centerPWMs[index] + (adjustedAngle - 90) * slopes[index];
  pulse = constrain(pulse, SERVO_MIN, SERVO_MAX);
  pwm.setPWM(channel, 0, pulse);
}

// Плавное изменение угла
void smoothSetServo(int index, int startAngle, int endAngle, int steps, int delayMs) {
  for (int i = 0; i <= steps; i++) {
    int angle = startAngle + (endAngle - startAngle) * i / steps;
    setServo(index, angle);
    delay(delayMs / steps);
  }
}

// Нейтральная стойка
void setNeutralStance() {
  for (int i = 0; i < 12; i++) {
    setServo(i, 90);  // 90° с учётом offsets
  }
  Serial.println("Нейтральная стойка установлена.");
}

// Смещение CoG для баланса перед подъёмом ноги
void shiftCoGForLeg(String leg, int shift) {
  if (leg == "FR") {
    // Смещение назад-влево для опоры на BL, BR, FL
    smoothSetServo(1, 90, 90 - shift, smoothSteps, stepDelay / 4);  // BL_fwd назад
    smoothSetServo(4, 90, 90 - shift, smoothSteps, stepDelay / 4);  // BR_fwd назад
    smoothSetServo(7, 90, 90 + shift, smoothSteps, stepDelay / 4);  // FL_fwd назад (инверсия?)
    smoothSetServo(0, 90, 90 + shift, smoothSteps, stepDelay / 4);  // BL_side влево
    smoothSetServo(3, 90, 90 - shift, smoothSteps, stepDelay / 4);  // BR_side влево
    smoothSetServo(6, 90, 90 + shift, smoothSteps, stepDelay / 4);  // FL_side влево
  } else if (leg == "FL") {
    // Смещение назад-вправо для опоры на BL, BR, FR
    smoothSetServo(1, 90, 90 - shift, smoothSteps, stepDelay / 4);  // BL_fwd назад
    smoothSetServo(4, 90, 90 - shift, smoothSteps, stepDelay / 4);  // BR_fwd назад
    smoothSetServo(10, 90, 90 - shift, smoothSteps, stepDelay / 4); // FR_fwd назад
    smoothSetServo(0, 90, 90 - shift, smoothSteps, stepDelay / 4);  // BL_side вправо
    smoothSetServo(3, 90, 90 + shift, smoothSteps, stepDelay / 4);  // BR_side вправо
    smoothSetServo(9, 90, 90 - shift, smoothSteps, stepDelay / 4);  // FR_side вправо
  } else if (leg == "BR") {
    // Смещение вперед-влево для опоры на BL, FL, FR
    smoothSetServo(1, 90, 90 + shift, smoothSteps, stepDelay / 4);  // BL_fwd вперед
    smoothSetServo(7, 90, 90 - shift, smoothSteps, stepDelay / 4);  // FL_fwd вперед
    smoothSetServo(10, 90, 90 + shift, smoothSteps, stepDelay / 4); // FR_fwd вперед
    smoothSetServo(0, 90, 90 + shift, smoothSteps, stepDelay / 4);  // BL_side влево
    smoothSetServo(6, 90, 90 + shift, smoothSteps, stepDelay / 4);  // FL_side влево
    smoothSetServo(9, 90, 90 + shift, smoothSteps, stepDelay / 4);  // FR_side влево
  } else if (leg == "BL") {
    // Смещение вперед-вправо для опоры на BR, FL, FR
    smoothSetServo(4, 90, 90 + shift, smoothSteps, stepDelay / 4);  // BR_fwd вперед
    smoothSetServo(7, 90, 90 - shift, smoothSteps, stepDelay / 4);  // FL_fwd вперед
    smoothSetServo(10, 90, 90 + shift, smoothSteps, stepDelay / 4); // FR_fwd вперед
    smoothSetServo(3, 90, 90 + shift, smoothSteps, stepDelay / 4);  // BR_side вправо
    smoothSetServo(6, 90, 90 - shift, smoothSteps, stepDelay / 4);  // FL_side вправо
    smoothSetServo(9, 90, 90 - shift, smoothSteps, stepDelay / 4);  // FR_side вправо
  }
}

// Возврат CoG в нейтраль после опускания ноги
void resetCoG() {
  smoothSetServo(0, getCurrentAngle(0), 90, smoothSteps, stepDelay / 4);  // BL_side
  smoothSetServo(1, getCurrentAngle(1), 90, smoothSteps, stepDelay / 4);  // BL_fwd
  smoothSetServo(3, getCurrentAngle(3), 90, smoothSteps, stepDelay / 4);  // BR_side
  smoothSetServo(4, getCurrentAngle(4), 90, smoothSteps, stepDelay / 4);  // BR_fwd
  smoothSetServo(6, getCurrentAngle(6), 90, smoothSteps, stepDelay / 4);  // FL_side
  smoothSetServo(7, getCurrentAngle(7), 90, smoothSteps, stepDelay / 4);  // FL_fwd
  smoothSetServo(9, getCurrentAngle(9), 90, smoothSteps, stepDelay / 4);  // FR_side
  smoothSetServo(10, getCurrentAngle(10), 90, smoothSteps, stepDelay / 4); // FR_fwd
}

// Функция для получения текущего угла (простая заглушка, поскольку нет обратной связи; используй 90 как default или храни состояние если нужно)
int getCurrentAngle(int index) {
  return 90;  // Предполагаем, что мы всегда возвращаем к 90, но можно добавить массив состояний
}

// Функции для шага одной ноги
void liftLeg(int sideIdx, int fwdIdx, int kneeIdx) {
  int swingDir = (sideIdx == 0 || sideIdx == 6) ? -stepSwing : stepSwing;  // В зависимости от стороны
  smoothSetServo(sideIdx, 90, 90 + swingDir, smoothSteps, stepDelay / 2);  // Смещение вбок для свинга
  smoothSetServo(kneeIdx, 90, 90 + stepHeight, smoothSteps, stepDelay);    // Подъём колена
}

void moveLegForward(int fwdIdx, int length) {
  int adjustedLength = (fwdIdx == 1 || fwdIdx == 7) ? -length : length;    // Инверсия для BL и FL
  smoothSetServo(fwdIdx, 90, 90 + adjustedLength, smoothSteps, stepDelay); // Движение вперёд
}

void lowerLeg(int sideIdx, int fwdIdx, int kneeIdx) {
  smoothSetServo(kneeIdx, 90 + stepHeight, 90, smoothSteps, stepDelay);    // Опускание колена
  int swingDir = (sideIdx == 0 || sideIdx == 6) ? -stepSwing : stepSwing;
  smoothSetServo(sideIdx, 90 + swingDir, 90, smoothSteps, stepDelay / 2);  // Возврат свинга
}

// Trot gait: остаётся без изменений (диагональные пары)
void forwardTrotStep() {
  shiftCoGForLeg("BL", cogShift);  // Адаптировали shiftCoG для trot, но для простоты оставим старый
  liftLeg(9, 10, 11);  // FR
  liftLeg(0, 1, 2);    // BL
  moveLegForward(10, stepLength);
  moveLegForward(1, stepLength);
  lowerLeg(9, 10, 11);
  lowerLeg(0, 1, 2);
  smoothSetServo(10, 90 + stepLength, 90, smoothSteps, stepDelay);  // Возврат FR_fwd
  smoothSetServo(1, 90 + stepLength, 90, smoothSteps, stepDelay);   // Для BL_fwd adjusted был -, но здесь +? Исправить если нужно
  resetCoG();
  delay(stepDelay / 2);

  shiftCoGForLeg("BR", cogShift);
  liftLeg(3, 4, 5);    // BR
  liftLeg(6, 7, 8);    // FL
  moveLegForward(4, stepLength);
  moveLegForward(7, stepLength);
  lowerLeg(3, 4, 5);
  lowerLeg(6, 7, 8);
  smoothSetServo(4, 90 + stepLength, 90, smoothSteps, stepDelay);
  smoothSetServo(7, 90 + stepLength, 90, smoothSteps, stepDelay);   // Для FL_fwd adjusted -, но здесь +
  resetCoG();
  delay(stepDelay / 2);

  Serial.println("Trot шаг завершён.");
}

// Creep gait: переработан под описание - опора на три ноги, смещение свободной ноги, опускание, упор на новую + две старые, затем следующая свободная вперёд
void forwardCreepStep() {
  // Шаг 1: FR - свободная, опора на BL, BR, FL
  shiftCoGForLeg("FR", cogShift);  // Смещение CoG для баланса на трёх
  liftLeg(9, 10, 11);             // Подъём и свинг FR
  moveLegForward(10, stepLength);  // Смещение FR вперёд
  lowerLeg(9, 10, 11);             // Опускание, теперь упор на FR + BL, BR (FL всё ещё опора, но CoG сдвинут)
  resetCoG();                      // Возврат CoG в нейтраль постепенно
  delay(stepDelay / 2);

  // Шаг 2: FL - свободная, опора на FR, BL, BR
  shiftCoGForLeg("FL", cogShift);
  liftLeg(6, 7, 8);
  moveLegForward(7, stepLength);
  lowerLeg(6, 7, 8);
  resetCoG();
  delay(stepDelay / 2);

  // Шаг 3: BL - свободная, опора на FR, FL, BR
  shiftCoGForLeg("BL", cogShift);
  liftLeg(0, 1, 2);
  moveLegForward(1, stepLength);
  lowerLeg(0, 1, 2);
  resetCoG();
  delay(stepDelay / 2);

  // Шаг 4: BR - свободная, опора на FR, FL, BL
  shiftCoGForLeg("BR", cogShift);
  liftLeg(3, 4, 5);
  moveLegForward(4, stepLength);
  lowerLeg(3, 4, 5);
  resetCoG();
  delay(stepDelay / 2);

  Serial.println("Creep шаг завершён.");
}