/* ============================================================================
 *  nova_sm3.ino - CANONICAL Arduino firmware for the Spotty quadruped
 * ----------------------------------------------------------------------------
 *  Consolidation of three prior sketches. NOT a rewrite, NOT a redesign.
 *
 *  Lineage:
 *    code/dog.txt              (2025-10-09) - structural base: full command set,
 *                                             leg map, angleOffsets, step params
 *    code/dogg v.2.1 copy.ino  (2025-09-30) - identical inversion set; cross-check
 *    code/dogg v.2.ino         (2025-09-30) - source of shiftCoGForLeg()/resetCoG()
 *
 *  Architecture retained verbatim:
 *    - Adafruit PCA9685 @ I2C default 0x40, 50 Hz
 *    - 12 servos on channels {1,2,3,5,6,7,9,10,11,13,14,15}
 *    - open-loop PWM, no feedback, no sensors, no kinematics
 *
 *  CALIBRATION IS FROZEN. servoChannels[], slopes[], centerPWMs[] and
 *  angleOffsets[] are copied value-for-value from code/dog.txt. Their physical
 *  correctness is UNKNOWN and was NOT recomputed, "corrected" or reordered.
 *
 *  SAFETY - READ BEFORE POWERING SERVOS
 *    - This firmware has NEVER been run on hardware and has NEVER been compiled.
 *    - setup() does NOT command any servo. There is no startup movement.
 *    - Motion commands are refused until the operator explicitly establishes the
 *      neutral reference pose by sending '2'. See stanceKnown below.
 *    - There is still NO emergency stop and NO torque-off. Motion is blocking.
 *      Cut power to stop the robot.
 * ==========================================================================*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();  // Адрес по умолчанию 0x40

/* --------------------------------------------------------------------------
 * CALIBRATION BLOCK - FROZEN. Values identical to code/dog.txt lines 7,10,13,16-29.
 * Do not edit without physical re-measurement on the actual robot.
 * ------------------------------------------------------------------------*/

// Каналы серво (PCA9685 channels). dog.txt:7 - identical in all three sources.
int servoChannels[12] = {1, 2, 3, 5, 6, 7, 9, 10, 11, 13, 14, 15};

// Slopes из таблицы (pwm/deg). dog.txt:10 - identical in all three sources.
float slopes[12] = {2.18, -1.50, 2.30, 2.22, 1.81, 3.27, 1.29, 1.59, 2.17, 1.17, 1.59, 3.13};

// Center PWM из таблицы. dog.txt:13 - identical in all three sources.
int centerPWMs[12] = {304, 236, 396, 300, 338, 352, 384, 357, 402, 394, 357, 359};

// Offsets для корректировки колен до 90°. dog.txt:16-29 verbatim.
// NOTE: these values differ from the v2/v2.1 set. dog.txt changed them together
// with the leg map below; the two are kept together as one consistent pair.
int angleOffsets[12] = {
  35,    // FR_side (index 0)
  50,    // FR_fwd (1)
  0,     // FR_knee (2)
  30,    // FL_side (3)
  50,    // FL_fwd (4)
  -5,    // FL_knee (5)
  0,     // BR_side (6)
  80,    // BR_fwd (7)
  -30,   // BR_knee (8)
  -10,   // BL_side (9)
  90,    // BL_fwd (10)
  15     // BL_knee (11)
};

#define SERVO_MIN 100  // Минимальный импульс для MG996R
#define SERVO_MAX 500  // Максимальный импульс

/* --------------------------------------------------------------------------
 * JOINT DIRECTION INVERSION - see FIX 3 in the report.
 *
 * All three source files apply "adjustedAngle = 180 - adjustedAngle" to the
 * SAME four raw indices {1, 6, 7, 9} (dog.txt:124,129 and v2.1:119,124 are
 * byte-identical; v2.ino:76,81 uses {1,6,9}). The runtime behaviour of the two
 * newest files is therefore IDENTICAL and is preserved here exactly.
 *
 * What is NOT identical is what those indices are called. Under the v2.1 map
 * {1,6,7,9} = BL_fwd, FL_side, FL_fwd, FR_side - which matches the source
 * comments. Under the dog.txt map used below they are FR_fwd, BR_side, BR_fwd,
 * BL_side - which does NOT match those same comments.
 *
 * WHICH LABELLING IS PHYSICALLY CORRECT IS UNKNOWN and cannot be determined
 * from the repository. The contradiction is made explicit here instead of
 * being hidden in magic numbers. Behaviour is unchanged from dog.txt/v2.1.
 * ------------------------------------------------------------------------*/
const bool JOINT_INVERTED[12] = {
  false,  //  0
  true,   //  1  <-- inverted (dog.txt map: FR_fwd / v2.1 map: BL_fwd)
  false,  //  2
  false,  //  3
  false,  //  4
  false,  //  5
  true,   //  6  <-- inverted (dog.txt map: BR_side / v2.1 map: FL_side)
  true,   //  7  <-- inverted (dog.txt map: BR_fwd  / v2.1 map: FL_fwd)
  false,  //  8
  true,   //  9  <-- inverted (dog.txt map: BL_side / v2.1 map: FR_side)
  false,  // 10
  false   // 11
};

// Параметры шага. dog.txt:35-40 verbatim.
int stepHeight = 40;
int stepLength = 15;
int stepSwing = 10;
int cogShift = 10;
int stepDelay = 300;
int smoothSteps = 60;

/* --------------------------------------------------------------------------
 * LEG MAP - taken from code/dog.txt:43-46.
 * v2.1/v2 use the exact inverse (BL={0,1,2} ... FR={9,10,11}).
 * WHICH IS PHYSICALLY CORRECT IS UNKNOWN. dog.txt's map is used because its
 * angleOffsets[] above were authored against it.
 * ------------------------------------------------------------------------*/
const int FR[3] = {0, 1, 2};
const int FL[3] = {3, 4, 5};
const int BR[3] = {6, 7, 8};
const int BL[3] = {9, 10, 11};

/* --------------------------------------------------------------------------
 * COMMANDED-ANGLE STATE - see FIX 5.
 * The hardware gives no position feedback. Every source file worked around this
 * by hard-coding "90" as the start of each ramp, which is wrong whenever the
 * previous command left the joint elsewhere. We instead remember what we last
 * commanded and ramp from there.
 *
 * stanceKnown is false until the operator commands the neutral stance, because
 * before that the physical position of every joint is genuinely unknown.
 * ------------------------------------------------------------------------*/
int commandedAngle[12];
bool stanceKnown = false;

// Serial line buffer - see FIX 2.
const byte CMD_BUF_LEN = 8;
char cmdBuf[CMD_BUF_LEN];
byte cmdLen = 0;

/* ---------------------------- prototypes -------------------------------- */
void setServo(int index, int angle);
void smoothSetServo(int index, int startAngle, int endAngle, int steps, int delayMs);
void moveJointTo(int index, int endAngle, int delayMs);
void setNeutralStance();
const int* getLegFromChar(char c);
bool requireStance();
void handleCommand(char cmd, char legChar);
void verifyJointMap();
void setLyingPose();
void setStandingPose();
void setSittingPose();
void liftLowerLeg(const int leg[3]);
void givePaw(const int leg[3]);
void abductLeg(const int leg[3]);
void bendExtendKnee(const int leg[3]);
void tiltBodyForward();
void tiltBodyBackward();
void tiltBodyLeft();
void tiltBodyRight();
void lowerBody();
void raiseBody();
void liftLeg(int sideIdx, int fwdIdx, int kneeIdx);
void moveLegForward(int fwdIdx, int length);
void lowerLeg(int sideIdx, int fwdIdx, int kneeIdx);
void returnFwdToNeutral(const int leg[3]);
void shiftCoGForLeg(const int leg[3], int shift);
void resetCoG();
void forwardTrotStep();
void forwardCreepStep();
void forwardPaceStep();
void forwardGallopStep();
void turnInPlace();
void smoothTurnDuringMovement();
void jumpTrick();
void danceTrick();
void bowTrick();
void shakeBodyTrick();

/* ------------------------------- setup ---------------------------------- */
void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(50);  // 50 Гц для серво
  delay(10);

  // No servo position is commanded here. All PCA9685 outputs stay off after
  // begin(), so no torque is applied at power-on. This differs deliberately
  // from all three source files, which called setNeutralStance() in setup().
  for (int i = 0; i < 12; i++) {
    commandedAngle[i] = 90;  // placeholder only; not trusted until stanceKnown
  }

  verifyJointMap();

  Serial.println("Nova SM3. ДВИЖЕНИЕ ЗАБЛОКИРОВАНО.");
  Serial.println("Отправьте '2' чтобы установить нейтральную стойку и разблокировать движение.");
  Serial.println("ВНИМАНИЕ: '2' мгновенно подаёт нейтраль на все 12 серво.");
  Serial.println("Терминал должен добавлять перевод строки (Newline или CR).");
  Serial.println("Статические позы: 1 - лежа, 2 - стоя, 3 - сидя");
  Serial.println("Локальные движения (добавь букву лапы F/L/B/R, напр. 4F): 4 - поднять/опустить, 5 - дай лапу, 6 - отведение в сторону, 7 - сгиб/разгиб колена");
  Serial.println("Перемещение корпуса: 8 - наклон вперед, 9 - назад, a - влево, b - вправо, c - опустить корпус, d - поднять");
  Serial.println("Ходьба: t - трот, f - кроул, p - пейс, g - галоп");
  Serial.println("Повороты: r - на месте, s - плавный во время движения");
  Serial.println("Трюки: j - прыжок, n - танец, o - поклон, k - качание корпусом");
  Serial.println("h+/- для stepHeight, l+/- для stepLength");
}

/* -------------------------------- loop ----------------------------------
 * FIX 2: line-buffered parser. CR and LF terminate a command and can never be
 * read as a leg selector, and the second character is no longer subject to a
 * race against UART arrival timing.
 * ------------------------------------------------------------------------*/
void loop() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\r' || c == '\n') {
      if (cmdLen > 0) {
        char cmd = cmdBuf[0];
        char legChar = (cmdLen >= 2) ? cmdBuf[1] : '\0';
        cmdLen = 0;
        handleCommand(cmd, legChar);
      }
      continue;
    }

    if (c == ' ' || c == '\t') {
      continue;  // whitespace is never part of a command token
    }

    if (cmdLen < CMD_BUF_LEN - 1) {
      cmdBuf[cmdLen++] = c;
    }
  }
}

/* ---------------------------- command dispatch --------------------------- */
void handleCommand(char cmd, char legChar) {
  const int* leg = NULL;

  switch (cmd) {
    // --- tuning: no actuation, always allowed ---
    case 'h': stepHeight += 5; Serial.println("stepHeight: " + String(stepHeight)); return;
    case 'H': stepHeight -= 5; Serial.println("stepHeight: " + String(stepHeight)); return;
    case 'l': stepLength += 5; Serial.println("stepLength: " + String(stepLength)); return;
    case 'L': stepLength -= 5; Serial.println("stepLength: " + String(stepLength)); return;

    // --- establishes the reference pose; the only command allowed first ---
    case '2': setStandingPose(); return;
    default: break;
  }

  if (!requireStance()) return;

  switch (cmd) {
    case '1': setLyingPose(); break;
    case '3': setSittingPose(); break;

    // FIX 1: a missing or unrecognised leg letter aborts the command instead of
    // dereferencing NULL (dog.txt) or silently moving BL (v2.1).
    case '4':
    case '5':
    case '6':
    case '7':
      leg = getLegFromChar(legChar);
      if (leg == NULL) {
        Serial.println("Команда отменена: лапа не выбрана.");
        break;
      }
      if (cmd == '4') liftLowerLeg(leg);
      else if (cmd == '5') givePaw(leg);
      else if (cmd == '6') abductLeg(leg);
      else bendExtendKnee(leg);
      break;

    case '8': tiltBodyForward(); break;
    case '9': tiltBodyBackward(); break;
    case 'a': tiltBodyLeft(); break;
    case 'b': tiltBodyRight(); break;
    case 'c': lowerBody(); break;
    case 'd': raiseBody(); break;
    case 't': forwardTrotStep(); break;
    case 'f': forwardCreepStep(); break;
    case 'p': forwardPaceStep(); break;
    case 'g': forwardGallopStep(); break;
    case 'r': turnInPlace(); break;
    case 's': smoothTurnDuringMovement(); break;
    case 'j': jumpTrick(); break;
    case 'n': danceTrick(); break;
    case 'o': bowTrick(); break;
    case 'k': shakeBodyTrick(); break;
    default:
      Serial.println("Неизвестная команда.");
      break;
  }
}

// FIX 5 (precondition): refuse to ramp from an unknown physical position.
bool requireStance() {
  if (stanceKnown) return true;
  Serial.println("Движение заблокировано: сначала отправьте '2' (нейтральная стойка).");
  return false;
}

/* FIX 1: NULL for both a missing and an unrecognised leg letter.
 * dog.txt:106 already returned NULL for a missing letter but loop() never
 * checked it; dog.txt:114 silently substituted BL for a typo, which actuates a
 * leg the operator did not name. Both now abort the command. */
const int* getLegFromChar(char c) {
  switch (c) {
    case 'F': return FR;
    case 'L': return FL;
    case 'B': return BL;
    case 'R': return BR;
    case '\0':
      Serial.println("Ошибка: укажите букву лапы (F, L, B, R)");
      return NULL;
    default:
      Serial.println("Ошибка: неизвестная лапа (ожидается F, L, B, R)");
      return NULL;
  }
}

/* ------------------------------ servo layer ------------------------------ */

// Structurally identical to dog.txt:119-137. The only change is that the two
// hard-coded inversion conditions are replaced by the JOINT_INVERTED[] table
// (same four indices), and the commanded angle is recorded.
void setServo(int index, int angle) {
  if (index < 0 || index > 11) return;

  int channel = servoChannels[index];
  int adjustedAngle = angle + angleOffsets[index];

  if (JOINT_INVERTED[index]) {
    adjustedAngle = 180 - adjustedAngle;
  }

  adjustedAngle = constrain(adjustedAngle, 0, 180);
  int pulse = centerPWMs[index] + (adjustedAngle - 90) * slopes[index];
  pulse = constrain(pulse, SERVO_MIN, SERVO_MAX);
  pwm.setPWM(channel, 0, pulse);

  commandedAngle[index] = angle;
}

// dog.txt:140-146, plus a guard against a zero step count (latent divide-by-zero).
void smoothSetServo(int index, int startAngle, int endAngle, int steps, int delayMs) {
  if (steps < 1) steps = 1;
  for (int i = 0; i <= steps; i++) {
    int angle = startAngle + (endAngle - startAngle) * i / steps;
    setServo(index, angle);
    delay(delayMs / steps);
  }
}

/* FIX 5: ramp from the angle actually last commanded, not from an assumed 90. */
void moveJointTo(int index, int endAngle, int delayMs) {
  if (index < 0 || index > 11) return;
  smoothSetServo(index, commandedAngle[index], endAngle, smoothSteps, delayMs);
}

// dog.txt:149-154 verbatim (direct, non-ramped writes), plus it is now the
// operation that establishes the known reference pose.
void setNeutralStance() {
  for (int i = 0; i < 12; i++) {
    setServo(i, 90);
  }
  stanceKnown = true;
  Serial.println("Нейтральная стойка установлена.");
}

// Reports only. Writes nothing to the servo driver.
void verifyJointMap() {
  bool seen[12];
  for (int i = 0; i < 12; i++) seen[i] = false;

  const int* legs[4] = {FR, FL, BR, BL};
  bool ok = true;
  for (int l = 0; l < 4; l++) {
    for (int j = 0; j < 3; j++) {
      int idx = legs[l][j];
      if (idx < 0 || idx > 11 || seen[idx]) { ok = false; }
      else seen[idx] = true;
    }
  }
  for (int i = 0; i < 12; i++) if (!seen[i]) ok = false;

  Serial.println(ok ? "Карта суставов: OK (12 уникальных индексов)"
                    : "КАРТА СУСТАВОВ НЕКОРРЕКТНА - проверьте FR/FL/BR/BL");
}

/* ------------------------------ 1. Статика ------------------------------- */

// dog.txt:158-168 verbatim, targets unchanged (including the asymmetric 30/150
// "Тест инверсии" experiment, whose correctness is UNKNOWN).
void setLyingPose() {
  int knees[4] = {2, 5, 8, 11};  // FR_knee, FL_knee, BR_knee, BL_knee
  for (int i = 0; i < 4; i++) {
    if (knees[i] == FL[2] || knees[i] == BL[2]) {
      moveJointTo(knees[i], 30, stepDelay);   // Тест инверсии
    } else {
      moveJointTo(knees[i], 150, stepDelay);
    }
  }
  Serial.println("Поза: Лёжа");
}

void setStandingPose() {
  setNeutralStance();
  Serial.println("Поза: Стоя");
}

// dog.txt:176-184 verbatim targets (BL knee 60 vs BR knee 120 asymmetry preserved).
void setSittingPose() {
  moveJointTo(BL[2], 60, stepDelay);
  moveJointTo(BR[2], 120, stepDelay);
  moveJointTo(FL[2], 90, stepDelay);
  moveJointTo(FR[2], 90, stepDelay);
  moveJointTo(BL[1], 60, stepDelay);
  moveJointTo(BR[1], 60, stepDelay);
  Serial.println("Поза: Сидя");
}

/* ------------------------ 2. Локальные движения лап ---------------------- */

void liftLowerLeg(const int leg[3]) {
  moveJointTo(leg[2], 90 - stepHeight, stepDelay);
  delay(1000);
  moveJointTo(leg[2], 90, stepDelay);
  Serial.println("Лапа поднята/опущена");
}

void givePaw(const int leg[3]) {
  moveJointTo(leg[1], 90 + stepLength * 2, stepDelay);
  moveJointTo(leg[2], 90 - stepHeight, stepDelay);
  delay(1000);
  moveJointTo(leg[2], 90, stepDelay);
  moveJointTo(leg[1], 90, stepDelay);
  Serial.println("Дай лапу");
}

/* FIX 4: dog.txt:207 selected the negative direction for raw indices 0 and 6,
 * which under dog.txt's own map are FR and BR - the RIGHT side. Every other
 * function in the same file (liftLeg/lowerLeg, dog.txt:476,488) uses
 * "BL[0] || FL[0]" - the LEFT side. Two opposite side conventions cannot both
 * be right. abductLeg is aligned to the file's dominant convention. */
void abductLeg(const int leg[3]) {
  int dir = (leg[0] == BL[0] || leg[0] == FL[0]) ? -stepSwing * 2 : stepSwing * 2;
  moveJointTo(leg[0], 90 + dir, stepDelay);
  delay(1000);
  moveJointTo(leg[0], 90, stepDelay);
  Serial.println("Отведение лапы в сторону");
}

void bendExtendKnee(const int leg[3]) {
  moveJointTo(leg[2], 120, stepDelay);
  delay(500);
  moveJointTo(leg[2], 60, stepDelay);
  delay(500);
  moveJointTo(leg[2], 90, stepDelay);
  Serial.println("Сгиб/разгиб колена");
}

/* ------------------------ 3. Перемещение корпуса ------------------------- */

void tiltBodyForward() {
  moveJointTo(FL[1], 90 - cogShift * 2, stepDelay / 2);
  moveJointTo(FR[1], 90 - cogShift * 2, stepDelay / 2);
  moveJointTo(BL[1], 90 + cogShift * 2, stepDelay / 2);
  moveJointTo(BR[1], 90 + cogShift * 2, stepDelay / 2);
  Serial.println("Наклон вперёд");
}

void tiltBodyBackward() {
  moveJointTo(FL[1], 90 + cogShift * 2, stepDelay / 2);
  moveJointTo(FR[1], 90 + cogShift * 2, stepDelay / 2);
  moveJointTo(BL[1], 90 - cogShift * 2, stepDelay / 2);
  moveJointTo(BR[1], 90 - cogShift * 2, stepDelay / 2);
  Serial.println("Наклон назад");
}

void tiltBodyLeft() {
  moveJointTo(FL[0], 90 + cogShift * 2, stepDelay / 2);
  moveJointTo(BL[0], 90 + cogShift * 2, stepDelay / 2);
  moveJointTo(FR[0], 90 - cogShift * 2, stepDelay / 2);
  moveJointTo(BR[0], 90 - cogShift * 2, stepDelay / 2);
  Serial.println("Наклон влево");
}

void tiltBodyRight() {
  moveJointTo(FL[0], 90 - cogShift * 2, stepDelay / 2);
  moveJointTo(BL[0], 90 - cogShift * 2, stepDelay / 2);
  moveJointTo(FR[0], 90 + cogShift * 2, stepDelay / 2);
  moveJointTo(BR[0], 90 + cogShift * 2, stepDelay / 2);
  Serial.println("Наклон вправо");
}

void lowerBody() {
  int knees[4] = {2, 5, 8, 11};
  for (int i = 0; i < 4; i++) {
    moveJointTo(knees[i], 120, stepDelay);
  }
  Serial.println("Опускание корпуса");
}

void raiseBody() {
  int knees[4] = {2, 5, 8, 11};
  for (int i = 0; i < 4; i++) {
    moveJointTo(knees[i], 60, stepDelay);
  }
  Serial.println("Подъём корпуса");
}

/* --------------------------- leg primitives ------------------------------ */

void liftLeg(int sideIdx, int fwdIdx, int kneeIdx) {
  int swingDir = (sideIdx == BL[0] || sideIdx == FL[0]) ? -stepSwing : stepSwing;
  moveJointTo(sideIdx, 90 + swingDir, stepDelay / 2);
  moveJointTo(kneeIdx, 90 + stepHeight, stepDelay);
}

void moveLegForward(int fwdIdx, int length) {
  int adjustedLength = (fwdIdx == BL[1] || fwdIdx == FL[1]) ? -length : length;
  moveJointTo(fwdIdx, 90 + adjustedLength, stepDelay);
}

void lowerLeg(int sideIdx, int fwdIdx, int kneeIdx) {
  moveJointTo(kneeIdx, 90, stepDelay);
  moveJointTo(sideIdx, 90, stepDelay / 2);
}

/* FIX 6: every source left the fwd joints parked off-neutral at the end of a
 * gait cycle (dog.txt:490 "Возврат fwd если нужно, но в шаге отдельно").
 * v2.ino:189-190 and :201-202 did perform this return for trot, so the return
 * is restored, not invented. */
void returnFwdToNeutral(const int leg[3]) {
  moveJointTo(leg[1], 90, stepDelay);
}

/* ----------------------- CoG - restored from v2.ino ----------------------
 * Ported from code/dogg v.2.ino:109-143 (shiftCoGForLeg) and :146-155
 * (resetCoG). Both were reduced to an empty stub in v2.1 and dog.txt.
 *
 * The port is mechanical: v2.ino addressed raw indices under the v2 map
 * (BL={0,1,2}, BR={3,4,5}, FL={6,7,8}, FR={9,10,11}), so every literal index
 * was translated back to its symbolic joint and is emitted symbolically here.
 * The shift magnitudes and signs are unchanged.
 *
 * resetCoG() replaces v2.ino's getCurrentAngle() stub (which always returned
 * 90) with the tracked commanded angle - this is FIX 5 applied to the port.
 * ------------------------------------------------------------------------*/
void shiftCoGForLeg(const int leg[3], int shift) {
  if (leg == FR) {
    // Смещение назад-влево для опоры на BL, BR, FL
    moveJointTo(BL[1], 90 - shift, stepDelay / 4);
    moveJointTo(BR[1], 90 - shift, stepDelay / 4);
    moveJointTo(FL[1], 90 + shift, stepDelay / 4);
    moveJointTo(BL[0], 90 + shift, stepDelay / 4);
    moveJointTo(BR[0], 90 - shift, stepDelay / 4);
    moveJointTo(FL[0], 90 + shift, stepDelay / 4);
  } else if (leg == FL) {
    // Смещение назад-вправо для опоры на BL, BR, FR
    moveJointTo(BL[1], 90 - shift, stepDelay / 4);
    moveJointTo(BR[1], 90 - shift, stepDelay / 4);
    moveJointTo(FR[1], 90 - shift, stepDelay / 4);
    moveJointTo(BL[0], 90 - shift, stepDelay / 4);
    moveJointTo(BR[0], 90 + shift, stepDelay / 4);
    moveJointTo(FR[0], 90 - shift, stepDelay / 4);
  } else if (leg == BR) {
    // Смещение вперед-влево для опоры на BL, FL, FR
    moveJointTo(BL[1], 90 + shift, stepDelay / 4);
    moveJointTo(FL[1], 90 - shift, stepDelay / 4);
    moveJointTo(FR[1], 90 + shift, stepDelay / 4);
    moveJointTo(BL[0], 90 + shift, stepDelay / 4);
    moveJointTo(FL[0], 90 + shift, stepDelay / 4);
    moveJointTo(FR[0], 90 + shift, stepDelay / 4);
  } else if (leg == BL) {
    // Смещение вперед-вправо для опоры на BR, FL, FR
    moveJointTo(BR[1], 90 + shift, stepDelay / 4);
    moveJointTo(FL[1], 90 - shift, stepDelay / 4);
    moveJointTo(FR[1], 90 + shift, stepDelay / 4);
    moveJointTo(BR[0], 90 + shift, stepDelay / 4);
    moveJointTo(FL[0], 90 - shift, stepDelay / 4);
    moveJointTo(FR[0], 90 - shift, stepDelay / 4);
  }
}

void resetCoG() {
  const int* legs[4] = {FR, FL, BR, BL};
  for (int l = 0; l < 4; l++) {
    moveJointTo(legs[l][0], 90, stepDelay / 4);
    moveJointTo(legs[l][1], 90, stepDelay / 4);
  }
}

/* --------------------------- 4. Ходьба и шаги ---------------------------- */

// Trot: диагональные лапы (FR+BL, FL+BR). Sequence from dog.txt:278-296;
// the per-pair fwd return is restored from v2.ino:189-190,201-202 (FIX 6).
void forwardTrotStep() {
  liftLeg(FR[0], FR[1], FR[2]);
  liftLeg(BL[0], BL[1], BL[2]);
  moveLegForward(FR[1], stepLength);
  moveLegForward(BL[1], stepLength);
  lowerLeg(FR[0], FR[1], FR[2]);
  lowerLeg(BL[0], BL[1], BL[2]);
  returnFwdToNeutral(FR);
  returnFwdToNeutral(BL);
  delay(stepDelay / 2);

  liftLeg(FL[0], FL[1], FL[2]);
  liftLeg(BR[0], BR[1], BR[2]);
  moveLegForward(FL[1], stepLength);
  moveLegForward(BR[1], stepLength);
  lowerLeg(FL[0], FL[1], FL[2]);
  lowerLeg(BR[0], BR[1], BR[2]);
  returnFwdToNeutral(FL);
  returnFwdToNeutral(BR);
  delay(stepDelay / 2);

  Serial.println("Trot шаг завершён.");
}

// Creep (кроул): по одной лапе. Leg order FR, FL, BL, BR is identical in
// dog.txt:300 and v2.ino:211-241. The shiftCoG/resetCoG bracketing around each
// leg is restored from v2.ino's creep; resetCoG also performs the fwd return.
void forwardCreepStep() {
  const int* order[4] = {FR, FL, BL, BR};
  for (int i = 0; i < 4; i++) {
    shiftCoGForLeg(order[i], cogShift);
    liftLeg(order[i][0], order[i][1], order[i][2]);
    moveLegForward(order[i][1], stepLength);
    lowerLeg(order[i][0], order[i][1], order[i][2]);
    resetCoG();
    delay(stepDelay / 2);
  }
  Serial.println("Creep шаг завершён.");
}

// Pace: боковые лапы (FL+BL, FR+BR). dog.txt:311-329 + fwd return (FIX 6).
void forwardPaceStep() {
  liftLeg(FL[0], FL[1], FL[2]);
  liftLeg(BL[0], BL[1], BL[2]);
  moveLegForward(FL[1], stepLength);
  moveLegForward(BL[1], stepLength);
  lowerLeg(FL[0], FL[1], FL[2]);
  lowerLeg(BL[0], BL[1], BL[2]);
  returnFwdToNeutral(FL);
  returnFwdToNeutral(BL);
  delay(stepDelay / 2);

  liftLeg(FR[0], FR[1], FR[2]);
  liftLeg(BR[0], BR[1], BR[2]);
  moveLegForward(FR[1], stepLength);
  moveLegForward(BR[1], stepLength);
  lowerLeg(FR[0], FR[1], FR[2]);
  lowerLeg(BR[0], BR[1], BR[2]);
  returnFwdToNeutral(FR);
  returnFwdToNeutral(BR);
  delay(stepDelay / 2);

  Serial.println("Pace шаг завершён.");
}

/* Gallop. Sequence from dog.txt:332-369.
 * FIX 7: dog.txt:341,343,354,356 computed the return offsets with conditions
 * that are constant by construction - "FL[1] == FL[1]" is always true and
 * "BR[1] == BL[1] || BR[1] == FL[1]" is always false, comparing distinct
 * compile-time constants. Two of the four legs were therefore ramped from the
 * wrong start angle. The intent stated in the comment ("Возврат fwd ... в
 * нейтраль") is now expressed directly as a return to 90 from the tracked
 * angle, which removes the dead conditions entirely. */
void forwardGallopStep() {
  liftLeg(FL[0], FL[1], FL[2]);
  liftLeg(FR[0], FR[1], FR[2]);
  moveLegForward(FL[1], stepLength * 2);
  moveLegForward(FR[1], stepLength * 2);
  lowerLeg(FL[0], FL[1], FL[2]);
  lowerLeg(FR[0], FR[1], FR[2]);

  // Возврат fwd передних лап в нейтраль
  returnFwdToNeutral(FL);
  returnFwdToNeutral(FR);

  liftLeg(BL[0], BL[1], BL[2]);
  liftLeg(BR[0], BR[1], BR[2]);
  moveLegForward(BL[1], stepLength * 2);
  moveLegForward(BR[1], stepLength * 2);
  lowerLeg(BL[0], BL[1], BL[2]);
  lowerLeg(BR[0], BR[1], BR[2]);

  // Возврат fwd задних лап в нейтраль
  returnFwdToNeutral(BL);
  returnFwdToNeutral(BR);

  // Фаза прыжка: поднять все колени слегка
  int knees[4] = {2, 5, 8, 11};
  for (int i = 0; i < 4; i++) {
    moveJointTo(knees[i], 70, stepDelay / 2);
  }
  delay(300);
  for (int i = 0; i < 4; i++) {
    moveJointTo(knees[i], 90, stepDelay / 2);
  }
  Serial.println("Gallop шаг завершён.");
}

/* ------------------------------ 5. Повороты ------------------------------ */

/* dog.txt:373-382. FIX 5/6: the trailing setNeutralStance() snapped all twelve
 * joints instantaneously from wherever they were. Only the four fwd joints are
 * disturbed by this function, so only those are returned, and smoothly. */
void turnInPlace() {
  moveJointTo(FR[1], 90 + stepLength, stepDelay);
  moveJointTo(BR[1], 90 + stepLength, stepDelay);
  moveJointTo(FL[1], 90 - stepLength, stepDelay);
  moveJointTo(BL[1], 90 - stepLength, stepDelay);
  delay(500);
  moveJointTo(FR[1], 90, stepDelay);
  moveJointTo(BR[1], 90, stepDelay);
  moveJointTo(FL[1], 90, stepDelay);
  moveJointTo(BL[1], 90, stepDelay);
  Serial.println("Поворот на месте");
}

// dog.txt:385-418, sequence order unchanged. The trailing side-joint return
// keeps its position in the sequence but now ramps from the tracked angle.
void smoothTurnDuringMovement() {
  liftLeg(FR[0], FR[1], FR[2]);
  liftLeg(BL[0], BL[1], BL[2]);
  moveJointTo(FR[0], 90 + stepSwing, stepDelay / 2);
  moveJointTo(BL[0], 90 - stepSwing, stepDelay / 2);
  moveLegForward(FR[1], stepLength);
  moveLegForward(BL[1], stepLength);
  lowerLeg(FR[0], FR[1], FR[2]);
  lowerLeg(BL[0], BL[1], BL[2]);
  returnFwdToNeutral(FR);
  returnFwdToNeutral(BL);
  delay(stepDelay / 2);

  liftLeg(FL[0], FL[1], FL[2]);
  liftLeg(BR[0], BR[1], BR[2]);
  moveJointTo(FL[0], 90 - stepSwing, stepDelay / 2);
  moveJointTo(BR[0], 90 + stepSwing, stepDelay / 2);
  moveLegForward(FL[1], stepLength);
  moveLegForward(BR[1], stepLength);
  lowerLeg(FL[0], FL[1], FL[2]);
  lowerLeg(BR[0], BR[1], BR[2]);
  returnFwdToNeutral(FL);
  returnFwdToNeutral(BR);
  delay(stepDelay / 2);

  // Возврат всех side-сервов в нейтраль после поворота
  moveJointTo(FR[0], 90, stepDelay / 2);
  moveJointTo(FL[0], 90, stepDelay / 2);
  moveJointTo(BR[0], 90, stepDelay / 2);
  moveJointTo(BL[0], 90, stepDelay / 2);

  Serial.println("Плавный поворот во время движения");
}

/* ----------------------------- 6. Доп. трюки ----------------------------- */

void jumpTrick() {
  raiseBody();
  delay(200);
  lowerBody();
  Serial.println("Прыжок");
}

void danceTrick() {
  for (int i = 0; i < 3; i++) {
    liftLeg(FL[0], FL[1], FL[2]);
    liftLeg(FR[0], FR[1], FR[2]);
    lowerLeg(FL[0], FL[1], FL[2]);
    lowerLeg(FR[0], FR[1], FR[2]);
    liftLeg(BL[0], BL[1], BL[2]);
    liftLeg(BR[0], BR[1], BR[2]);
    lowerLeg(BL[0], BL[1], BL[2]);
    lowerLeg(BR[0], BR[1], BR[2]);
  }
  Serial.println("Танец");
}

void bowTrick() {
  moveJointTo(FL[1], 90 - stepLength * 2, stepDelay);
  moveJointTo(FR[1], 90 - stepLength * 2, stepDelay);
  lowerBody();
  delay(1000);
  raiseBody();
  moveJointTo(FL[1], 90, stepDelay);
  moveJointTo(FR[1], 90, stepDelay);
  Serial.println("Поклон");
}

/* dog.txt:457-466. FIX 5/6: the trailing setNeutralStance() snap is replaced by
 * a smooth return of the four side joints that tiltBodyLeft/Right disturb. */
void shakeBodyTrick() {
  for (int i = 0; i < 3; i++) {
    tiltBodyLeft();
    delay(300);
    tiltBodyRight();
    delay(300);
  }
  moveJointTo(FL[0], 90, stepDelay / 2);
  moveJointTo(BL[0], 90, stepDelay / 2);
  moveJointTo(FR[0], 90, stepDelay / 2);
  moveJointTo(BR[0], 90, stepDelay / 2);
  Serial.println("Качание корпусом");
}
