# OpenQuadruped — Project Ground Truth

## Status

This repository is being reconstructed into a working OpenQuadruped
quadruped firmware.

Do not infer hardware architecture from historical documentation.
The hardware facts below are authoritative unless explicitly changed.

Companion documents (all in repository root):
README.md · PROJECT_STATUS.md · ARCHITECTURE.md · HARDWARE.md ·
CALIBRATION.md · SAFETY.md

## CURRENT HARDWARE — AUTHORITATIVE

MCU:

- ESP32-S3
- One MCU only

Servo controller:

- PCA9685
- I2C address: UNKNOWN — physical verification required
- I2C
- 12 servo channels

Servos:

- QY3240MG HV
- 12 servos
- 4 legs × 3 joints:
  - side
  - forward/back
  - knee

Mechanical platform:

- OpenQuadruped

Servo power:

- Separate 2S LiPo 50C
- Servo power is electrically separate from MCU power
- Common ground between MCU and servo/PCA9685 system must be verified physically

## ARCHITECTURE

ESP32-S3
    |
    | I2C
    v
PCA9685
    |
    +-- 12 PWM channels
          |
          +-- 12 × QY3240MG HV

USB/Serial
    |
    v
ESP32-S3 command interface

There is NO:

- second MCU
- Raspberry Pi
- ROS
- Teensy
- Arduino Nano
- FEETECH serial bus
- IMU
- OLED
- PS2 receiver
- ultrasonic sensors
- PIR
- battery telemetry
- host robotics stack

unless explicitly added later as a documented architectural decision.

## MECHANICAL PLATFORM

The target robot is OpenQuadruped.

NovaSM3 / SpotMicro / KDY0523 documentation is historical/reference material only
and must not be used to infer the current mechanical architecture.

## CURRENT PROJECT GOAL

Make the existing firmware architecture operational on this actual hardware.

- Historical code is reference material only.
- Historical code must never override confirmed current hardware facts.
- Unknown parameters must remain explicitly UNKNOWN.
- Do not infer physical facts from source code when they require hardware
  verification.

## CONFIRMED

- ESP32-S3
- one MCU
- PCA9685
- 12 servos
- QY3240MG HV
- OpenQuadruped
- separate 2S LiPo 50C servo supply

Source of truth: operator statement, 2026-08-20. Not derived from source code.

## UNKNOWN

- physical channel mapping
- joint directions
- inversion
- calibration
- I2C pins/address
- exact power wiring
- safe servo limits

## MUST NOT BE TREATED AS ESTABLISHED BEFORE PHYSICAL VERIFICATION

The following are UNKNOWN regardless of what any source file contains:

- PCA9685 I2C address
- ESP32-S3 SDA/SCL pins
- exact PCA9685 channel → physical joint mapping
- joint inversion/direction
- servo center PWM
- servo safe min/max
- actual servo supply voltage at the driver
- actual electrical grounding/power topology

A value appearing in historical firmware is a CANDIDATE, never a measurement.

## FIRMWARE GOAL

The goal is not to preserve historical firmware behavior blindly.

The goal is:

1. establish the real ESP32-S3 + PCA9685 + QY3240MG hardware mapping;
2. establish safe servo calibration;
3. establish correct leg/joint orientation;
4. establish reliable neutral pose;
5. establish individual joint movement;
6. establish individual leg movement;
7. establish stable basic gaits;
8. only then add higher-level behavior.

## SAFETY

Never introduce startup servo movement automatically.

Never invent:

- servo limits
- calibration values
- channel mapping
- inversion flags
- power parameters

If hardware evidence is missing, mark it UNKNOWN and create a verification procedure.

Servo power is external to the MCU.

Hardware testing must initially be performed with the robot mechanically unloaded
and with a current-limited supply.

Full bring-up rules: see SAFETY.md.

## HISTORICAL FILES

The following files are historical evidence, not authoritative architecture:

- code/dogg v.2.ino
- code/dogg v.2.1 copy.ino
- code/dog.txt
- README_OLD.md
- docs/*
- external NovaSM3 documentation

They may be mined for algorithms, calibration candidates and intent,
but they must not override CURRENT HARDWARE.

Historical files must not be deleted, edited, renamed or moved without explicit
operator permission. Their per-file status is recorded in README.md.

## AMBIGUOUS FILE — NAMING

`code/nova_sm3/nova_sm3.ino` is a consolidation of the three historical sketches.
Its filename references NovaSM3, which is NOT the current mechanical platform.
The name is misleading and a rename is recommended, but the file has NOT been
renamed pending operator permission. Its content targets a generic Arduino core
and has NOT been ported to ESP32-S3.

## CHANGE POLICY

Before changing hardware-facing behavior:

1. identify the source of the current value;
2. distinguish measured fact from inherited value;
3. preserve unknowns explicitly;
4. explain why the change is required;
5. update project documentation if architecture changes.

Do not silently "fix" calibration or servo mapping by assumption.

## IMPORTANT: HOW TO HANDLE HISTORICAL CODE

Do not assume that the newest or most complete historical firmware is correct.

Historical firmware was written for an incompletely identified hardware setup.

Use historical code only to recover:

- algorithms
- intended behavior
- command semantics
- candidate calibration values
- candidate leg mappings

Current hardware facts always take precedence.

If historical code conflicts with confirmed hardware:
DO NOT preserve the historical behavior merely for compatibility.

If a value cannot be established from hardware evidence:
mark it UNKNOWN and create a verification step.
