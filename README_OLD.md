# Robotics Lab — Spotty (Квадроногий робот-пёс)

## Описание

Четырёхногий робот-пёс на базе проекта **SpotMicro** (mike4192).  
Ключевое отличие от оригинала: вместо PWM-серво через PCA9685 используются **FEETECH STS3020** — серийные шинные сервоприводы (TTL Serial Bus) с обратной связью по положению, нагрузке и температуре.

**Первоисточник:** https://github.com/mike4192/spotMicro  
**Корпус (STL):** https://www.thingiverse.com/thing:3445283 (KDY0523)

---

## Отличия нашей версии от оригинала SpotMicro

| Параметр | SpotMicro (оригинал) | Spotty (наш) |
|----------|----------------------|--------------|
| Сервоприводы | 12× PDI-HV5523MG (PWM) | 12× FEETECH STS3020 (Serial Bus TTL) |
| Серво-драйвер | PCA9685 (I2C → PWM) | Waveshare Bus Servo Adapter (UART → TTL) |
| Обратная связь серво | Нет | Позиция, нагрузка, температура |
| IMU | Опционально | BNO055 (основной) + MPU6050 |
| Датчики ног | Нет | FSR 402 × 4 + ADS1115 |
| Питание серво | 2S LiPo 7.4V | 3S LiPo 11.1V → Buck 8V |
| АКБ ёмкость | 4000 mAh | 4200 mAh (35C) + 1500 mAh (100C) |
| ПО | ROS Kinetic | ROS Noetic / Python (адаптация) |

---

## Цели проекта
- [ ] Печать и сборка корпуса SpotMicro (STL)
- [ ] Монтаж сервоприводов STS3020, назначение ID (1–12)
- [ ] Калибровка нулевых положений каждого серво
- [ ] Настройка Waveshare Bus Servo Adapter
- [ ] Установка ROS Noetic на Raspberry Pi
- [ ] Адаптация кода SpotMicro под STS3020 (замена PCA9685 → Serial Bus)
- [ ] Реализация гейтов ходьбы (trot, 8-phase)
- [ ] Интеграция IMU BNO055 для балансировки
- [ ] Интеграция FSR 402 для определения контакта ног

---

## Аппаратная часть

| Компонент | Модель | Кол-во | Назначение |
|-----------|--------|--------|------------|
| **Управление** |
| Компьютер | Raspberry Pi 3B / 4 | 1 | ROS, кинематика |
| Серво-адаптер | Waveshare Bus Servo Adapter (A) | 2 | UART → TTL шина STS3020 |
| **Сервоприводы** |
| Серво шинные | FEETECH STS3020 (8.4В, 24.3 кг·см) | 20 | 12 рабочих + 8 запас |
| **Сенсоры** |
| IMU основной | BNO055 (GY-BNO055) | 2 | Абсолютная ориентация тела |
| IMU доп. | MPU6050 | 10 | Акселерометр/гироскоп (запас) |
| Датчики давления | FSR 402 | 4 | Контакт ступни с землёй |
| АЦП | ADS1115 (I2C) | 2 | Чтение FSR (16-бит) |
| **Питание** |
| АКБ основной | LiPo 11.1V 4200mAh 35C XT60 | 3 | Питание серво |
| АКБ резервный | LiPo 11.1V 1500mAh 100C XT60 | 3 | Пиковые токи |
| Buck DC-DC | XL4016 8V/8A | 3 | 11.1V → 8V для STS3020 |
| UBEC | HKU5 5V/5A | 10 | 8V → 5V для RPi и логики |
| Зарядник | SkyRC D200neo BD350 | 1 | Балансировочная зарядка LiPo |

---

## Архитектура серво-шины

FEETECH STS3020 работают по Half-Duplex UART (не PWM). Все 12 серво на одной шине, каждому назначен уникальный ID.

```
Raspberry Pi (UART)
        │
Waveshare Bus Servo Adapter
        │
  [ID:1]──[ID:2]──[ID:3]──[ID:4]──[ID:5]──[ID:6]
  FL_hip  FL_thi  FL_kne  FR_hip  FR_thi  FR_kne
        │
  [ID:7]──[ID:8]──[ID:9]──[ID:10]─[ID:11]─[ID:12]
  BL_hip  BL_thi  BL_kne  BR_hip  BR_thi  BR_kne
```

**Карта ID:**

| ID | Нога | Сустав | ID | Нога | Сустав |
|----|------|--------|----|------|--------|
| 1 | FL (перед лево) | Hip | 7 | BL (зад лево) | Hip |
| 2 | FL | Thigh | 8 | BL | Thigh |
| 3 | FL | Knee | 9 | BL | Knee |
| 4 | FR (перед право) | Hip | 10 | BR (зад право) | Hip |
| 5 | FR | Thigh | 11 | BR | Thigh |
| 6 | FR | Knee | 12 | BR | Knee |

---

## Схема питания

```
LiPo 11.1V 4200mAh (XT60)
      │
      ├── XL4016 Buck → 8V/8A ──→ FEETECH STS3020 × 12
      │
      └── UBEC HKU5  → 5V/5A ──→ Raspberry Pi + BNO055 + ADS1115
```

---

## Программная часть

**ОС:** Ubuntu 20.04 (RPi) + ROS Noetic  
**База:** SpotMicro (mike4192) — адаптация под STS3020

**Ключевые пакеты ROS (из оригинала):**
```
spot_micro_motion_cmd   — управление движением, state machine
spot_micro_joy          — джойстик → команды
spot_micro_rviz         — визуализация URDF в RViz
spot_micro_kinematics   — прямая/обратная кинематика (C++)
```

**Адаптация под STS3020** (вместо PCA9685):
```python
# Оригинал (PCA9685):
# pca.channels[i].duty_cycle = angle_to_pwm(angle)

# Наш вариант (STS3020 Serial Bus):
from feetech import SMS_STS
servo = SMS_STS()
servo.begin(115200, '/dev/ttyUSB0')
servo.WritePosEx(servo_id, position, speed=1000, acc=50)
```

**Установка ROS Noetic на RPi:**
```bash
# Ubuntu 20.04 для RPi
sudo apt install ros-noetic-desktop
sudo apt install ros-noetic-joy ros-noetic-hector-slam

# Сборка SpotMicro
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/mike4192/spotMicro.git
cd ~/catkin_ws
catkin_make   # нужен SWAP 1GB на RPi!
```

**Запуск:**
```bash
# Назначение ID серво (первый запуск)
python set_id.py --port /dev/ttyUSB0 --old 1 --new 3

# Основное управление
roslaunch spot_micro_motion_cmd spot_micro_motion_cmd.launch

# Джойстик
roslaunch spot_micro_joy spot_micro_joy.launch
```

---

## Гейты ходьбы (из SpotMicro)

| Гейт | Описание | Команда |
|------|----------|---------|
| Stand | Стоять | joy кнопка |
| Sit | Сидеть | joy кнопка |
| Trot | Рысь (диагонали) | joy стик |
| 8-phase walk | Медленный шаг | joy стик |
| Angle cmd | Наклон тела | joy стик |

---

## Структура проекта
```
Spotty/
├── code/
│   ├── dogg v.2.ino           # архив: старая PWM-версия
│   ├── dogg v.2.1 copy.ino    # архив: версия с трюками
│   └── spotty_ros/            # ROS-пакет (адаптация SpotMicro)
│       ├── servo_interface.py # STS3020 вместо PCA9685
│       └── launch/
├── docs/
│   ├── 1.md – 4.md            # ROS документация
│   └── servo_id_map.md        # карта ID серво
├── hardware/                  # STL-файлы корпуса SpotMicro
└── README.md
```

---

## Дополнительно
- SpotMicro: https://github.com/mike4192/spotMicro
- Корпус STL: https://www.thingiverse.com/thing:3445283
- FEETECH SDK: https://github.com/ftservo/SCServo
- Waveshare Bus Servo Adapter: https://www.waveshare.com/bus-servo-adapter-a.htm
- Улучшения: RPLidar A1 для SLAM, камера + OpenCV, интеграция с ROY
