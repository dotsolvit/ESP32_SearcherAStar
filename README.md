[Читати мене Українською](README_uk.md)

# Educational project «Smart car that detects obstacles and plans a route using the A* algorithm» on ESP32

![Real Photo](/images/Searcher_photo2.jpg)

## Short description
This is an educational project that implements autonomous control of a model car on an ESP32. It uses a Sharp IR distance sensor (20–150 cm) mounted on an MG90S servo for obstacle detection, two optical encoders with slotted disks for odometry, an MPU-6050 accelerometer/gyroscope for heading, an OLED display, buzzer, motors, and other peripherals. Path planning is performed using the A* algorithm; obstacles detected during motion are added to the obstacle map and routes are recalculated accordingly.

## Features
- **Start/Goal:** Set start and goal coordinates to begin navigation.
- **A-star path planning:** Computes shortest route (minimizing distance and turns) using the known obstacle map at calculation time.
- **Two-stage motion:** Each move executes a rotation (multiple of 45°) followed by straight movement.
- **On-the-fly obstacle detection:** IR sensor scans ±6° during straight motion; on detecting an unknown obstacle the vehicle stops and performs a wide-angle scan.
- **Wide-angle scanning:** Stationary scan at ±90° for accurate mapping of new obstacles (map grid step = 10 cm).
- **Dynamic replanning:** New obstacles are added to the obstacle map and A* recalculates the route; motion resumes if a route exists.
- **Termination:** Movement ends upon reaching the goal or when no feasible route can be built.

## Model Control
To control the model, the microcontroller is connected to a WiFi network and a web server provides an appropriate interface that allows you to: set the coordinates of the target point, start movement, view the obstacle map and movement route, and obtain information on movement diagnostics and resource usage.

Example of an obstacle map and route: ![Example](/images/ExampleOfAnObstacleMapAndRoute.jpg)

where “X” is an obstacle, “A” and “C” are the start and end of the movement, “B” are the turning points, “+” are the intermediate movement points.

## Components and wiring
- **Microcontroller:** ESP32 LuaNode32 Type-C (38-pin)
- **Motors:** Four geared DC motors (1:48, 6–8V) with wheels
- **IR distance sensor:** Sharp GP2Y0A02YK0F (20–150 cm)
- **Servo:** MG90S (for aiming the IR sensor)
- **Encoders:** Two optical encoders + slotted disks (odometry)
- **IMU:** MPU-6050 (accelerometer + gyroscope)
- **Display:** 0.96" I2C OLED 128×64
- **Buzzer:** passive buzzer
- **Power:** two 18650 Li-ion cells, 2S BMS with balancing, LM2596 DC-DC step-down, and 2S Li-ion charger module (Type-C)

## Schematic
Electrical diagram and connections: ![Schematic](/images/Schematic_ESP32_SearcherAStar.png)

## Code and build
The project is developed with PlatformIO (Visual Studio Code + PlatformIO) using the Arduino framework. The code is written in C/C++ and uses FreeRTOS to distribute tasks across the two ESP32 cores (WiFi/web server on one core, other tasks on the other core). The A* implementation follows "Pathfinding Demystified. Generic Search Algorithm. Practical A*" (https://www.gabrielgambetta.com/generic-search.html).

- **Sources:** See the `src` folder for the project code.

### Build & upload (PlatformIO)
Open the project in VS Code with PlatformIO and build/upload using the PlatformIO UI, or use the CLI:

```bash
platformio run -t upload
```

## External libraries
- `electroniccats/MPU6050@^1.4.4`
- `adafruit/Adafruit SSD1306@^2.5.16`
- `adafruit/Adafruit GFX Library@^1.12.6`

## Notes
- Map grid step is 10 cm (see `config.hpp`).
- Interrupt-based odometry and MPU-6050 heading are used to control motion and turns.

## Conclusion
This project demonstrates ESP32 programming, peripheral control, creating an embedded web server, and remote control via WiFi from a smartphone or computer.

Good luck and enjoy experimenting!

P.S. The project was developed as a coursework while studying ESP32, Ground Drone, Unmanned Ground Vehicle and GITHUB courses at Nanit Academy: https://www.linkedin.com/company/nanitrobot/
