# Sparky firmware

![sparky gif](animated.gif)

Arduino sketch + associated source files for running Sparky, the [RHex](https://en.wikipedia.org/wiki/Rhex)-style hexapedal robot we designed for UC Berkeley's Introduction to Prototyping & Fabrication course.

Our code's made up of a few main components:
- an open-loop gait controller for generating synchronized leg trajectories
- six local joint controllers: these are simple PD loops w/ velocity feedforwards
- a BLE interface for sending status information & receiving twist commands
- a set of dynamically configurable settings which can be pushed to & pulled from the EEPROM (this is primarily for control loop tuning on the fly)

---

[Demo video](https://www.youtube.com/watch?v=aiBIEI0JHwY) and relevant repositories:
- [mechanical design](https://github.com/nanditapiyer/sparky_mechanical)
- [main control PCB](https://github.com/brentyi/sparky_electronics)
- [absolute encoder PCB](https://github.com/brentyi/as5048b_breakout)

---

Potential features for the future:
- gait optimization based on accelerometer data (already supported by hardware --  for inclines, flip scenarios, etc)
- smarter leg control implementation
- quantitatively calculated leg trajectory parameters (speeds are currently preset)
- ROS support

---

Made possible by:
- Adafruit's nRF51-based [BLE modules](https://github.com/adafruit/Adafruit_BluefruitLE_nRF51)
- Adafruit's [library](https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library) for interfacing with the PCA9685 PWM driver
- br3ttb's [PID library](https://github.com/br3ttb/Arduino-PID-Library)
- sosandroid's [AS5048B library](https://github.com/sosandroid/AMS_AS5048B) (for programming slave addresses)
