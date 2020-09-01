# Tiva Launchpad Utilities
This is a collection of modules that can be used with the Tiva Launchpad (specifically, the [TM4C123GH6PM](https://www.ti.com/lit/ds/spms376e/spms376e.pdf?ts=1598921447800&ref_url=https%253A%252F%252Fwww.google.com%252F) model) microcontroller and whose functionality is intended for mechatronics based applications, such as modules for streamlining PWM pin usage and interfaces for motor control.

## Getting Started
It is assumed that development is occurring within an IDE that specifically supports Tiva microcontrollers, e.g., [Keil's uVision](https://www.ti.com/lit/ml/spmu355/spmu355.pdf?ts=1598921718271&ref_url=https%253A%252F%252Fwww.google.com%252F) and associated Tiva-specific header libraries ([TivaWare](https://www.ti.com/tool/SW-TM4C)).

Additionally, the header file [tm4c123gh6pm.h](include/tm4c123gh6pm.h) includes model-specific macro defines and must be included within the header search path.

As some of these modules leverage interrupts / ISR, their corresponding callback functions have been registered in [startup_rvmdk.S](startup/startup_rvmdk.S) which is included in this repo and must be copied into the appropriate location in your project directory tree.

Otherwise, all library modules are self-contained, and should not require any additional external dependencies. All modules' top-level functions are documented in their corresponding source files, and can be considered as the public API for that given module.

## Modules

### Actuators
This section includes modules that are intended to be used with motor drivers or similar actuators. One module is included in this section -- [MotorWheels](actuators/MotorWheels.h), which includes functionality for controlling a two-wheeled holonomic robot that is equipped with encoder-enabled DC motors to drive the wheels. The robot can be driven in either an open-loop or closed-loop fashion, with the latter being controlled via PID.

### Peripherals
This section includes modules that directly interface with the TIVA hardware. One module is included in this section -- [PWMFunctions](peripherals/PWMFunctions.h), which provides a streamlined API for easily toggling and configuring an arbitrary number (up to 16) of PWM pins on the TIVA board. Specific PWM channels / groups can easily have their period / duty cycle configured at runtime using this API.

### Sensors
This section includes modules that interface directly with specific sensors. Two modules are included in this section -- [ADXL343Lib](sensors/ADXL343Lib.h), a cheap and common 3-axis accelerometer, and [LSM9DS1Lib](sensors/LSM9DS1Lib.h), a popular 9-axis IMU. Note that, unfortunately, I was never able to get the Tiva to correctly read from the 9-axis IMU, so that module is fairly underdeveloped and not fully tested ):

### Utilities
This section includes modules that have general widespread applicability and are intended to be utilized in many places. Two modules are included in this section -- [PIDControllers](util/PIDControllers.h), which provides an interface for initializing up to 4 independent PID controllers with tunable parameters (with automatically linked function callbacks for grabbing new data / executing the control signal), and [UtilityFunctions](util/UtilityFunctions.h), which contains general-purpose functionality such as a RingBuffer implementation and hardware debugging line.

## Contributing to this Repo
If there are any errors / bugs / improvements you notice, please don't hesitate to open and issue and/or create a pull request!