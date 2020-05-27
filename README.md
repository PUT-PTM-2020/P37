<p align="center">
  <a>
    <img src="logo.png">
  </a>
</p>             

<!-- TABLE OF CONTENTS -->
## Table of Contents

* [About the Project](#about-the-project)
  * [Required Devices](#required-devices)
  * [Built With](#built-with)
* [Getting Started](#getting-started)
  * [Prerequisites](#prerequisites)
  * [Installation](#installation)
  * [Wiring scheme](#wiring-scheme)
  * [Build and flash](#build-and-flash)
* [Authors](#Authors)

<!-- ABOUT THE PROJECT -->
## About The Project
<p align="center">
  <a>
    <img width="50%" height="50%" src="vehicle.jpg">
  </a>
</p>

F10 is a unviersity project where main goal is to use microcontroller STM32 in practical way. We decided to created remote controlled vehicle with 2 driving modes:
* Autonomous - 3 sonic sensors mounted in front of the vehicle are responsible for checking distance to obstacles. Basing on results vehicle is avoiding them, in some cases it can decide than obstacle can not be avoided in simple way and it stops.
* Manual -  is connected to PC app via WiFi. User can steer with vehicle by sending signals to it. Connection is provided by ESP8266 with TCP Protocol.

<p align="center">
  <a>
    <img src="vehicle1.gif">
  </a>
</p>

### Required devices
You need to have all following devices to run project properly:
* STM32F4 Discovery
* 3x Sonic Sensor HC-S04
* ESP8266 WiFi module
* 2x DC Motor 3-6V
* Motor Driver Board L298N
* Power link +7.4 V 

Motors and power link can be changed when these conditions are performed:
* Power link must be at least +7.4V to power STM32
* Motors must be able to work with voltage of power link minus 1.4 V (i.e. 7.4V - 1.4V = 6V)

### Built With
To run project you will need following modules:
* [Zephyr](https://www.zephyrproject.org)
* [CMake](https://cmake.org/)
* [Python](https://www.python.org/)

<!-- GETTING STARTED -->
## Getting Started

This is an example of how you may give instructions on setting up your project locally.
To get a local copy up and running follow these simple example steps.

### Prerequisites

1. Install Python. <br>
	1.1. For Windows download the newest version from [here](https://www.python.org/downloads/). <br>
	1.2. For Linux (Ubuntu)  a add the deadsnakes PPA to your system’s sources list
	```
	$ sudo add-apt-repository ppa:deadsnakes/ppa
	```
	Once the repository is enabled, install Python 3.8 with:
	```
	$ sudo apt install python3.8
	```

### Installation

1. Install Zephyr Poject RTOS by following [this](https://docs.zephyrproject.org/latest/getting_started/index.html) instruction.
2. Clone the repo to any folder
	```
	git clone https://github.com/PUT-PTM-2020/P37
	```
3. Copy zephyrproject from your local zephyr path to previously cloned folder but do not overwrite existing files.

### Wiring scheme

* STM32 <-> ESP8266 <br>

| STM32 Pin | ESP8266 Pin |
|-----------|-------------|
| GND       | GND         |
| PB7       | U0_TX       |
| PB6       | U0_RX       |
| VDD       | CHIP_EN     |
| VDD       | VDD         |

* STM32 <-> Sonic Sensors <br>

| STM32 Pin | Sensor Pin  |
|-----------|-------------|
| GND       | GND         |
| PE0       | ECHO_FRONT  |
| PE1       | TRIG_FRONT  |
| PE2       | ECHO_LEFT   |
| PE3       | TRIG_LEFT   |
| PE4       | ECHO_RIGHT  |
| PE5       | TRIG_RIGHT  |
| VDD +5V   | VDD         |

* STM32 <-> Motor Driver Board

| STM32 Pin | Driver Pin |
|-----------|-------------|
| MiniUSB   | GND, +5V    |
| PA0       | ENA    	  |
| PA1       | ENB  	  |
| PA2       | IN2   	  |
| PA3       | IN3	  |
| PA4       | IN4 	  |
| PA5       | IN1	  |

### Build and flash
1. Connect STM32 to PC via USB.
2. Edit files ```zephyrproject/zephyr/samples/F1TENTH/src/main.c``` in line 463 and ```F1TENTH-python-app/app.py``` in line 189. Change IP address to match your network adapter.
3. Run python app.
4. Run cmd and go to ```/zephyrproject/zephyr``` in project repository.
5. Build project:
```
west build -p auto -b stm32f4_disco samples/f1tenth
```
6. Flash project:
```
west flash
```
Done!

## Authors

Robert Szczepański - [SomeLogic](https://github.com/SomeLogic)

Mateusz Sierszulski - [mts-srs](https://github.com/mts-srs)

Wojciech Rzeczycki - [VoiTee](https://github.com/VoiTee)

