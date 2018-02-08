# NXP Car Project

The following repository documents NXP car assignment made for the purpose of Electronics and Electrical Engineering course on Northumbria University.

## Autonomous operation
The main objective of the project is to correctly assemble and program an autonomous NXP car. It must be capable of driving around the track provided by Northumbria University which is a simplified version of the real competition one. In comparison to the full version of the NXP layout, it does not include special elements such as chicanes, start/stop lines, intersections or hills. The track is a white, 1’10’’ inch (approximately 56 cm) lane surrounded by 1’’ (approximately 2,54 cm) black lines – as illustrated on Figure 1 and Figure 2 below. It is made of matte, plastic-based material so that any reflections from the ambient lightning are reduced. The track is a closed, elliptic-shaped model consisting of two 180 degree corners and two parallel straights, there are no start/finish markings.  

## Hardware
The vehicle consists of FRDM-KL25Z board with a TFC-Shield that provides a convenient and structured layout to link all the essential components. The main control unit is 48MHz NXP-KL25Z MCU based on ARM cortex-M0+; high-performance core with 16kB SRAM and 128kb of the flash memory. It features UART, SPI and I2C interfaces which enable to connect additional peripherals. The PC connection and debugging is established via PEMicro OpenSDA  - USB to Micro-USB interface.

## Video
This is a short video presenting the car's performance during the demonstration assessment.

[![Video](https://img.youtube.com/vi/uWvNPKmrz1I/0.jpg)](https://www.youtube.com/watch?v=uWvNPKmrz1I)
