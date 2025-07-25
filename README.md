# Overview

This program is firmware to CNC driver module based on ARM STM32H725. Program is configured to use dedicated PCB - see schematic in PCB directory. 
Project is created in Segger Embedded Studio format.

It is available also version prepared to run on NUCLEO-F439ZI module, created in STM32CubeIDE : 
https://github.com/r-gal/CNC_Nucleo_stm32f439

 Module need GUI application on PC, source is in https://github.com/r-gal/CNC_app repository.


# Compilation

General information: Project is prepared on IDE that uses GCC compiller, files from System directory may need be adapted to other compillers.
Program is prepared to use on dedicated Hardware, Schematic and PCB layout in KiCad format is available in PCB directory.
Program uses 3rd party software: CMSIS, HAL libraries and FreeRTOS, in Core directory files generated in STM32CubeMX tool are stored.

1. Clone this repository with submodules

```
git clone https://github.com/r-gal/CNC --recursive
```

2. Open project in Segger Embedded Studio or import to your IDE

3. Change default IP address to wanted in file CNC_source/EthernetConfig.hpp file

For default program use static IP address, it may be configured in EthernetConfig.hpp file
```
  #define DEFAULT_IP 0xC0A83732 /*192.168.55.50*/
  #define DEFAULT_DHCP_SERVER  0xC0A83701 /*192.168.55.1*/
  #define DEFAULT_DNS_SERVER 0xC0A83701 /*192.168.55.1*/
  #define DEFAULT_SUBNET_MASK 0xFFFFFF00
  #define DEFAULT_GATEWAY 0xC0A83701 /*192.168.55.1*
 ``` 
# Usage

1. Load generated software into module
2. connect by ethernet wire with PC
3. Use GUI application to drive module

