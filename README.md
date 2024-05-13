# IOT_SYSTEM_FOR_SMART_AGRICULTURE_ESP32_Linux

## Environment

Ubuntu22.04 in VMware workstation player 17, ESP-IDF 5.1.1, VSCode;
ESP Rainmaker and EspBlufi on the phone

## Devices in use

1.  ESP32-S3-LCD-EV-Board
2.  ESP32-S3-DevKitM-1
3.  HTS221
4.  BH1750
5.  Capacitive soil moisture sensor
6.  LED matrix based on WS2812B
7.  Relay and water pump

## Update History

### 2023/9/13

Converting the whole environment from Windows to Linux

### 2024/2/5

Complete the whole mesh network using the IP internal networking example
I originally tried the manual networking example but there are [some bugs in data transmission by just using esp_mesh_send()](ref:https://esp32.com/viewtopic.php?t=9181). So I have to switch to the new example.
```
Data transmission: 
 -Root->Node: Router table transmission
 -Node->Root: Switch pressing detection
```
### 2024/3/5

Complete the wifi provisioning by blufi (GATT is too complex), packing the functions into components and do some minor changes for stability.

### 2024/4/30

Complete the whole project
1. Replace the MQTT communication with mesh data transmission functions
2. Add a small board (ESP32-S3-DevKitM-1) to the Router part to act as the router for Rainmaker, which communicates with mesh root through UART.
3. Add commands to the nodes to control the actuators

## The whole structure
Rainmaker <--> Router(ESP32-S3-DevKitM-1) <--> Mesh Root(ESP32-S3-LCD-EV-Board) <--> Mesh nodes(ESP32-S3-DevKitM-1)

## For those who will use the project for further development
The connections between the root and the nodes are currently completely maintained by ESP Mesh, which actually reacts slowly. I personally suggest to make the root send an OK to the node who has send the data to let the node knows that the data has been transmitted successfully. 
