# ECE 443: Distributed Processing and Control Networks

This repository stores my work for ECE 443 at the University of Idaho. All projects in this repository were built for the PIC32MX using MPLAB.

### Project 1

Toggles LEDA and LEDB depending on whether BTN1 and BTN2 are pressed. Uses a queue to transfer information between tasks that read the buttons and a task that toggles the LEDs.

### Project 2

Demonstrates FreeRTOS task scheduling and priorities using LEDA -- LEDD. Each LED is operated at different times by a different task, some of which are tied to BTN1.

### Project 3

Scheduling and communication using FreeRTOS. Receives messages from UART1 and writes them to an I2C EEPROM. When BTN1 is pressed, reads a message from the EEPROM and displays it on the LCD, breaking lines in nice places and scrolling upward.