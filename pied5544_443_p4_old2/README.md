# Project 3 #

Demonstrates the use of FreeRTOS task scheduling and interrupt handling. Receives messages from UART1, stores them in an I2C EEPROM, retrieves them from the EEPROM, and prints them to an LCD. Uses the UART1 RX interrupt to receive individual characters from UART1. Passes them through a queue to a task, which stores them in the EEPROM. When BTN1 is pressed, another task will read from the EEPROM and display the message on the LCD, breaking lines in nice places and scrolling upward. LEDA and LEDB indicate whether the EEPROM can be written to or read from, and LEDC acts as a "heartbeat" to assess timings.

### Author ###

Parker Piedmont

### Date Created ###

September 21, 2021