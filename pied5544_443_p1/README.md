# Project 1 #

Demonstrates the use of FreeRTOS task scheduling. Toggles the state of LED1 or LED2 whenever BTN1 or BTN2, respectively, is pressed. Uses two tasks to accomplish this:

* sendButton
* toggleLED

The value of each button is sent from sendButton to toggleLED using two queues.

### Author ###

Parker Piedmont

### Date Created ###

September 7, 2021