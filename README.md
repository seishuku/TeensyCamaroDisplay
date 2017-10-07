# TeensyCamaroDisplay
Aux display for my 2011 Camaro.
Reads off the single wire CANbus, displays various data on a DFRobot 8 digit 7-segment SPI display.
Also displays some data on the cluster's navigation DIC text screen and handles steering wheel controls for my Kenwood radio.

Uses a much improved FlexCAN driver, interrupt driven and makes use of multiple mailboxes.
Steering wheel controls are MUCH more responsive now.
