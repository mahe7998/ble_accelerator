# ble_accelerator
Tinyduino BLE + Accelerator code for Arduino Environment.

This code is intended to work on Tinyduino ATmega328 (3.3V, 8MHz) with nRF8001 and ASD2611-R (TinyCircuits)

It uses a Blutooth LE Serial (UART) connection to transfer accelerator data from the Bosch BMA250 over BTLE to a remote PC. 
This can be used to track 3D motion when attached to an object.

This code is based on the Nordic Sample code - See Copyright in ble_accelerator.c
