# Project Name
E-Bike Tracking and Alert System

## Description
This project implements Geofence Security Alerts, Emergency Location Sharing via SMS, and Emergency Call Assistance using Arduino, SIM900 GSM module, and NEO-6M GPS.

## Features
- Sends an alert SMS with GPS location when a button is pressed.
- Makes an emergency call when another button is pressed.
- Implements geofencing for security alerts.

## Hardware Components
- **Olimexino-32U4 Microcontroller**
- **SIM900 GSM Module**
- **NEO-6M GPS Module**
- **Push Buttons**
- **3.7V Li-ion Batteries**

## Power Considerations
- The GPS module operates at **3.3V**.
- The SIM900 GSM and microcontroller require **5V**.
- Batteries take time to recharge and must provide stable voltage.

## Setup & Usage
1. Connect the components as per the circuit diagram.
2. Upload the Arduino sketch.
3. Monitor serial output for debugging.
4. Press buttons for emergency SMS or calls.

## Authors
Anika Tabassum, Prajakta Mestri, Pronab Malaker
