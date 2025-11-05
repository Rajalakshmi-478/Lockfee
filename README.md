"# Lockfee"
ESP32 TOTP Smart Lock System
📘 Overview

This project uses an ESP32 to build a secure smart lock system powered by TOTP (Time-based One-Time Password) authentication.
The ESP32 generates and validates 6-digit time-based codes, displays status on an OLED screen, controls a relay lock, and supports Bluetooth for configuration and RTC synchronization.

⚙️ Features

✅ Secure TOTP-based access (6-digit code)
✅ OLED display for real-time feedback
✅ Relay-controlled lock with 30-second unlock timer
✅ Keypad input with buzzer feedback
✅ Deep sleep mode for power saving
✅ Bluetooth mode for RTC sync and TOTP access
✅ Stable Lock mode (double * press) to disable unlocking temporarily
✅ Automatic re-lock and sleep timeout

🧩 Hardware Requirements
Component	Description
ESP32 Board	Main controller (Wi-Fi + Bluetooth + TOTP logic)
DS3231 RTC Module	Provides accurate real-time clock for TOTP
OLED Display (128x32)	Displays password, lock status, and info
4x3 Keypad	Used to enter 6-digit TOTP code
Relay Module	Controls door lock mechanism
Buzzer	Provides feedback for valid/invalid input
Push Button	Short press = unlock / Long press = enable Bluetooth

⚡ Working Principle

The RTC DS3231 provides the current time.

The ESP32 generates a TOTP code using a predefined HMAC key.

The user enters the 6-digit code on the keypad.

If the entered code matches the current or previous valid TOTP,
the relay activates for 30 seconds (door unlocked).

After timeout, the lock automatically re-engages.

Deep sleep saves power when idle.

Holding the unlock button enables Bluetooth, allowing:

RTC time update

Viewing the current TOTP code
