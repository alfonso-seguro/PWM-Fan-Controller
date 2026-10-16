# PWM-Fan-Controller

  ESP32 Server Cabinet Fan Controller - Final Production Version
  ----------------------------------------------------------------
  Hardware: ESP32, SHT31-D (0x44), Elegoo Bi-Color OLED (0x3C), Arctic P12/F14 PWM
  
  Features:
    - 24/7 Hardware-safety critical design (Watchdog, Active-HIGH Alert)
    - SHT31 Periodic Data Acquisition Mode
    - Non-blocking Wi-Fi dashboard & WebSockets
    - Clean Serial logging (Boot & Network state changes only)
    - Anti-Stall Kickstart (Tach watchdog)
    - True Non-blocking Soft Start
    - Bi-color OLED layout optimization
