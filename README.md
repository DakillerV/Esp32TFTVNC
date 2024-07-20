ESP32 VNC TFT

Required Parts: Esp32(nodemcu-32s), SPI tft display

Required Libraries: ArduinoJson, ESP32Encoder, Adafruit GFX Library, Adafruit ILI9341, arduinoVNC

Configuration: Change ssid, password, vnc ip , vnc port & vnc password (optional)
Screen config: ILI9341 display configured in hardware spi using esp's VSPI line (use this for fastest screen update or alternatively use software spi)

Issues: assides from buggy code, there are two known issues:

1: Build error from "TFT_eSPI" not being found, to resolve comment out "#define VNC_RA8875" in the file "VNC_config.h" found in the arduinoVNC source files

2: Tft output doesn't fit the screen, To resolve, in the "Adafruit_ILI9341.h" file flip screen width and height. 
