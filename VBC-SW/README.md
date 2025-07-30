As of right now, only the dev branch of simpleFOC works with the latest ESP32 v3 for Arduino IDE. I have made some modifications to it to limit the current commanded for safety and added Dagor's modifications to improve ISR-ADC reading precision of the phases and PWM frequency modification.

The other option is to use Dagor's modified library (2.3.3) with ESP32 v2 for Arduino IDE. This version (included in this folder) still works a bit better.

For now we go with the SECOND option. Install the library by using the Include Library/Add .ZIP Library.