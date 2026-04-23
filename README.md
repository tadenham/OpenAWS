OpenAWS is an ESP32-based automated remote weather station. The station transmits via cellular or satellite network.

Currently, the following modules are supported:
 SIMCom SIM7070G (GSM/LTE-M/NB-IoT),
  SIMCom A7670G (GSM/LTE Cat-1),
  and Murata Type 1SC-NTN (NB-NTN geostationary satellite)
 
OpenAWS supports the following sensors:
  temperature/humidity (Sensirion SHT4x or SHT3x),
  precipitation (hall switch),
  barometric pressure (Bosch BMP580),
  wind speed (hall switch),
  and wind direction (AMS AS5600)

The station transmits to the OpenAWS server over UDP and receives a Unix timestamp in response which is used to set the ESP32 clock. This eliminates dependence on unreliable NITZ or NTP updates.
If a response is not received from the server, the data is stored locally and retransmitted at a later time. This ensures guaranteed delivery.

Major station settings are configured in the settings.h file. Otherwise, the station is configured at installation via Bluetooth.

Please visit OpenAWS.org to view stations and further documentation.
