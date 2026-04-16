#if defined(ESP32) || defined(TARGET_PORTENTA_H7_M7) || defined(ARDUINO_NANO_RP2040_CONNECT) || defined(ARDUINO_WIO_TERMINAL) || defined(BOARD_WITH_ESP_AT)
#include <Arduino.h>


#if defined(ESP32) || defined(TARGET_PORTENTA_H7_M7)
#include <WiFi.h>
#include <WiFiUdp.h>
#elif defined(ARDUINO_NANO_RP2040_CONNECT)
#include <SPI.h>
#include <WiFiNINA.h>
#elif defined(ARDUINO_WIO_TERMINAL)
#include <rpcWiFi.h>
#include <WiFiUdp.h>
#elif defined(BOARD_WITH_ESP_AT)
#include <WiFiEspAT.h>
#endif

#include <micro_ros_arduino.h>

extern "C"
{

  static WiFiUDP udp_client;

  bool arduino_wifi_transport_open(struct uxrCustomTransport * transport)
  {
    struct micro_ros_agent_locator * locator = (struct micro_ros_agent_locator *) transport->args;
    // Ensure socket is properly closed before reopening
    udp_client.stop();
    delay(10);
    // Begin listening on the specified port
    bool success = udp_client.begin(locator->port);
    if (success) {
      delay(10);
    }
    return success;
  }

  bool arduino_wifi_transport_close(struct uxrCustomTransport * transport)
  {
    udp_client.stop();
    return true;
  }

  size_t arduino_wifi_transport_write(struct uxrCustomTransport * transport, const uint8_t *buf, size_t len, uint8_t *errcode)
  {
    (void)errcode;
    struct micro_ros_agent_locator * locator = (struct micro_ros_agent_locator *) transport->args;

    // Validate inputs
    if (buf == NULL || len == 0) {
      return 0;
    }

    // Check if beginPacket was successful before writing
    if (!udp_client.beginPacket(locator->address, locator->port)) {
      // If beginPacket fails, socket may be in bad state - return 0
      return 0;
    }
    
    size_t sent = udp_client.write(buf, len);
    
    // Check if endPacket succeeds, but don't fail if it doesn't
    int end_result = udp_client.endPacket();
    if (end_result == 0) {
      // endPacket failed but data was queued, force flush
      udp_client.flush();
    }

    return sent;
  }

  size_t arduino_wifi_transport_read(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode)
  {
    (void) errcode;

    uint32_t start_time = millis();

    while(millis() - start_time < timeout && udp_client.parsePacket() == 0){
      delay(1);
    }

    int readed  = udp_client.read(buf, len);

    return (readed < 0) ? 0 : readed;
  }
}

#endif

