#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <esp_sleep.h>
#include <esp_system.h>

// Web page served over HTTP — generated from index.html by
// tools/gen_index_html.py (Arduino IDE cannot embed files directly).
#include "index_html.h"

// GY-85 IMU: driver + 9-DoF sensor fusion (ADXL345 + ITG3205 + QMC5883L).
// Settings and API live in gy85_imu.h / gy85_imu.cpp.
#include "gy85_imu.h"

/* =============== config section start =============== */

#if __has_include("credentials.h")
#include "credentials.h"
#else

// WiFi credentials
#define NUM_NETWORKS 2
// Add your networks credentials here
const char *ssidTab[NUM_NETWORKS] = {
    "wifi-ssid-one",
    "wifi-ssid-two",
};
const char *passwordTab[NUM_NETWORKS] = {
    "wifi-pass-one",
    "wifi-pass-two",
};
#endif

/* --- network --- */

// mDNS host name: the board is reachable at <MDNS_HOSTNAME>.local
#define MDNS_HOSTNAME "esp32c3"

/* --- deep sleep (motion activated) --- */

// Enter deep sleep after this many milliseconds without movement and with
// no web browser connected. Set to 0 to keep the board always awake.
#define SLEEP_IDLE_MS 60000

/* =============== config section end =============== */

#define HTTP_PORT 80
#define WEBSOCKET_PORT 8001

// you can provide credentials to multiple WiFi networks
WiFiMulti wifiMulti;

// HTTP server on port 80
WebServer server(HTTP_PORT);

// WebSocket server
WebSocketsServer webSocket = WebSocketsServer(WEBSOCKET_PORT);

StaticJsonDocument<512> jsonDocTx;

// Web page content from index_html.h (PROGMEM string).
const String html = String(index_html);

bool wsconnected = false;

void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t *payload,
                      size_t length) {
  switch (type) {
    case WStype_DISCONNECTED: {
      wsconnected = false;
      Serial.printf("[%u] Disconnected\r\n", num);
    } break;
    case WStype_CONNECTED: {
      wsconnected = true;
      Serial.printf("\r\n[%u] Connection from client\r\n", num);
    } break;

    case WStype_TEXT: {
      /* Commands from the web page (the "Calibrate" button). */
      if (length == 9 && memcmp(payload, "calibrate", 9) == 0) {
        gy85CalRequest = true;
        Serial.printf("[%u] Calibration requested from the web page\r\n", num);
      } else {
        Serial.printf("[%u] Text:\r\n", num);
        for (int i = 0; i < length; i++) {
          Serial.printf("%c", (char)(*(payload + i)));
        }
        Serial.println();
      }
    } break;

    case WStype_BIN:
    case WStype_ERROR:
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
    default:
      break;
  }
}

void onHttpReqFunc() {
  server.sendHeader("Connection", "close");
  server.send(200, "text/html", html);
}

/* Put the board into deep sleep; the ADXL345 activity interrupt wakes it up
   again when the module is moved (gy85PrepareWakeOnMotion()).
   Does not return. */
void enterDeepSleep(void) {
  Serial.println("No movement - entering deep sleep (move the module to wake "
                 "up)");

  gy85PrepareWakeOnMotion();

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(50);

  esp_deep_sleep_enable_gpio_wakeup(1ULL << GY85_PIN_WAKE,
                                    ESP_GPIO_WAKEUP_GPIO_HIGH);
  esp_deep_sleep_start();  // does not return
}

void taskWifi(void *parameter);
void taskStatus(void *parameter);

SemaphoreHandle_t mtx;

void setup() {
  Serial.begin(115200);

  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_GPIO) {
    Serial.println("Woke up from deep sleep - motion detected");
  }

  mtx = xSemaphoreCreateMutex();
  xSemaphoreGive(mtx);

  xTaskCreatePinnedToCore(taskWifi,   /* Task function. */
                          "taskWifi", /* String with name of task. */
                          20000,      /* Stack size in bytes. */
                          NULL, /* Parameter passed as input of the task */
                          2,    /* Priority of the task. */
                          NULL, /* Task handle. */
                          0);   /* Core where the task should run */

  xTaskCreatePinnedToCore(taskStatus,   /* Task function. */
                          "taskStatus", /* String with name of task. */
                          20000,        /* Stack size in bytes. */
                          NULL, /* Parameter passed as input of the task */
                          3,    /* Priority of the task. */
                          NULL, /* Task handle. */
                          0);   /* Core where the task should run */
}

void taskWifi(void *parameter) {
  uint8_t stat = WL_DISCONNECTED;

  /* Configure Wi-Fi */
  for (int i = 0; i < NUM_NETWORKS; i++) {
    wifiMulti.addAP(ssidTab[i], passwordTab[i]);
    Serial.printf("WiFi %d: SSID: \"%s\" ; PASS: \"%s\"\r\n", i, ssidTab[i],
                  passwordTab[i]);
  }

  /* DHCP host name (the mDNS name is started after connecting) */
  WiFi.setHostname(MDNS_HOSTNAME);

  while (stat != WL_CONNECTED) {
    stat = wifiMulti.run();
    Serial.printf("WiFi status: %d\r\n", (int)stat);
    delay(100);
  }

  Serial.printf("WiFi connected\r\n", (int)stat);
  Serial.printf("IP address: ");
  Serial.println(WiFi.localIP());

  /* mDNS: the board is reachable at <MDNS_HOSTNAME>.local */
  if (!MDNS.begin(MDNS_HOSTNAME)) {
    Serial.println("mDNS responder failed to start");
  } else {
    MDNS.addService("http", "tcp", HTTP_PORT);
    if (HTTP_PORT == 80) {
      Serial.printf("mDNS: http://%s.local/\r\n", MDNS_HOSTNAME);
    } else {
      Serial.printf("mDNS: http://%s.local:%d/\r\n", MDNS_HOSTNAME, HTTP_PORT);
    }
  }

  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);

  /* Confgiure HTTP server */
  server.on("/", HTTP_GET, onHttpReqFunc);
  server.on("/index.html", HTTP_GET, onHttpReqFunc);
  server.begin();

  while (1) {
    while (WiFi.status() == WL_CONNECTED) {
      if (xSemaphoreTake(mtx, 5) == pdTRUE) {
        webSocket.loop();
        server.handleClient();
        xSemaphoreGive(mtx);
      }
      delay(5);
    }
    Serial.printf("WiFi disconnected, reconnecting\r\n");
    delay(500);
    stat = wifiMulti.run();
    Serial.printf("WiFi status: %d\r\n", (int)stat);
  }
}

void taskStatus(void *parameter) {
  String output;

  if (!gy85Init()) {
    while (1) {
      Serial.println("Unable to communicate with GY-85");
      Serial.println("Check connections, and try again.");
      delay(5000);
    }
  }
  unsigned long lastMicros = micros();
  unsigned long lastMotionMs = millis();

  while (1) {
    unsigned long nowMicros = micros();
    float dt = (nowMicros - lastMicros) * 1e-6f;
    lastMicros = nowMicros;
    if (dt > 0.1f) dt = 0.1f;

    gy85Update(dt);

    /* Manual calibration requested from the web page? */
    if (gy85CalRequest) {
      gy85CalRequest = false;
      gy85CalReply = gy85StartManualRecal() ? 1 : 3;
    }

    if (gy85RecalTick()) {
      lastMotionMs = millis();  // do not sleep right after re-calibrating
    }

    /* Deep sleep when nothing moves and nobody is watching the page. */
    if (gy85Moving) {
      lastMotionMs = millis();
    } else if (SLEEP_IDLE_MS > 0 && !wsconnected && !gy85RecalActive &&
               (millis() - lastMotionMs > (unsigned long)SLEEP_IDLE_MS)) {
      enterDeepSleep();  // does not return
    }

    /* Calibration status reply to the web page. */
    if (gy85CalReply != 0) {
      const char *calMsg = "{\"cal\":\"refused\"}";
      if (gy85CalReply == 1) {
        calMsg = "{\"cal\":\"started\"}";
      } else if (gy85CalReply == 2) {
        calMsg = "{\"cal\":\"done\"}";
      } else if (gy85CalReply == 4) {
        calMsg = "{\"cal\":\"aborted\"}";
      }
      if (wsconnected == true) {
        if (xSemaphoreTake(mtx, 5) == pdTRUE) {
          webSocket.sendTXT(0, calMsg);
          xSemaphoreGive(mtx);
        }
      }
      gy85CalReply = 0;
    }

    // Euler angles -> quaternion for the 3D cube
    float qw, qx, qy, qz;
    gy85EulerToQuat(gy85Roll, gy85Pitch, gy85Yaw, &qw, &qx, &qy, &qz);

    output = "";
    jsonDocTx.clear();
    jsonDocTx["q0"] = qw;
    jsonDocTx["q1"] = qx;
    jsonDocTx["q2"] = qy;
    jsonDocTx["q3"] = qz;
    jsonDocTx["roll"] = gy85Roll;
    jsonDocTx["pitch"] = gy85Pitch;
    jsonDocTx["yaw"] = gy85Yaw;
    serializeJson(jsonDocTx, output);

    Serial.printf(
        "Qgy85=[%f,%f,%f,%f] roll=%f pitch=%f yaw=%f mag=[%d,%d,%d]\r\n", qw,
        qx, qy, qz, gy85Roll, gy85Pitch, gy85Yaw, gy85MagRaw[0], gy85MagRaw[1],
        gy85MagRaw[2]);

    if (wsconnected == true) {
      if (xSemaphoreTake(mtx, 5) == pdTRUE) {
        webSocket.sendTXT(0, output);
        xSemaphoreGive(mtx);
      }
    }
    delay(20);
  }
}

void loop() {
  Serial.printf("loop() running on core %d\r\n", xPortGetCoreID());
  while (1) {
    Serial.printf("[RAM: %d]\r\n", esp_get_free_heap_size());
    delay(1000);
  }
}
