#define DIRECT // Use NeoPixelBus and sleep

#include <Arduino.h>
#ifdef DIRECT
#include <NeoPixelBus.h>
#include <utility/MPU6886.h>
#else
#include <M5Atom.h>
#endif
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLECharacteristic.h>
#include <BLEUtils.h>
#include <HIDTypes.h>
#include <BLEHIDDevice.h>
#include <BLE2902.h>

//#define INVERT // Swap front and back
//#define BLE_TASK // BLE on separate task

#define MOUSE_LEFT 1
#define MOUSE_RIGHT 2
#define MOUSE_MIDDLE 4
#define MOUSE_BACK 8
#define MOUSE_FORWARD 16

#ifdef DIRECT
const uint8_t BRIGHTNESS = 4;

RgbColor color(BRIGHTNESS, BRIGHTNESS, 0); // Yellow
#else
CRGB color = CRGB(255, 255, 0); // Yellow
#endif

class BLEMouseCallbacks : public BLEServerCallbacks {
public:
  BLEMouseCallbacks(BLECharacteristic *input) : _input(input), _connected(false) {}

  void onConnect(BLEServer *pServer);
  void onDisconnect(BLEServer *pServer);

protected:
  friend class BLEMouse;

  BLECharacteristic *_input;
  bool _connected;
};

class BLEMouse {
public:
  BLEMouse() : _buttons(0) {}

#ifdef BLE_TASK
  bool begin();
#else
  void begin();
#endif
  void click(uint8_t b);
  void move(int8_t x, int8_t y, int8_t wheel = 0, int8_t tilt = 0);
  void press(uint8_t b);
  void release(uint8_t b);
  bool isPressed(uint8_t b);
  bool isConnected();

#ifdef BLE_TASK
protected:
  void run();
#endif

private:
  void buttons(uint8_t b);

  BLEServer *_server;
  BLESecurity *_security;
  BLEAdvertising *_advertising;
  BLEMouseCallbacks *_callbacks;
  BLEHIDDevice *_hid;
  BLECharacteristic *_input;

  uint8_t _buttons;
};

static const uint8_t _hidReportDescriptor[] = {
  USAGE_PAGE(1),       0x01, // USAGE_PAGE (Generic Desktop)
  USAGE(1),            0x02, // USAGE (Mouse)
  COLLECTION(1),       0x01, // COLLECTION (Application)
  USAGE(1),            0x01, // USAGE (Pointer)
  COLLECTION(1),       0x00, // COLLECTION (Physical)
  REPORT_ID(1),        0x01, // REPORT_ID (1)
  // ------------------------------------------------- Buttons (Left, Right, Middle, Back, Forward)
  USAGE_PAGE(1),       0x09, // USAGE_PAGE (Button)
  USAGE_MINIMUM(1),    0x01, // USAGE_MINIMUM (Button 1)
  USAGE_MAXIMUM(1),    0x05, // USAGE_MAXIMUM (Button 5)
  LOGICAL_MINIMUM(1),  0x00, // LOGICAL_MINIMUM (0)
  LOGICAL_MAXIMUM(1),  0x01, // LOGICAL_MAXIMUM (1)
  REPORT_SIZE(1),      0x01, // REPORT_SIZE (1)
  REPORT_COUNT(1),     0x05, // REPORT_COUNT (5)
  HIDINPUT(1),         0x02, // INPUT (Data, Variable, Absolute) ;5 button bits
  // ------------------------------------------------- Padding
  REPORT_SIZE(1),      0x03, // REPORT_SIZE (3)
  REPORT_COUNT(1),     0x01, // REPORT_COUNT (1)
  HIDINPUT(1),         0x03, // INPUT (Constant, Variable, Absolute) ;3 bit padding
  // ------------------------------------------------- X/Y position, Wheel
  USAGE_PAGE(1),       0x01, // USAGE_PAGE (Generic Desktop)
  USAGE(1),            0x30, // USAGE (X)
  USAGE(1),            0x31, // USAGE (Y)
  USAGE(1),            0x38, // USAGE (Wheel)
  LOGICAL_MINIMUM(1),  0x81, // LOGICAL_MINIMUM (-127)
  LOGICAL_MAXIMUM(1),  0x7f, // LOGICAL_MAXIMUM (127)
  REPORT_SIZE(1),      0x08, // REPORT_SIZE (8)
  REPORT_COUNT(1),     0x03, // REPORT_COUNT (3)
  HIDINPUT(1),         0x06, // INPUT (Data, Variable, Relative) ;3 bytes (X, Y, Wheel)
  // ------------------------------------------------- Horizontal wheel
  USAGE_PAGE(1),       0x0c, // USAGE PAGE (Consumer Devices)
  USAGE(2),      0x38, 0x02, // USAGE (AC Pan)
  LOGICAL_MINIMUM(1),  0x81, // LOGICAL_MINIMUM (-127)
  LOGICAL_MAXIMUM(1),  0x7f, // LOGICAL_MAXIMUM (127)
  REPORT_SIZE(1),      0x08, // REPORT_SIZE (8)
  REPORT_COUNT(1),     0x01, // REPORT_COUNT (1)
  HIDINPUT(1),         0x06, // INPUT (Data, Var, Rel)
  END_COLLECTION(0),         // END_COLLECTION
  END_COLLECTION(0)          // END_COLLECTION
};

void BLEMouseCallbacks::onConnect(BLEServer *pServer) {
  BLE2902 *desc = (BLE2902*)_input->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));

  desc->setNotifications(true);
  _connected = true;
#ifdef DIRECT
  color = RgbColor(0, 0, BRIGHTNESS); // Blue
#else
  color = CRGB(0, 0, 255); // Blue
#endif
}

void BLEMouseCallbacks::onDisconnect(BLEServer *pServer) {
  BLE2902 *desc = (BLE2902*)_input->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));

  desc->setNotifications(false);
  _connected = false;
#ifdef DIRECT
  color = RgbColor(BRIGHTNESS, BRIGHTNESS, 0); // Yellow
#else
  color = CRGB(255, 255, 0); // Yellow
#endif
}

#ifdef BLE_TASK
bool BLEMouse::begin() {
  return xTaskCreate((TaskFunction_t)&BLEMouse::run, "BLE", 10240, this, 5, NULL) == pdPASS;
#else
void BLEMouse::begin() {
  BLEDevice::init("M5Atom BLE Mouse");

  _server = BLEDevice::createServer();
  _hid = new BLEHIDDevice(_server);
  _hid->manufacturer()->setValue("M5Stack");
  _hid->pnp(0x02, 0xe502, 0xa111, 0x0210);
  _hid->hidInfo(0x00, 0x02);
  _input = _hid->inputReport(1); // <-- input REPORTID from report map
  _callbacks = new BLEMouseCallbacks(_input);
  _server->setCallbacks(_callbacks);

  _security = new BLESecurity();
  _security->setKeySize();
  _security->setAuthenticationMode(ESP_LE_AUTH_BOND);

  _hid->reportMap((uint8_t*)_hidReportDescriptor, sizeof(_hidReportDescriptor));
  _hid->startServices();

  _advertising = _server->getAdvertising();
  _advertising->setAppearance(HID_MOUSE);
  _advertising->addServiceUUID(_hid->hidService()->getUUID());
  _advertising->start();

  _hid->setBatteryLevel(100);
#endif
}

void BLEMouse::click(uint8_t b) {
  _buttons = b;
  move(0, 0, 0, 0);
  _buttons = 0;
  move(0, 0, 0, 0);
}

void BLEMouse::move(int8_t x, int8_t y, int8_t wheel, int8_t tilt) {
  if (isConnected()) {
    uint8_t m[5];

    m[0] = _buttons;
    m[1] = x;
    m[2] = y;
    m[3] = wheel;
    m[4] = tilt;
    _input->setValue(m, sizeof(m));
    _input->notify();
  }
}

void BLEMouse::buttons(uint8_t b) {
  if (b != _buttons) {
    _buttons = b;
    move(0, 0, 0, 0);
  }
}

void BLEMouse::press(uint8_t b) {
  buttons(_buttons | b);
}

void BLEMouse::release(uint8_t b) {
  buttons(_buttons & ~b);
}

bool BLEMouse::isPressed(uint8_t b) {
  return (_buttons & b) != 0;
}

bool BLEMouse::isConnected() {
  return _callbacks->_connected;
}

#ifdef BLE_TASK
void BLEMouse::run() {
  BLEDevice::init("M5Atom BLE Mouse");

  _server = BLEDevice::createServer();
  _hid = new BLEHIDDevice(_server);
  _hid->manufacturer()->setValue("M5Stack");
  _hid->pnp(0x02, 0xe502, 0xa111, 0x0210);
  _hid->hidInfo(0x00, 0x02);
  _input = _hid->inputReport(1); // <-- input REPORTID from report map
  _callbacks = new BLEMouseCallbacks(_input);
  _server->setCallbacks(_callbacks);

  _security = new BLESecurity();
  _security->setKeySize();
  _security->setAuthenticationMode(ESP_LE_AUTH_BOND);

  _hid->reportMap((uint8_t*)_hidReportDescriptor, sizeof(_hidReportDescriptor));
  _hid->startServices();

  _advertising = _server->getAdvertising();
  _advertising->setAppearance(HID_MOUSE);
  _advertising->addServiceUUID(_hid->hidService()->getUUID());
  _advertising->start();

  _hid->setBatteryLevel(100);

  vTaskDelay(portMAX_DELAY);
}
#endif

#ifdef DIRECT
NeoPixelBus<NeoGrbFeature, NeoEsp32Rmt0Ws2812xMethod> leds(25, 27);

static void drawArrow(int8_t horz, int8_t vert) {
  leds.ClearTo(RgbColor(0));
  if (horz < 0) {
    if (vert < 0) {
      leds.SetPixelColor(0, color);
      leds.SetPixelColor(6, color);
    } else if (vert > 0) {
      leds.SetPixelColor(16, color);
      leds.SetPixelColor(20, color);
    } else {
      leds.SetPixelColor(10, color);
      leds.SetPixelColor(11, color);
    }
  } else if (horz > 0) {
    if (vert < 0) {
      leds.SetPixelColor(8, color);
      leds.SetPixelColor(4, color);
    } else if (vert > 0) {
      leds.SetPixelColor(18, color);
      leds.SetPixelColor(24, color);
    } else {
      leds.SetPixelColor(13, color);
      leds.SetPixelColor(14, color);
    }
  } else {
    if (vert < 0) {
      leds.SetPixelColor(2, color);
      leds.SetPixelColor(7, color);
    } else if (vert > 0) {
      leds.SetPixelColor(17, color);
      leds.SetPixelColor(22, color);
    }
  }
  leds.SetPixelColor(12, color);
  leds.Show();
}

static void drawClick(bool left) {
  leds.ClearTo(RgbColor(0));
  if (left) {
#ifdef INVERT
    leds.SetPixelColor(3, color);
    leds.SetPixelColor(23, color);
    for (uint8_t y = 1; y < 4; ++y)
      leds.SetPixelColor(y * 5 + 4, color);
#else
    for (uint8_t y = 1; y < 4; ++y)
      leds.SetPixelColor(y * 5, color);
    leds.SetPixelColor(1, color);
    leds.SetPixelColor(21, color);
#endif
  } else { // right
#ifdef INVERT
    for (uint8_t y = 1; y < 4; ++y)
      leds.SetPixelColor(y * 5, color);
    leds.SetPixelColor(1, color);
    leds.SetPixelColor(21, color);
#else
    leds.SetPixelColor(3, color);
    leds.SetPixelColor(23, color);
    for (uint8_t y = 1; y < 4; ++y)
      leds.SetPixelColor(y * 5 + 4, color);
#endif
  }
  for (uint8_t y = 0; y < 5; ++y)
    leds.SetPixelColor(y * 5 + 2, color);
  leds.Show();
}

#else
static void drawArrow(int8_t horz, int8_t vert) {
  M5.dis.clear();
  if (horz < 0) {
    if (vert < 0) {
      M5.dis.drawpix(0, 0, color);
      M5.dis.drawpix(1, 1, color);
    } else if (vert > 0) {
      M5.dis.drawpix(1, 3, color);
      M5.dis.drawpix(0, 4, color);
    } else {
      M5.dis.drawpix(0, 2, color);
      M5.dis.drawpix(1, 2, color);
    }
  } else if (horz > 0) {
    if (vert < 0) {
      M5.dis.drawpix(3, 1, color);
      M5.dis.drawpix(4, 0, color);
    } else if (vert > 0) {
      M5.dis.drawpix(3, 3, color);
      M5.dis.drawpix(4, 4, color);
    } else {
      M5.dis.drawpix(3, 2, color);
      M5.dis.drawpix(4, 2, color);
    }
  } else {
    if (vert < 0) {
      M5.dis.drawpix(2, 0, color);
      M5.dis.drawpix(2, 1, color);
    } else if (vert > 0) {
      M5.dis.drawpix(2, 3, color);
      M5.dis.drawpix(2, 4, color);
    }
  }
  M5.dis.drawpix(2, 2, color);
}

static void drawClick(bool left) {
  M5.dis.clear();
  if (left) {
#ifdef INVERT
    M5.dis.drawpix(3, 0, color);
    M5.dis.drawpix(3, 4, color);
    for (uint8_t y = 1; y < 4; ++y)
      M5.dis.drawpix(4, y, color);
#else
    for (uint8_t y = 1; y < 4; ++y)
      M5.dis.drawpix(0, y, color);
    M5.dis.drawpix(1, 0, color);
    M5.dis.drawpix(1, 4, color);
#endif
  } else { // right
#ifdef INVERT
    for (uint8_t y = 1; y < 4; ++y)
      M5.dis.drawpix(0, y, color);
    M5.dis.drawpix(1, 0, color);
    M5.dis.drawpix(1, 4, color);
#else
    M5.dis.drawpix(3, 0, color);
    M5.dis.drawpix(3, 4, color);
    for (uint8_t y = 1; y < 4; ++y)
      M5.dis.drawpix(4, y, color);
#endif
  }
  for (uint8_t y = 0; y < 5; ++y)
    M5.dis.drawpix(2, y, color);
}
#endif

static int8_t floatSign(float value, float threshold) {
  if (value < -threshold)
    return (int8_t)((value + threshold) * 10) - 1;
  if (value > threshold)
    return (int8_t)((value - threshold) * 10) + 1;
  return 0;
}

static void halt(const char *msg) {
#ifdef DIRECT
  leds.ClearTo(RgbColor(0));
  leds.Show();
  leds.~NeoPixelBus();
#else
  M5.dis.clear();
  M5.dis.~LED_Display();
#endif
  Serial.println(msg);
  Serial.flush();
  esp_deep_sleep_start();
}

#ifdef DIRECT
const uint8_t BTN_PIN = 39;
const bool BTN_LEVEL = LOW;

MPU6886 IMU;
#endif
BLEMouse bleMouse;

void setup() {
#ifdef DIRECT
  pinMode(BTN_PIN, BTN_LEVEL ? INPUT : INPUT_PULLUP);
  leds.Begin();
  leds.Show();
  if (IMU.Init())
    halt("MPU6886 not found!");
#else
  M5.begin(false, false, true);
  M5.dis.setBrightness(10);
  if (M5.IMU.Init())
    halt("MPU6886 not found!");
#endif
#ifdef BLE_TASK
  if (! bleMouse.begin())
    halt("Error starting BLE task!");
#else
  bleMouse.begin();
#endif
}

void loop() {
  const float ACCEL_THRESHOLD = 0.20;
#ifdef DIRECT
  const uint32_t SLEEP_TIME = 30000; // 30 sec.
  const uint32_t CLICK_TIME = 20;
  const uint32_t LONGCLICK_TIME = 500;

  static uint32_t lastMoved = 0;
  static uint32_t lastPressed = 0;
#endif

  float accelX, accelY, dummy;
  int8_t x, y;

#ifdef DIRECT
  IMU.getAccelData(&accelX, &accelY, &dummy);
#else
  M5.IMU.getAccelData(&accelX, &accelY, &dummy);
#endif
  x = floatSign(accelX, ACCEL_THRESHOLD);
  y = floatSign(accelY, ACCEL_THRESHOLD);
  drawArrow(x, y);
  if (x || y) {
#ifdef INVERT
    bleMouse.move(-x, -y);
#else
    bleMouse.move(x, y);
#endif
#ifdef DIRECT
    lastMoved = millis();
#endif
  }
#ifdef DIRECT
  if (digitalRead(BTN_PIN) == BTN_LEVEL) { // Button pressed
    if (! lastPressed)
      lastPressed = millis();
    lastMoved = millis();
  } else { // Button released
    if (lastPressed) { // Was pressed
      uint32_t duration = millis() - lastPressed;

      if (duration >= LONGCLICK_TIME) {
        drawClick(false);
        bleMouse.click(MOUSE_RIGHT);
      } else if (duration >= CLICK_TIME) {
        drawClick(true);
        bleMouse.click(MOUSE_LEFT);
      }
      lastPressed = 0;
      lastMoved = millis();
    }
  }
#else
  M5.update();
  if (M5.Btn.wasReleasefor(500)) {
    drawClick(false);
    bleMouse.click(MOUSE_RIGHT);
  } else if (M5.Btn.wasReleasefor(50)) {
    drawClick(true);
    bleMouse.click(MOUSE_LEFT);
  }
#endif
#ifdef DIRECT
  if (millis() - lastMoved >= SLEEP_TIME) {
    leds.ClearTo(RgbColor(0));
    leds.Show();
    leds.~NeoPixelBus();
    Serial.println("Sleeping...");
    Serial.flush();
    esp_sleep_enable_ext0_wakeup((gpio_num_t)BTN_PIN, BTN_LEVEL);
    esp_deep_sleep_start();
  }
#endif
}
