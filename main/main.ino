
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include "time.h"
#include <ESP32Servo.h>

// ========== WIFI / FIREBASE ==========
#define WIFI_SSID "Full house 2.4GHz"
#define WIFI_PASSWORD "motdentam"

#define API_KEY "AIzaSyBgzRsUUpotAw230h8JTthlEx_edM41oqI"
#define DATABASE_URL "https://doan2-77a41-default-rtdb.asia-southeast1.firebasedatabase.app"
#define USER_EMAIL "binhiot0209@gmail.com"
#define USER_PASSWORD "123456"

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 7 * 3600; // +7
const int daylightOffset_sec = 0;

// ========== PINS / CONFIG COLOR SENSOR ==========
#define IR_SENSOR_PIN 34
#define S0 33
#define S1 32
#define S2 25
#define S3 26
#define OUT 35
#define LED_PIN 2

#define DEBOUNCE_SAMPLES 10
#define STABLE_THRESHOLD 7
#define NUM_COLOR_SAMPLES 5
#define MIN_CONFIDENCE 0.6  // ngưỡng tin cậy tối thiểu (0..1)

// ========== SERVO PINS & HOME ==========
Servo servoBase;
Servo servoShoulder;
Servo servoArm;
Servo servoGripper;

#define SERVO_BASE_PIN     13
#define SERVO_SHOULDER_PIN 12
#define SERVO_ARM_PIN      14
#define SERVO_GRIPPER_PIN  27

int baseHome = 0;
int shoulderHome = 0;
int armHome = 0;
int gripperHome = 0;

// ========== COLOR DATA STRUCTS ==========
struct RGB { float r, g, b; };
struct HSV { float h, s, v; };

struct ColorData {
  String name;
  HSV hsv;
  RGB rgb;
  float tolerance;
};

struct ColorResult {
  String name;
  float confidence;
  float distance;
  String method;
};

// màu mẫu (bản mới bạn gửi)
ColorData colors[] = {
  {"Trang",      {254, 0.06, 0.89}, {216.2, 213, 226}, 20.0},
  {"Den",        {0,   0.00, 0.00}, {0,   0,   0},   15.0},
  {"Do",         {0,   1.00, 0.45}, {114, 0,  0},  20.0},
  {"Vang",       {45.1,  0.49, 0.77}, {197, 173, 100},  20.0},
  {"Xanh La",    {115.6, 0.15, 0.69}, {151,  176, 149},  25.0},
  {"Xanh Duong", {240, 1.00, 0.55}, {0,  0,  141}, 25.0},
};
const int NUM_COLORS = sizeof(colors) / sizeof(ColorData);

// ========== IR debounce ==========
int irSamples[DEBOUNCE_SAMPLES];
int irIndex = 0;
int irState = LOW;
int irLastState = LOW;

int readIRStable() {
  irSamples[irIndex] = digitalRead(IR_SENSOR_PIN);
  delay(100);
  irIndex = (irIndex + 1) % DEBOUNCE_SAMPLES;

  int highCount = 0;
  for (int i = 0; i < DEBOUNCE_SAMPLES; i++) {
    if (irSamples[i] == HIGH) highCount++;
  }

  if (highCount >= STABLE_THRESHOLD) return LOW;
  else if (highCount <= (DEBOUNCE_SAMPLES - STABLE_THRESHOLD)) return HIGH;
  return irState;
}

// ========== Đọc màu (TCS3200 style) ==========
RGB readColor() {
  RGB color;

  digitalWrite(S2, LOW); digitalWrite(S3, LOW); delay(50);
  int red = pulseIn(OUT, LOW, 50000);

  digitalWrite(S2, HIGH); digitalWrite(S3, HIGH); delay(50);
  int green = pulseIn(OUT, LOW, 50000);

  digitalWrite(S2, LOW); digitalWrite(S3, HIGH); delay(50);
  int blue = pulseIn(OUT, LOW, 50000);

  color.r = constrain(map(red, 20, 200, 255, 0), 0, 255);
  color.g = constrain(map(green, 20, 200, 255, 0), 0, 255);
  color.b = constrain(map(blue, 20, 200, 255, 0), 0, 255);

  return color;
}

RGB readColorMedian() {
  float rValues[NUM_COLOR_SAMPLES];
  float gValues[NUM_COLOR_SAMPLES];
  float bValues[NUM_COLOR_SAMPLES];

  for (int i = 0; i < NUM_COLOR_SAMPLES; i++) {
    RGB s = readColor();
    rValues[i] = s.r;
    gValues[i] = s.g;
    bValues[i] = s.b;
    delay(50);
  }

  // sort (simple)
  for (int i = 0; i < NUM_COLOR_SAMPLES - 1; i++) {
    for (int j = i + 1; j < NUM_COLOR_SAMPLES; j++) {
      if (rValues[i] > rValues[j]) { float t = rValues[i]; rValues[i] = rValues[j]; rValues[j] = t; }
      if (gValues[i] > gValues[j]) { float t = gValues[i]; gValues[i] = gValues[j]; gValues[j] = t; }
      if (bValues[i] > bValues[j]) { float t = bValues[i]; bValues[i] = bValues[j]; bValues[j] = t; }
    }
  }

  RGB med;
  med.r = rValues[NUM_COLOR_SAMPLES / 2];
  med.g = gValues[NUM_COLOR_SAMPLES / 2];
  med.b = bValues[NUM_COLOR_SAMPLES / 2];
  return med;
}

// ========== RGB -> HSV ==========
HSV rgbToHsv(RGB rgb) {
  HSV hsv;
  float r = rgb.r / 255.0;
  float g = rgb.g / 255.0;
  float b = rgb.b / 255.0;

  float maxC = max(max(r, g), b);
  float minC = min(min(r, g), b);
  float delta = maxC - minC;

  hsv.v = maxC;
  hsv.s = (maxC == 0) ? 0 : delta / maxC;

  if (delta == 0) hsv.h = 0;
  else if (maxC == r) hsv.h = 60.0 * fmod(((g - b) / delta), 6.0);
  else if (maxC == g) hsv.h = 60.0 * (((b - r) / delta) + 2.0);
  else hsv.h = 60.0 * (((r - g) / delta) + 4.0);

  if (hsv.h < 0) hsv.h += 360.0;
  return hsv;
}

// ========== Color matching functions (keeps your algorithms) ==========
ColorResult findColorHSV(HSV hsv) {
  float minDistance = 999999;
  int bestIndex = -1;

  for (int i = 0; i < NUM_COLORS; i++) {
    float hueDiff = abs(hsv.h - colors[i].hsv.h);
    if (hueDiff > 180) hueDiff = 360 - hueDiff;

    float hueWeight = hueDiff / 180.0;
    float satWeight = abs(hsv.s - colors[i].hsv.s);
    float valWeight = abs(hsv.v - colors[i].hsv.v);

    float distance = sqrt(
      pow(hueWeight * 3.0, 2) +
      pow(satWeight * 2.0, 2) +
      pow(valWeight * 1.0, 2)
    );

    if (distance < minDistance) {
      minDistance = distance;
      bestIndex = i;
    }
  }

  ColorResult result;
  if (bestIndex >= 0 && minDistance < colors[bestIndex].tolerance) {
    result.name = colors[bestIndex].name;
    result.distance = minDistance;
    result.confidence = 1.0 - (minDistance / colors[bestIndex].tolerance);
    result.method = "HSV";
  } else {
    result.name = "Khong xac dinh";
    result.confidence = 0;
    result.distance = minDistance;
    result.method = "HSV";
  }
  return result;
}

ColorResult findColorRGB(RGB rgb) {
  float minDistance = 999999;
  int bestIndex = -1;

  for (int i = 0; i < NUM_COLORS; i++) {
    float distance = sqrt(
      pow(rgb.r - colors[i].rgb.r, 2) +
      pow(rgb.g - colors[i].rgb.g, 2) +
      pow(rgb.b - colors[i].rgb.b, 2)
    );
    if (distance < minDistance) { minDistance = distance; bestIndex = i; }
  }

  ColorResult result;
  float maxDistance = 150.0;
  if (bestIndex >= 0 && minDistance < maxDistance) {
    result.name = colors[bestIndex].name;
    result.distance = minDistance;
    result.confidence = 1.0 - (minDistance / maxDistance);
    result.method = "RGB";
  } else {
    result.name = "Khong xac dinh";
    result.confidence = 0;
    result.distance = minDistance;
    result.method = "RGB";
  }
  return result;
}

ColorResult findColorRuleBased(HSV hsv, RGB rgb) {
  ColorResult result;
  result.method = "Rule";
  result.distance = 0;

  if (hsv.v < 0.15) { result.name = "Den"; result.confidence = 0.95; return result; }
  if (hsv.s < 0.15 && hsv.v > 0.75) { result.name = "Trang"; result.confidence = 0.95; return result; }
  // (bỏ rule Xam so va khac neu ban muon)
  if ((hsv.h < 20 || hsv.h > 340) && hsv.s > 0.5 && hsv.v > 0.4) { result.name = "Do"; result.confidence = 0.90; return result; }
  if (hsv.h >= 45 && hsv.h <= 75 && hsv.s > 0.6 && hsv.v > 0.6) { result.name = "Vang"; result.confidence = 0.90; return result; }

  result.name = "Khong xac dinh"; result.confidence = 0; return result;
}

ColorResult findColorCombined(RGB rgb, HSV hsv) {
  ColorResult hsvResult = findColorHSV(hsv);
  ColorResult rgbResult = findColorRGB(rgb);
  ColorResult ruleResult = findColorRuleBased(hsv, rgb);

  Serial.println("\n--- Ket qua tu tung phuong phap ---");
  Serial.printf("HSV:  %s (confidence: %.2f)\n", hsvResult.name.c_str(), hsvResult.confidence);
  Serial.printf("RGB:  %s (confidence: %.2f)\n", rgbResult.name.c_str(), rgbResult.confidence);
  Serial.printf("Rule: %s (confidence: %.2f)\n", ruleResult.name.c_str(), ruleResult.confidence);

  if (ruleResult.confidence >= 0.85) {
    Serial.println("=> Chon: Rule-based");
    return ruleResult;
  }

  if (hsvResult.confidence > rgbResult.confidence) {
    if (hsvResult.confidence >= MIN_CONFIDENCE) { Serial.println("=> Chon: HSV"); return hsvResult; }
  } else {
    if (rgbResult.confidence >= MIN_CONFIDENCE) { Serial.println("=> Chon: RGB"); return rgbResult; }
  }

  if (hsvResult.confidence > rgbResult.confidence) { Serial.println("=> Chon: HSV (mac du confidence thap)"); return hsvResult; }
  else { Serial.println("=> Chon: RGB (mac du confidence thap)"); return rgbResult; }
}

// ========== time / firebase helpers ==========
String getCurrentTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return "Unknown";
  char buffer[30];
  strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
  return String(buffer);
}
String getFirebasePath() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return "/color_log/unknown";
  char timePath[40];
  strftime(timePath, sizeof(timePath), "/color_log/%Y%m%d_%H%M%S", &timeinfo);
  return String(timePath);
}

// ========== SERVO HELPERS & GAP SEQUENCE ==========
void MoveServo(Servo &sv, int pin, int ang, unsigned long wait_ms = 300) {
  sv.attach(pin, 500, 2400);
  sv.write(ang);
  delay(wait_ms);
  sv.detach();
}

void ServoBase0(int ang)     { MoveServo(servoBase, SERVO_BASE_PIN, ang, 400); }
void ServoShoulder0(int ang) { MoveServo(servoShoulder, SERVO_SHOULDER_PIN, ang, 300); }
void ServoArm0(int ang)      { MoveServo(servoArm, SERVO_ARM_PIN, ang, 300); }
// Gripper: attach then detach after sequence for a cleaner open/close
void ServoGripper0(int ang)  { servoGripper.attach(SERVO_GRIPPER_PIN, 500, 2400); servoGripper.write(ang); delay(300); }

void GapVat(int gocBase, const String &mau) {        
  // sequence: approach - grab - lift - rotate base - place - release
  ServoShoulder0(90); delay(1000);
  ServoArm0(70);      delay(1000);
  ServoGripper0(95);  delay(1000);   // close gripper (grab)

  ServoShoulder0(0); delay(1000);
  ServoArm0(0);      delay(1000);

  ServoBase0(gocBase); delay(1000);

  ServoArm0(70);       delay(1000);
  ServoShoulder0(90);  delay(1000);

  ServoGripper0(0);    delay(1000);  // open gripper (release)
  servoGripper.detach();    
              // ensure detached
  ServoShoulder0(0);   delay(1000);

  ServoArm0(0);        delay(1000);
  
  ServoBase0(0);       delay(1000);

  Serial.print("Servo di chuyen: GAP - ");
  Serial.println(mau);
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);

  // WiFi + Firebase
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Dang ket noi WiFi");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\n✅ Da ket noi WiFi");

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  Serial.println("✅ Da ket noi Firebase");

  // Pins
  pinMode(IR_SENSOR_PIN, INPUT);
  pinMode(S0, OUTPUT); pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT); pinMode(S3, OUTPUT);
  pinMode(OUT, INPUT);
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(S0, HIGH); digitalWrite(S1, LOW); // frequency scale for TCS3200

  for (int i = 0; i < DEBOUNCE_SAMPLES; i++) irSamples[i] = LOW;

  // servo frequency
  servoBase.setPeriodHertz(50);
  servoShoulder.setPeriodHertz(50);
  servoArm.setPeriodHertz(50);
  servoGripper.setPeriodHertz(50);

  // move to HOME (use safe moves)
  ServoBase0(baseHome);
  ServoShoulder0(shoulderHome);
  ServoArm0(armHome);
  ServoGripper0(gripperHome);
  servoGripper.detach();

  Serial.println("San sang!");
}

// ========== MAIN LOOP ==========
void loop() {
  irState = readIRStable();

  // rising edge: object arrived
  if (irState == HIGH && irLastState == LOW) {
    digitalWrite(LED_PIN, HIGH);
    Serial.println("\n>>> VAT THE PHAT HIEN! Dang quet mau...");

    // small firebase marker (detected)
    if (Firebase.ready() && WiFi.status() == WL_CONNECTED) {
      String curPath = "/device/detect";
      FirebaseJson js;
      js.set("detected", true);
      js.set("time", getCurrentTime());
      if (Firebase.RTDB.setJSON(&fbdo, curPath, &js)) Serial.println("✅ Firebase: detected = true");
      else Serial.printf("❌ Firebase error: %s\n", fbdo.errorReason().c_str());
    }

    delay(200);

    // read color
    RGB rgb = readColorMedian();
    HSV hsv = rgbToHsv(rgb);
    ColorResult result = findColorCombined(rgb, hsv);

    // print
    Serial.println("\n========================================");
    Serial.printf("RGB: R=%d, G=%d, B=%d\n", (int)rgb.r, (int)rgb.g, (int)rgb.b);
    Serial.printf("HSV: H=%.1f, S=%.2f, V=%.2f\n", hsv.h, hsv.s, hsv.v);
    Serial.println("----------------------------------------");
    Serial.printf("MAU: %s\n", result.name.c_str());
    Serial.printf("DO TIN CAY: %.1f%%\n", result.confidence * 100.0);
    Serial.printf("PHUONG PHAP: %s\n", result.method.c_str());

    // firebase: color log
    String currentTime = getCurrentTime();
    String CurrentPath = getFirebasePath();
    if (Firebase.ready() && WiFi.status() == WL_CONNECTED) {
      FirebaseJson json;
      json.set("color", result.name.c_str());
      json.set("time", currentTime);
      if (Firebase.RTDB.setJSON(&fbdo, CurrentPath, &json)) Serial.println("✅ Gửi dữ liệu thành công!");
      else Serial.printf("❌ Lỗi gửi Firebase: %s\n", fbdo.errorReason().c_str());
    }

    // Decide base angle mapping
    int targetBase = 180; // default others
    if (result.confidence < MIN_CONFIDENCE) {
      Serial.println("[!] DO TIN CAY THAP -> CHON: KHAC (180°)");
      targetBase = 180;
    } else {
      String nm = result.name;
      if (nm == "Do") targetBase = 70;
      else if (nm == "Xanh Duong") targetBase = 89;
      else if (nm == "Xanh La") targetBase = 108;
      else if (nm == "Vang") targetBase = 128;
      else targetBase = 180;
    }

    Serial.print("-> Thuc hien gap & quay base den: ");
    Serial.println(targetBase);

    // Execute pick-move-release
    GapVat(targetBase, result.name);

    Serial.println("========================================\n");
    delay(1000); // tránh bùng phát nhiều lần
  }
  // falling edge: object left
  else if (irState == LOW && irLastState == HIGH) {
    digitalWrite(LED_PIN, LOW);
    Serial.println(">>> Vat the da roi di");

    if (Firebase.ready() && WiFi.status() == WL_CONNECTED) {
      String curPath = "/device/detect";
      FirebaseJson js;
      js.set("detected", false);
      js.set("time", getCurrentTime());
      if (Firebase.RTDB.setJSON(&fbdo, curPath, &js)) Serial.println("✅ Firebase: detected = false");
      else Serial.printf("❌ Firebase error: %s\n", fbdo.errorReason().c_str());
    }
  }

  irLastState = irState;
  delay(10);
}
