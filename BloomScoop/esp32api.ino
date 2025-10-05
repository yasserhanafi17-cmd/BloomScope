/* ESP32 GPS + NASA POWER (no PRECTOT) + Local Sensors (digital LDR + digital rain) -> ILI9341 TFT
   - Light value logic reversed (active when digital HIGH)
   - DHT temperature multiplied by 10, humidity multiplied by 3.7 (display & serial)
   - Everything else kept as before (API bloom calculation unchanged)
*/

#include <TinyGPSPlus.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Arduino_JSON.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <time.h>
#include <DHT.h>

// ====== CONFIG ======
const char* ssid = "FERDAOUSSE-SALLE-2";
const char* password = "FOSE2025+";
String apiKey = "brwnblRXfyuHGvUTxxnsbashOqhOcT06O65L7cAl";

// NASA POWER (PRECTOT removed)
const String baseURL = "https://power.larc.nasa.gov/api/temporal/daily/point";
const String parameters = "T2M,T2M_MAX,T2M_MIN,RH2M,WS2M";
const String formatStr = "JSON";

// ====== SENSOR SECTION ======
#define DHTPIN 19
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// Digital pins for sensors (digital modules that give HIGH/LOW)
const int LIGHT_PIN = 22;   // Digital light sensor output -> connect the digital out here
const int RAIN_PIN  = 21;   // Digital rain sensor output  -> connect the digital out here

// ====== GPS ======
TinyGPSPlus gps;
#define GPS_SERIAL Serial2
const int GPS_RX_PIN = 16; // GPS TX -> ESP32 RX2 (GPIO16)
const int GPS_TX_PIN = 17; // GPS RX -> ESP32 TX2 (GPIO17)
const uint32_t GPS_BAUD = 9600;

// ====== TFT ======
#define TFT_CS   15
#define TFT_DC   4
#define TFT_RST  2
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

// update interval (ms)
const unsigned long UPDATE_INTERVAL = 10000UL; // 10 seconds
unsigned long lastUpdate = 0;

// ====== utility ======
String getDateNDaysAgo(int daysBack) {
  time_t now;
  struct tm timeinfo;
  time(&now);
  now -= daysBack * 24 * 60 * 60;
  localtime_r(&now, &timeinfo);
  char buf[9];
  sprintf(buf, "%04d%02d%02d", timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday);
  return String(buf);
}

bool valueValid(double v) {
  // POWER uses -999 for missing; also guard NaN
  return !isnan(v) && (v != -999.0);
}

// Bloom Likelihood calculation using API data ONLY (T2M, RH2M, WS2M)
// PRECTOT excluded by request.
float computeBloomLikelihood(double t2m, double rh2m, double ws2m) {
  float tScore = 0.0;
  if (!isnan(t2m)) {
    if (t2m <= 18.0) tScore = constrain((t2m - 5.0) / (18.0 - 5.0), 0.0, 1.0);
    else if (t2m >= 28.0) tScore = constrain((35.0 - t2m) / (35.0 - 28.0), 0.0, 1.0);
    else tScore = 1.0;
  }

  float hScore = 0.0;
  if (!isnan(rh2m)) {
    if (rh2m < 40.0) hScore = constrain((rh2m) / 40.0, 0.0, 1.0);
    else if (rh2m > 70.0) hScore = constrain((100.0 - rh2m) / (100.0 - 70.0), 0.0, 1.0);
    else hScore = 1.0;
  }

  float wScore = 0.0;
  if (!isnan(ws2m)) {
    wScore = constrain((5.0 - ws2m) / 5.0, 0.0, 1.0);
  }

  const float wT = 0.55;   // temperature
  const float wH = 0.30;   // humidity
  const float wW = 0.15;   // wind

  float score = tScore * wT + hScore * wH + wScore * wW;
  return score * 100.0; // percentage
}

// ====== TFT layout helpers ======
void tftHeader() {
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(8, 4);
  tft.print(" Bloom Monitor");
  tft.setTextSize(1);
  tft.setCursor(8, 28);
  tft.setTextColor(ILI9341_CYAN);
  tft.print("API: NASA POWER (no PRECTOT)   Local: DHT11 + digital sensors");
  // draw separator lines
  tft.drawFastHLine(0, 40, tft.width(), ILI9341_WHITE);
  // left block and right block vertical divider
  tft.drawFastVLine(tft.width() / 2, 40, tft.height() - 40, ILI9341_DARKGREY);
}

// Draw the left column (GPS + NASA data)
void tftDrawLeft(double lat, double lon, String date,
                 double t2m, double tmax, double tmin, double rh, double ws) {
  int x = 4;
  int y = 48;
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setCursor(x, y);
  tft.print("API DATA");
  y += 20;

  tft.setTextSize(1);
  tft.setTextColor(ILI9341_CYAN);
  tft.setCursor(x, y); tft.print("Date: "); tft.setTextColor(ILI9341_WHITE); tft.print(date);
  y += 12;
  tft.setTextColor(ILI9341_CYAN);
  tft.setCursor(x, y); tft.print("GPS: ");
  tft.setTextColor(ILI9341_WHITE); tft.print(String(lat, 5)); tft.print(", "); tft.print(String(lon,5));
  y += 12;

  tft.setTextColor(ILI9341_CYAN); tft.setCursor(x, y); tft.print("T: ");
  tft.setTextColor(ILI9341_WHITE); tft.print(valueValid(t2m) ? String(t2m,1) + " C" : "N/A");
  y += 12;
  tft.setTextColor(ILI9341_CYAN); tft.setCursor(x, y); tft.print("Tmax: ");
  tft.setTextColor(ILI9341_WHITE); tft.print(valueValid(tmax) ? String(tmax,1)+" C" : "N/A");
  y += 12;
  tft.setTextColor(ILI9341_CYAN); tft.setCursor(x, y); tft.print("Tmin: ");
  tft.setTextColor(ILI9341_WHITE); tft.print(valueValid(tmin) ? String(tmin,1)+" C" : "N/A");
  y += 12;

  tft.setTextColor(ILI9341_CYAN); tft.setCursor(x, y); tft.print("RH: ");
  tft.setTextColor(ILI9341_WHITE); tft.print(valueValid(rh) ? String(rh,0)+" %" : "N/A");
  y += 12;

  tft.setTextColor(ILI9341_CYAN); tft.setCursor(x, y); tft.print("Wind: ");
  tft.setTextColor(ILI9341_WHITE); tft.print(valueValid(ws) ? String(ws,1)+" m/s" : "N/A");
}

// Draw the right column (local sensors) — applies multipliers here
void tftDrawRight(float localT_raw, float localH_raw, bool rainWet, bool lightBright) {
  // apply requested multipliers
  float localT = NAN;
  float localH = NAN;
  if (!isnan(localT_raw)) localT = localT_raw * 13.0f;
  if (!isnan(localH_raw)) localH = localH_raw * 3.5f;

  int x = tft.width() / 2 + 8;
  int y = 48;
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setCursor(x, y);
  tft.print("LOCAL");
  y += 20;

  tft.setTextSize(1);
  tft.setTextColor(ILI9341_MAGENTA);
  tft.setCursor(x, y); tft.print("DHT11:");
  tft.setTextColor(ILI9341_WHITE);
  if (!isnan(localT) && !isnan(localH)) {
    tft.print(String(localT,1) + " (T)  ");
    tft.print(String(localH,1) + " (H)");
  } else {
    tft.print("N/A");
  }

  y += 14;
  tft.setTextColor(ILI9341_MAGENTA); tft.setCursor(x, y); tft.print("Rain: ");
  tft.setTextColor(rainWet ? ILI9341_GREEN : ILI9341_WHITE);
  tft.print(rainWet ? "WET" : "DRY");

  y += 14;
  tft.setTextColor(ILI9341_MAGENTA); tft.setCursor(x, y); tft.print("Light: ");
  tft.setTextColor(lightBright ? ILI9341_YELLOW : ILI9341_WHITE);
  tft.print(lightBright ? "DARK" : "BRIGHT" );
}

// Draw big bloom status at bottom
void tftDrawBloom(float percent) {
  int y = tft.height() - 56;
  tft.fillRect(0, y - 4, tft.width(), 56, ILI9341_BLACK); // clear bottom area
  tft.drawFastHLine(0, y - 6, tft.width(), ILI9341_WHITE);

  tft.setTextSize(3);
  int centerX = tft.width() / 2 - 40;
  tft.setCursor(centerX-40, y);
  if (!isnan(percent)) {
    if (percent > 70.0) {
      tft.setTextColor(ILI9341_GREEN);
      tft.print("Bloom Likely");
    } else if (percent > 30.0) {
      tft.setTextColor(ILI9341_YELLOW);
      tft.print(" Growing");
    } else {
      tft.setTextColor(ILI9341_RED);
      tft.print(" Dormant");
    }
    tft.setTextSize(2);
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(centerX + 4, y + 28);
    tft.print("Score: ");
    tft.print(percent, 0);
    tft.print("%");
  } else {
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(2);
    tft.setCursor(centerX - 40, y + 8);
    tft.print("Bloom: N/A");
  }
}

// ====== SETUP ======
void setup() {
  Serial.begin(115200);
  delay(200);

  // GPS Serial2
  GPS_SERIAL.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  // sensors
  dht.begin();

  // Configure digital sensor pins - use pullups to avoid floating
  pinMode(LIGHT_PIN, INPUT_PULLUP);
  pinMode(RAIN_PIN, INPUT_PULLUP);
  // NOTE: INPUT_PULLUP means the pin will normally read HIGH.
  // Previously code assumed LOW = active; user asked to REVERSE light logic,
  // so we treat HIGH as active = BRIGHT (i.e., reversed compared to earlier).
  // If your module behaves differently, invert checks below.

  // TFT
  tft.begin();
  tft.setRotation(1);
  tftHeader();
  tft.setCursor(8, 44);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setTextSize(2);
  tft.println("Waiting for GPS fix...");
  Serial.println("Waiting for GPS fix...");

  // WiFi connect early (but we won't call API until GPS fix)
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 15000) {
    delay(300);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected. IP: " + WiFi.localIP().toString());
  } else {
    Serial.println("\nWiFi not connected (continuing, will retry on demand).");
  }

  // NTP time sync (helps date computation)
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  delay(500);
}

// ====== LOOP ======
void loop() {
  // feed GPS
  while (GPS_SERIAL.available() > 0) gps.encode(GPS_SERIAL.read());

  // if no fix yet, show waiting (but keep looping)
  if (!gps.location.isValid()) {
    static unsigned long lastMsg = 0;
    if (millis() - lastMsg > 3000) {
      Serial.println("Waiting for GPS fix...");
      tft.fillRect(0, 40, tft.width(), 40, ILI9341_BLACK);
      tft.setCursor(8, 44);
      tft.setTextColor(ILI9341_YELLOW);
      tft.setTextSize(2);
      tft.println("Waiting for GPS fix...");
      lastMsg = millis();
    }
    delay(100);
    return;
  }

  if (millis() - lastUpdate < UPDATE_INTERVAL) return;
  lastUpdate = millis();

  // Read GPS coords
  double lat = gps.location.lat();
  double lon = gps.location.lng();
  Serial.printf("GPS fix: lat=%.6f lon=%.6f sats=%d\n", lat, lon, gps.satellites.value());

  // ==== READ LOCAL SENSORS (digital) ====
  float localT_raw = dht.readTemperature();
  float localH_raw = dht.readHumidity();

  // REVERSED light logic per your request:
  // Treat HIGH as active -> BRIGHT (reversed compared to previous code)
  bool lightBright = (digitalRead(LIGHT_PIN) == HIGH);
  // Rain remains: LOW -> WET (module commonly pulls low when wet)
  bool rainWet = (digitalRead(RAIN_PIN) == LOW);

  // compute adjusted values for logging/display
  float localT_adj = NAN;
  float localH_adj = NAN;
  if (!isnan(localT_raw)) localT_adj = localT_raw * 10.0f;
  if (!isnan(localH_raw)) localH_adj = localH_raw * 3.7f;

  Serial.printf("Local Sensors -> DHT raw T=%.1fC H=%.0f%%  -> adjusted T=%.1f H=%.1f  Rain=%s Light=%s\n",
                localT_raw, localH_raw,
                isnan(localT_adj) ? NAN : localT_adj,
                isnan(localH_adj) ? NAN : localH_adj,
                rainWet ? "WET" : "DRY", lightBright ? "BRIGHT" : "DARK");

  // Ensure WiFi is connected; try to reconnect if not
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected. Attempting reconnect...");
    WiFi.disconnect();
    WiFi.begin(ssid, password);
    unsigned long st = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - st < 10000) { delay(200); }
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi still not connected. Showing local GPS + sensors only.");
      tftHeader();
      tftDrawLeft(lat, lon, getDateNDaysAgo(1), NAN, NAN, NAN, NAN, NAN);
      tftDrawRight(localT_raw, localH_raw, rainWet, lightBright);
      tftDrawBloom(NAN);
      tft.setCursor(8, 160);
      tft.setTextColor(ILI9341_RED);
      tft.println("WiFi offline - API skipped");
      return;
    }
  }

  // Query NASA POWER: try daysBack 1..7 to find valid values
  int daysBack = 1;
  bool found = false;
  double t2m= NAN, tmax=NAN, tmin=NAN, rh=NAN, ws=NAN;
  String usedDate = "";
  while (!found && daysBack <= 7) {
    String date = getDateNDaysAgo(daysBack);
    String url = baseURL + "?parameters=" + parameters +
                 "&community=RE" +
                 "&longitude=" + String(lon, 6) +
                 "&latitude=" + String(lat, 6) +
                 "&start=" + date +
                 "&end=" + date +
                 "&format=" + formatStr +
                 "&apikey=" + apiKey;

    Serial.println("Request: " + url);

    HTTPClient http;
    http.begin(url);
    int code = http.GET();
    if (code == HTTP_CODE_OK) {
      String payload = http.getString();
      JSONVar root = JSON.parse(payload);
      if (JSON.typeof(root) != "undefined") {
        JSONVar p = root["properties"]["parameter"];
        double tmpT2M = (double)p["T2M"][date];
        if (valueValid(tmpT2M)) {
          t2m  = (double)p["T2M"][date];
          tmax = (double)p["T2M_MAX"][date];
          tmin = (double)p["T2M_MIN"][date];
          rh   = (double)p["RH2M"][date];
          ws   = (double)p["WS2M"][date];
          usedDate = date;
          found = true;
        } else {
          Serial.println("Data invalid for date " + date + ", trying previous day...");
        }
      } else {
        Serial.println("JSON parse failed for date " + date);
      }
    } else {
      Serial.printf("HTTP error %d for date %s\n", code, date.c_str());
    }
    http.end();
    if (!found) daysBack++;
  }

  if (!found) {
    Serial.println("No valid NASA data found (last 7 days). Showing GPS + local sensors only.");
    tftHeader();
    tftDrawLeft(lat, lon, getDateNDaysAgo(1), NAN, NAN, NAN, NAN, NAN);
    tftDrawRight(localT_raw, localH_raw, rainWet, lightBright);
    tftDrawBloom(NAN);
    tft.setCursor(8, 160);
    tft.setTextColor(ILI9341_RED);
    tft.println("No NASA data (7d)");
    return;
  }

  // compute bloom likelihood using API values only (PRECTOT excluded)
  float bloomScore = computeBloomLikelihood(t2m, rh, ws);

  // Display everything (API + local sensors visible)
  tftHeader();
  tftDrawLeft(lat, lon, usedDate, t2m, tmax, tmin, rh, ws);
  tftDrawRight(localT_raw, localH_raw, rainWet, lightBright); // multipliers applied inside
  tftDrawBloom(bloomScore);

  // Also print to Serial
  Serial.println("=== NASA DATA ===");
  Serial.printf("Date %s  T=%.2f Tmax=%.2f Tmin=%.2f RH=%.1f WS=%.2f\n",
                usedDate.c_str(), t2m, tmax, tmin, rh, ws);
  Serial.printf("Bloom Likelihood (API only): %.1f%%\n", bloomScore);
}
