#include <Arduino.h>
#include <math.h>

#define TRIG_PIN 18
#define ECHO_PIN 5

// -------- Settings --------
const int   SAMPLE_HZ = 20;
const int   WINDOW_SEC = 3;
const int   N = SAMPLE_HZ * WINDOW_SEC;

const float MOVE_THRESHOLD_CM = 2.0f;
const float OUTLIER_JUMP_CM   = 15.0f;
const unsigned long ECHO_TIMEOUT_US = 50000; // чуть больше, меньше invalid

// Optional: for data collection (not required for prediction)
const char* label = "object";

// -------- Ultrasonic read (median of 5) --------
float readDistanceCm() {
  float values[5];
  int count = 0;

  for (int i = 0; i < 5; i++) {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    unsigned long duration = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
    if (duration > 0) {
      float d = (duration * 0.0343f) / 2.0f;
      if (d >= 2.0f && d <= 400.0f) values[count++] = d;
    }
    delay(5);
  }

  if (count == 0) return -1.0f;

  // sort
  for (int i = 0; i < count - 1; i++) {
    for (int j = i + 1; j < count; j++) {
      if (values[j] < values[i]) {
        float t = values[i];
        values[i] = values[j];
        values[j] = t;
      }
    }
  }

  return values[count / 2]; // median
}

// -------- Your trained Decision Tree --------
// class 0 = object, class 1 = human
int predictClass(float mean, float std, float max_delta, float moving_ratio) {
  (void)mean; // mean is not used in your tree, but keep for completeness

  if (moving_ratio <= 0.29f) {
    if (max_delta <= 3.25f) {
      return 0; // object
    } else {
      if (max_delta <= 6.39f) {
        return 1; // human
      } else {
        return 0; // object
      }
    }
  } else { // moving_ratio > 0.29
    if (std <= 3.69f) {
      return 1; // human
    } else {
      if (moving_ratio <= 0.40f) {
        return 0; // object
      } else {
        return 0; // object
      }
    }
  }
}

// simple confidence (heuristic, optional)
float predictConfidence(int cls, float std, float max_delta, float moving_ratio) {
  // Not true ML probability, just a helpful score 0..1
  float score = 0.5f;

  if (cls == 1) { // human
    score += 0.25f * fminf(moving_ratio / 0.5f, 1.0f);
    score += 0.25f * fminf(std / 4.0f, 1.0f);
  } else { // object
    score += 0.25f * (1.0f - fminf(moving_ratio / 0.5f, 1.0f));
    score += 0.25f * (1.0f - fminf(max_delta / 10.0f, 1.0f));
  }

  if (score < 0.0f) score = 0.0f;
  if (score > 1.0f) score = 1.0f;
  return score;
}

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  Serial.println("TinyML Ultrasonic (ESP32 + HC-SR04)");
  Serial.println("Commands (optional): 'h' set label human, 'o' set label object");
  Serial.println("Output: features + PREDICT result");
  Serial.println();
}

void loop() {
  // optional label switch (if you still want to log labeled data)
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r' || c == ' ') continue;

    if (c == 'h' || c == 'H') { label = "human"; Serial.println("# label -> HUMAN"); }
    if (c == 'o' || c == 'O') { label = "object"; Serial.println("# label -> OBJECT"); }
  }

  float samples[N];
  int valid = 0;

  float prev = -1.0f;

  // collect window
  for (int i = 0; i < N; i++) {
    float d = readDistanceCm();

    // outlier filter
    if (d > 0.0f && prev > 0.0f && fabsf(d - prev) > OUTLIER_JUMP_CM) {
      d = -1.0f;
    }

    if (d > 0.0f) {
      samples[valid++] = d;
      prev = d;
    }

    delay(1000 / SAMPLE_HZ);
  }

  if (valid < 15) {
    Serial.println("# too many invalid samples (try 20-150cm, better angle, slower movement)");
    delay(300);
    return;
  }

  // mean
  float sum = 0.0f;
  for (int i = 0; i < valid; i++) sum += samples[i];
  float mean = sum / (float)valid;

  // std
  float var = 0.0f;
  for (int i = 0; i < valid; i++) {
    float diff = samples[i] - mean;
    var += diff * diff;
  }
  float std = sqrtf(var / (float)valid);

  // max_delta + moving_ratio
  float max_delta = 0.0f;
  int moves = 0;
  for (int i = 1; i < valid; i++) {
    float delta = fabsf(samples[i] - samples[i - 1]);
    if (delta > max_delta) max_delta = delta;
    if (delta > MOVE_THRESHOLD_CM) moves++;
  }
  float moving_ratio = (valid > 1) ? ((float)moves / (float)(valid - 1)) : 0.0f;

  // prediction
  int cls = predictClass(mean, std, max_delta, moving_ratio);
  float conf = predictConfidence(cls, std, max_delta, moving_ratio);

  // print
  Serial.print("label="); Serial.print(label);
  Serial.print(" | mean="); Serial.print(mean, 2);
  Serial.print(" std="); Serial.print(std, 3);
  Serial.print(" max_delta="); Serial.print(max_delta, 2);
  Serial.print(" moving_ratio="); Serial.print(moving_ratio, 3);

  Serial.print("  =>  PREDICT: ");
  Serial.print(cls == 1 ? "HUMAN" : "OBJECT");
  Serial.print("  (conf~");
  Serial.print(conf, 2);
  Serial.println(")");

  delay(300);
}
