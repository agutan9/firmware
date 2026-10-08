# 1 "C:\\Users\\TimBr\\AppData\\Local\\Temp\\tmppn4xdf1e"
#include <Arduino.h>
# 1 "C:/Users/TimBr/Study/QIST/SecondYear/Q5PROJECT/Qbead/MDP_repo/src/Shake_to_measure/Shake_to_Measure.ino"




#include <internal/BlochVector.h>
#include <internal/QbeadUtils.h>
#include <Qbead.h>

using namespace Qbead;

Qbead::Qbead bead;

BlochVector current_state(90, 0);


uint32_t white = color(255, 255, 255);
uint32_t red = color(255, 0, 0);
uint32_t blue = color(0, 0, 255);
void setup();
void loop();
#line 23 "C:/Users/TimBr/Study/QIST/SecondYear/Q5PROJECT/Qbead/MDP_repo/src/Shake_to_measure/Shake_to_Measure.ino"
void setup() {
  bead.begin();
  bead.setBrightness(25);

  bead.testPixels();
}




void loop() {
  static long last_event = 0;
  static uint32_t event_color = white;

  bead.clear();
  bead.readIMU(false);

  if (bead.wasShaken()) {
    last_event = millis();
    BlochVector acc_vector(bead.x_whenshaken, bead.y_whenshaken, bead.z_whenshaken);

    float probability = pow(innerProductAbs(current_state, acc_vector), 2);
    float threshold = random(0, 100) / 100.0f;
    float identity_threshold = 0.9;
    Serial.print("probability: ");
    Serial.println(probability);
    Serial.print("threshold: ");
    Serial.println(threshold);
    if (probability > identity_threshold) {
      event_color = red;
      Serial.println("red identity");
    } else if (probability < 1 - identity_threshold) {
      event_color = blue;
      Serial.println("blue identity");
    } else if (probability > threshold) {
      current_state = acc_vector;
      event_color = red;
      Serial.println("red random");
    } else {
      current_state = -acc_vector;
      event_color = blue;
      Serial.println("blue random");
    }
  }

  float delta = max(min(millis() - last_event, 2000), 0) / 2000.0;
  uint32_t color = addColor(scaleColor(1 - delta, event_color), scaleColor(delta, white));
  bead.setBloch_deg(current_state, color);


  bead.show();
}