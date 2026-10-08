// # Tap to Measure
//

// First, let's include the Qbead library and set up a few useful data structures.
#include <internal/BlochVector.h>
#include <internal/QbeadUtils.h>
#include <Qbead.h>

using namespace Qbead;

Qbead::Qbead bead;

BlochVector current_state(90, 0);

// Prepare some colors for the visualization during the game.
uint32_t white = color(255, 255, 255);
uint32_t red = color(255, 0, 0);
uint32_t blue = color(0, 0, 255);

// ## Setup
//
// The setup function is called once when the Qbead is powered on and it is used to initialize the Qbead and set up the game.
void setup() {
  bead.begin();
  bead.setBrightness(25);
  // Test the pixels by flashing a colorful pattern to make sure they are working.
  bead.testPixels();
}

// TODO: Maybe rewrite this to be more in line with how Barna add Serials etc to his
// sketches? I of course copied this from TapToMeasure.
// Combine the two? Have one gesture be the default one?
void loop() {
  static long last_event = 0;
  static uint32_t event_color = white;

  bead.clear();
  bead.readIMU(false);          // must run every loop: it feeds the gravity tracker and shake detector

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

// ## Event loop
//
// The loop function is called repeatedly until the Qbead is powered off.
// It is used to read the IMU and update the current state of the game.
//float delta = 0.0;
//float beta = 0.0;
//bool clean = false;
//bool clean_block = false;
//
//void loop() {
//  static long last_tap = 0;
//  static uint32_t tap_color = white;
//
//  static long last_write = 0;
//
//  // Clear the display.
//  bead.clear();
//  // Read the IMU to get the current gravity direction.
//  if (delta < 0.5)
//  {
//    clean = true;
//    if (millis() - last_write > 100){
//        bead.readIMU(true);
//        last_write = millis();
//    }
//    else {
//        bead.readIMU(false);
//    }
//  }
//  else {
//    clean = false;
//    clean_block = false;
//    bead.readIMU(false);
//  }
//
//  if (clean && !clean_block){
//    for (size_t i = 0; i < 5; i++)
//    {
//        Serial.println();
//    }    
//    Serial.println("trec |  tap   |   X   |   Y   |   Z   |  mag2 | rmag2 | ?     | ?    |  t_acc | p_acc  | ?   |   ?");
//    clean_block = true;
//
//    last_write = millis();
//  }
//
//  if (bead.wasTapped()){
//    last_tap = millis();
//    BlochVector acc_vector(bead.x_whentapped, bead.y_whentapped, bead.z_whentapped);
//
//    float probability = pow(innerProductAbs(current_state, acc_vector),2);
//    float threshold = random(0, 100)/100.0f;
//    float identity_threshold = 0.9;
//    //Serial.print("probability: ");
//    //Serial.println(probability);
//    //Serial.print("threshold: ");
//    //Serial.println(threshold);
//    if (probability > identity_threshold) {
//      tap_color = red;
//      //Serial.println("red identity");
//    } else if (probability < 1 - identity_threshold) {
//      tap_color = blue;
//      //Serial.println("blue identity");
//    } else if (probability > threshold) {
//      current_state = acc_vector;
//      tap_color = red;
//      //Serial.println("red random");
//    } else {
//      current_state = -acc_vector;
//      tap_color = blue;
//      //Serial.println("blue random");
//    }
//  }
//
//  delta = max(min(millis() - last_tap, 2000), 0) / 2000.0;
//  //float delta = max(min(millis() - last_tap, 2000), 0) / 2000.0;
//  uint32_t color = addColor(scaleColor(1-delta, tap_color), scaleColor(delta, white));
//  bead.setBloch_deg(current_state, color);
//
//  // Show the result.
//  bead.show();
//}
void loop() {
  static long last_event = 0;
  static uint32_t event_color = white;

  bead.clear();
  bead.readIMU(false);          // must run every loop: it feeds the gravity tracker and shake detector

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
float delta = 0.0;
float beta = 0.0;
bool clean = false;
bool clean_block = false;


void loop() {
  static long last_tap = 0;
  static uint32_t tap_color = white;

  static long last_write = 0;

  // Clear the display.
  bead.clear();
  // Read the IMU to get the current gravity direction.
  if (delta < 0.5)
  {
    clean = true;
    if (millis() - last_write > 100){
        bead.readIMU(true);
        last_write = millis();
    }
    else {
        bead.readIMU(false);
    }
  }
  else {
    clean = false;
    clean_block = false;
    bead.readIMU(false);
  }

  if (clean && !clean_block){
    for (size_t i = 0; i < 5; i++)
    {
        Serial.println();
    }    
    Serial.println("trec |  tap   |   X   |   Y   |   Z   |  mag2 | rmag2 | ?     | ?    |  t_acc | p_acc  | ?   |   ?");
    clean_block = true;

    last_write = millis();
  }

  if (bead.wasTapped()){
    last_tap = millis();
    BlochVector acc_vector(bead.x_whentapped, bead.y_whentapped, bead.z_whentapped);

    float probability = pow(innerProductAbs(current_state, acc_vector),2);
    float threshold = random(0, 100)/100.0f;
    float identity_threshold = 0.9;
    //Serial.print("probability: ");
    //Serial.println(probability);
    //Serial.print("threshold: ");
    //Serial.println(threshold);
    if (probability > identity_threshold) {
      tap_color = red;
      //Serial.println("red identity");
    } else if (probability < 1 - identity_threshold) {
      tap_color = blue;
      //Serial.println("blue identity");
    } else if (probability > threshold) {
      current_state = acc_vector;
      tap_color = red;
      //Serial.println("red random");
    } else {
      current_state = -acc_vector;
      tap_color = blue;
      //Serial.println("blue random");
    }
  }

  delta = max(min(millis() - last_tap, 2000), 0) / 2000.0;
  //float delta = max(min(millis() - last_tap, 2000), 0) / 2000.0;
  uint32_t color = addColor(scaleColor(1-delta, tap_color), scaleColor(delta, white));
  bead.setBloch_deg(current_state, color);

  // Show the result.
  bead.show();
}
