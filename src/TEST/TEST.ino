#include <Qbead.h>
#include <internal/QbeadUtils.h>

using namespace Qbead;

Qbead::Qbead bead;

bool test = false;

void setup()
{
    // bead.begin();
    // bead.setBrightness(25);
    // bead.testPixels();
    Serial.begin(9600);
    delay(10000);
    //uint32_t test_seed = getHardwareSeed();
    delay(100);
    //Serial.printf("Hardware Seed: %d\n", test_seed);
    delay(5000);
}

void loop()
{
    if (test)
    {
        Serial.println("WE DID NOT FAIL");
    }
    else
    {
        Serial.println("WE DID FAIL");
    }
    delay(250);
}
