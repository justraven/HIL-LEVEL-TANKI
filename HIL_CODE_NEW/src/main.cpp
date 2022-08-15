#include <parsing.h>

float KP, KI, KD, KP_, KI_, KD_;
float setPoint, level, controlSignal; //distance is feedback sensor
float error = 0.00, totalError, lastError, deltaError;
float maxControl = 30;
float minControl = 0;

unsigned long currentPeriod;
int period = 1; // delay 1 ms

void setup() {

  Serial.begin(9600);

}

void loop() {

  if (Serial.available() > 0) {

    READ_DATA_UNTIL('\n');
    data.replace(',','.');
    parseString();

    setPoint = DATA_STR(0).toFloat();
    level    = DATA_STR(1).toFloat();
    KP       = DATA_STR(2).toFloat();
    KI       = DATA_STR(3).toFloat();
    KD       = DATA_STR(4).toFloat();

  }

  if (millis() >= currentPeriod + period) {

    currentPeriod += period;

    error = setPoint - level;
    totalError += error;

    constrain(totalError,minControl,maxControl);

    deltaError = error - lastError;

    KP_ = KP*error;
    KI_ = KI*totalError*period;
    KD_ = (KD/period)*deltaError;

    controlSignal = KP_ + KI_ + KD_;

    constrain(controlSignal,minControl,maxControl);

    lastError = error;

    //-----SEND DATA TO SERIAL------//

    Serial.println(
      "P" + String(controlSignal) +
      "S" + String(error)         +
      "T" + String(level)         +
      "A" + String(KP_)           +
      "B" + String(KI_)           +
      "C" + String(KD_)
      );
  }
}