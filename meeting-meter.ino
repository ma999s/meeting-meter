#include <RotaryEncoder.h>

const int shiftDataPin = 7;
const int shiftLatchPin = 6;
const int shiftClockPin = 5;

const int rotDataPin = 3; // interrupt
const int rotSwitchPin = 4;
const int rotClockPin = 2; // interrupt

RotaryEncoder rotEnc = RotaryEncoder(rotDataPin, rotClockPin, 5);

unsigned long lastRotClicked = 0; // software debounce

const int D1 = 8;
const int D2 = 9;
const int D3 = 10;
const int D4 = 11;

const int clearAmountPin = 12;
const int startStopPin = 0;

unsigned long lastStartStop = 0; // software debounce
boolean prevStartStopIsDown = false;

const float secsPerHour = 3600;

int rate = 200;
const int rateMax = 995;
const int rateIncrement = 5; // inc is also min

int parts = 18;
const int partsMax = 32;
const int partsIncrement = 1; // inc is also min

boolean isRunning = false;
const int runningLEDPin = 1;
unsigned long prevMicros = 0; // sync the updates

float amount = 0;
float amountDelta = 0;

byte table[] = {0xd7, 0x14, 0xcd, 0x5d, 0x1e, 0x5b, 0xdb, 0x15, 0xdf, 0x5f, 0xa3, 0xaf, 0x00};

const int delayMS = 5;
int blinkCount = 0;

volatile int dmode = 0; // 0 = RUN, 1 = SET RATE, 2 = SET PARTICIPANTS

void setup() {
  
  digitalWrite(LED_BUILTIN, LOW);
  
  pinMode(rotSwitchPin, INPUT);
  digitalWrite(rotSwitchPin, HIGH); // pull-up Rotary Switch
  
  attachInterrupt(digitalPinToInterrupt(rotDataPin), rotSample, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rotClockPin), rotSample, CHANGE);
  
  pinMode(shiftDataPin, OUTPUT);
  pinMode(shiftClockPin, OUTPUT);
  pinMode(shiftLatchPin, OUTPUT);
  
  pinMode(D1, OUTPUT);
  pinMode(D2, OUTPUT);
  pinMode(D3, OUTPUT);
  pinMode(D4, OUTPUT);

  pinMode(clearAmountPin, INPUT_PULLUP);
  pinMode(startStopPin, INPUT_PULLUP);

  pinMode(runningLEDPin, OUTPUT);
}

void rotClicked() {
  
  if ((micros() - lastRotClicked) < 500000) return;
  lastRotClicked = micros();
  
  dmode++;
  if (dmode > 2) dmode = 0;
}

void updateRate(boolean increase) {
  
  if (increase) {

    int newValue = rate + rateIncrement;
    if (newValue <= rateMax) rate = newValue;
    
  } else {
    
    if (rate >= (rateIncrement + rateIncrement)) rate = rate - rateIncrement;
  }
}

void updateParts(boolean increase) {
  
  if (increase) {

    int newValue = parts + partsIncrement;
    if (newValue <= partsMax) parts = newValue;
    
  } else {
    
    if (parts >= (partsIncrement + partsIncrement)) parts = parts - partsIncrement;
  }
}

void show(byte code, int digit) {

  unselectDigit();
  writeCharacter(code);
  selectDigit(digit);
  delay(delayMS);
}

void selectDigit(int digitSelected) {

  digitalWrite(D1, (digitSelected != 0));
  digitalWrite(D2, (digitSelected != 1));
  digitalWrite(D3, (digitSelected != 2));
  digitalWrite(D4, (digitSelected != 3));
}

void unselectDigit() {

  digitalWrite(D1, HIGH);
  digitalWrite(D2, HIGH);
  digitalWrite(D3, HIGH);
  digitalWrite(D4, HIGH);
}

void writeCharacter(byte code) {

  digitalWrite(shiftLatchPin, LOW);
  shiftOut(shiftDataPin, shiftClockPin, LSBFIRST, code);
  digitalWrite(shiftLatchPin, HIGH);
}

void loop() {

  if (digitalRead(rotSwitchPin) == LOW) rotClicked();
  
  readClearAmount();
  readStartStop();
  calculateAmount();
  
  refreshDisplay();
}

void rotSample(){

  int rotStatus = rotEnc.sampleRotationStatus();
  if (rotStatus != 0) processRotTurn(rotStatus);
}

void readClearAmount() {

  if (digitalRead(clearAmountPin) == LOW) {
    amount = 0;
  }
}

void readStartStop() {

  if ((micros() - lastStartStop) < 50000) return;
  lastStartStop = micros();

  if (digitalRead(startStopPin) == LOW) {

    if (prevStartStopIsDown) return;
    
    isRunning = !(isRunning);
    prevStartStopIsDown = true;
    
  } else {

    prevStartStopIsDown = false;
  }

  digitalWrite(runningLEDPin, (isRunning ? HIGH : LOW));
}

void calculateAmount() {

  if (!isRunning) return;
  if ((micros() - prevMicros) < 1000000) return;

  prevMicros = micros();
  
  amountDelta = ((float(rate) * float(parts)) / secsPerHour);
  amount = amount + amountDelta;

  if (amount > 9999) amount -= 10000;
}

void processRotTurn(int rotationStatus) {

  switch (dmode) {

    case 0:         // Run - ignore Rot encoder turns
        break;

    case 1:         // set Rate - CW increase
        updateRate(rotationStatus > 0);
        break;

    case 2:         // set Participants - CW increase
        updateParts(rotationStatus > 0);
        break;
  }
}

void refreshDisplay() {

  byte ch1; 
  byte ch2;
  byte ch3;
  byte ch4;

  unsigned int thousands = 0;
  unsigned int hundreds = 0;
  unsigned int tens = 0;
  unsigned singles = 0;
  
  switch (dmode) {

      case 0:
              thousands = int(amount / 1000);
              hundreds = int((amount - (1000 * thousands)) / 100);
              tens = int((amount - (1000 * thousands) - (100 * hundreds)) / 10);
              singles = int((amount - (1000 * thousands) - (100 * hundreds) - (10 * tens)));
              
              ch1 = (thousands == 0) ? table[12] : table[thousands];
              ch2 = (thousands == 0 && hundreds == 0) ? table[12] : table[hundreds];
              ch3 = (thousands == 0 && hundreds == 0 && tens == 0) ? table[12] : table[tens];
              ch4 = table[singles];
              break;
              
      case 1:
              hundreds = int(rate / 100);
              tens = int((rate - (100 * hundreds)) / 10);
              singles = int((rate - (100 * hundreds) - (10 * tens)));
              
              ch1 = table[10];
              ch2 = (blinkCount > 29) ? (table[12]) : (table[hundreds]);
              ch3 = (blinkCount > 29) ? (table[12]) : (table[tens]);
              ch4 = (blinkCount > 29) ? (table[12]) : (table[singles]);
              break;
              
      case 2:
              hundreds = int(parts / 100);
              tens = int((parts - (100 * hundreds)) / 10);
              singles = int((parts - (100 * hundreds) - (10 * tens)));
              
              ch1 = table[11];
              ch2 = (blinkCount > 29) ? (table[12]) : (table[hundreds]);
              ch3 = (blinkCount > 29) ? (table[12]) : (table[tens]);
              ch4 = (blinkCount > 29) ? (table[12]) : (table[singles]);
              break;
  }

  blinkCount++;
  if (blinkCount > 49) blinkCount = 0;
  
  show(ch1, 0);
  show(ch2, 1);
  show(ch3, 2);
  show(ch4, 3);
}
