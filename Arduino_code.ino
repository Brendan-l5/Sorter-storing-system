// --- Pin setup ---
const int buttonPause = 7;    // Red button
const int buttonReady = 6;    // Yellow button
const int buttonResume = 5;   // Green button

const int ledR = 2;
const int ledG = 3;

bool paused = false;
bool ready = false;

void setup() {
  Serial.begin(9600);
  pinMode(buttonPause, INPUT);
  pinMode(buttonReady, INPUT);
  pinMode(buttonResume, INPUT);
  pinMode(ledR, OUTPUT);
  pinMode(ledG, OUTPUT);
  setLED(0, 255);
}

void loop() {
  if (paused == false &&digitalRead(buttonPause) == HIGH) {
    paused = true;
    ready = false;
    Serial.println("PAUSE");
    setLED(255, 0); // Red
    delay(300);
  }

  if (paused && digitalRead(buttonReady) == HIGH) {
    ready = true;
    Serial.println("READY");
    setLED(200, 255); // Yellow
    delay(300);
  }

  if (paused && ready && digitalRead(buttonResume) == HIGH) {
    paused = false;
    ready = false;
    Serial.println("RESUME");
    setLED(0, 255); // Green
    delay(300);
  }

  delay(50);
}

void setLED(int r, int g) {
  analogWrite(ledR, r);
  analogWrite(ledG, g);
}
