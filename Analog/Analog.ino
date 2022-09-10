uint16_t sensorValue;
int sensorPin = A0;    // select the input pin for the potentiometer

uint32_t sum;
uint8_t n;

void setup() {
  // declare the ledPin as an OUTPUT:
  Serial.begin(115200);

  pinMode(D0, OUTPUT);
  digitalWrite(D0, LOW);
  pinMode(D5, OUTPUT);
  digitalWrite(D5, HIGH);
}

void loop() {
  // read the value from the sensor:
  sensorValue = analogRead(sensorPin);
  sum += sensorValue;
  n++;

//  Serial.println(sensorValue);
  if (n >= 200) {
    Serial.println(sum / n);
    sum = 0;
    n = 0;
  }
  delay(1);
}
