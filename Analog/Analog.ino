uint16_t sensorValue;
int sensorPin = 36;    // select the input pin for the potentiometer

uint32_t sum;
uint8_t n;

void setup() {
  // declare the ledPin as an OUTPUT:
  Serial.begin(115200);

  pinMode(26, OUTPUT);
  digitalWrite(26, LOW);
  pinMode(18, OUTPUT);
  digitalWrite(18, HIGH);
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
