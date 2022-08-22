
int sensorPin = A0;    // select the input pin for the potentiometer

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
  Serial.println(sensorValue);
  delay(1);
}
