#include "arduinoFFT.h"
#define BLYNK_TEMPLATE_ID "TMPL5uKc3ii6"
#define BLYNK_DEVICE_NAME "Ford"
#define BLYNK_AUTH_TOKEN "F6ciwDWw9YCAzK2QgiCo6iDwvXun1Txb"

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <Servo.h>

char auth[] = BLYNK_AUTH_TOKEN;

char ssid[] = "HUANG HOUCAFE_2.4GHz";
char pass[] = "GOLDENAGE";

BlynkTimer timer;
Servo myservo;

void myTimerEvent()
{
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  Blynk.virtualWrite(V0, "test");
}
arduinoFFT FFT = arduinoFFT(); /* Create FFT object */
/*
  These values can be changed in order to evaluate the functions
*/
#define CHANNEL A0
const uint16_t samples = 64;           // This value MUST ALWAYS be a power of 2
const double samplingFrequency = 2000; // Hz, must be less than 10000 due to ADC
uint8_t exponent;
uint16_t snoreCount;
unsigned int sampling_period_us;
unsigned long microseconds;

/*
  These are the input and output vectors
  Input vectors receive computed results from FFT
*/
double vReal[samples];
double vImag[samples];

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

bool snoringState;
uint8_t detectIndex, snoringCount, correctPeriodCount;
const uint16_t snoreThreshold = 200;
uint32_t startQuietTime, stopQuietTime, startLoudTime, stopLoudTime;

void setup()
{
  sampling_period_us = round(1000000 * (1.0 / samplingFrequency));
  Serial.begin(115200);

  myservo.attach(D6);
  myservo.write(0);

  Blynk.begin(auth, ssid, pass);

  pinMode(D0, OUTPUT);
  digitalWrite(D0, LOW);
  pinMode(D4, OUTPUT);
  digitalWrite(D4, HIGH);
  pinMode(D5, OUTPUT);
  digitalWrite(D5, HIGH);

  //  while (!Serial)
  //    ;
  //  Serial.println("Ready");
  exponent = FFT.Exponent(samples);
}

void loop()
{
  Blynk.run();

  /*SAMPLING*/
  microseconds = micros();
  for (int i = 0; i < samples; i++)
  {
    vReal[i] = analogRead(CHANNEL);
    vImag[i] = 0;
    while (micros() - microseconds < sampling_period_us)
    {
      // empty loop
    }
    microseconds += sampling_period_us;
  }
  FFT.DCRemoval(vReal, samples);
  /* Print the results of the sampling according to time */
  //  Serial.println("Data:");
  //  PrintVector(vReal, samples, SCL_TIME);
  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD); /* Weigh data */
  //  Serial.println("Weighed data:");
  //  PrintVector(vReal, samples, SCL_TIME);
  FFT.Compute(vReal, vImag, samples, exponent, FFT_FORWARD); /* Compute FFT */
  //  Serial.println("Computed Real values:");
  //  PrintVector(vReal, samples, SCL_INDEX);
  //  Serial.println("Computed Imaginary values:");
  //  PrintVector(vImag, samples, SCL_INDEX);
  FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
  //  Serial.println("Computed magnitudes:");
  //  PrintVector(vReal, (samples >> 1), SCL_FREQUENCY);
  SnoringDetect(ComputeSumPower(vReal, (samples >> 1)));
  Serial.print("\t\tState:" + String(snoringState * 100));
  Serial.println("\t\tCount:" + String(correctPeriodCount * 100));
  Blynk.virtualWrite(V2, snoringState);
  Blynk.virtualWrite(V3, correctPeriodCount);
  //  double x = FFT.MajorPeak(vReal, samples, samplingFrequency);
  //  Serial.println(x, 6); //Print out what frequency is the most dominant.
  //  while(1); /* Run Once */
  delay(50); /* Repeat after delay */
}

uint16_t ComputeSumPower(double *vData, uint16_t bufferSize)
{
  uint16_t sum;

  for (uint16_t i = 0; i < bufferSize; i++)
  {
    sum += vData[i];
  }
  Serial.print("Sound:" + String(sum));
  Blynk.virtualWrite(V1, sum);
  return sum;
}

void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    double abscissa;
    /* Print abscissa value */
    switch (scaleType)
    {
      case SCL_INDEX:
        abscissa = (i * 1.0);
        break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / samplingFrequency);
        break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * samplingFrequency) / samples);
        break;
    }
    //    Serial.print("x:");
    Serial.println(vData[i], 3);
    //        Serial.print(" \t");
  }
  Serial.println();
}

void SnoringDetect(uint16_t sumPower)
{

  if (snoringState)
  { // quiet sound from loud
    if (sumPower <= snoreThreshold - 10)
    {
      detectIndex++;
      if (detectIndex >= 5)
      {
        snoringState = LOW;
        startQuietTime = millis();
        //         calPeriodTime();
      }
    }
    else
    {
      detectIndex = 0;
    }
  }
  else
  { // loud sound from quiet
    if (sumPower >= snoreThreshold + 10)
    {
      detectIndex++;
      if (detectIndex >= 5)
      {
        snoringState = HIGH;
        startLoudTime = millis();
        calPeriodTime();
      }
    }
    else
    {
      detectIndex = 0;
    }
  }
}

void calPeriodTime()
{
  if (!startQuietTime || !startLoudTime)
  {
    Serial.println("not full period");
    correctPeriodCount = 0;
    return;
  }

  int32_t periodTime = startQuietTime - startLoudTime; // normal 20-26 per minute => 2300ms to 3000ms cr. https://www.vibhavadi.com/Health-expert/detail/533
  periodTime = abs(periodTime);
  if (periodTime < 1500 || periodTime > 3300)
  {
    //    Serial.println("Time not in range : " + (String)periodTime);
    Serial.println("Time not in range");
    correctPeriodCount = 0;
    return;
  }

  correctPeriodCount++;
  if (correctPeriodCount >= 5)
  {
    correctPeriodCount = 0;
    snoringAction();
  }
}

void snoringAction()
{
  snoreCount++;
  Serial.println("Snoring Detect!!!! " + String(snoreCount) + " times");

  digitalWrite(D4, LOW);

  if (snoreCount == 1) {
    myservo.write(5);
    delay(100);
    myservo.write(10);
  }
  else if (snoreCount == 2) {
    myservo.write(15);
    delay(100);
    myservo.write(20);
  }
  else if (snoreCount == 3) {
    myservo.write(25);
    delay(100);
    myservo.write(30);
  }
  else if (snoreCount == 4) {
    myservo.write(35);
    delay(100);
    myservo.write(40);
  }
  else if (snoreCount == 5) {
    myservo.write(45);
    delay(100);
    myservo.write(50);
  }
}
