/*

  Example of use of the FFT libray to compute FFT for a signal sampled through the ADC.
        Copyright (C) 2018 Enrique Condés and Ragnar Ranøyen Homb

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

/*
   https://www.instructables.com/Snore-O-Meter/
   the Journal of Sleep Research has published a paper called "How to measure snoring? A comparison of the microphone, cannula and piezoelectric sensor" in 2015 (ref. J Sleep Res. (2016) 25, 158–168) that indicates that snoring sounds of people from Iceland have a fundamental frequency range mainly in the 70 - 160 Hz range.
   https://onlinelibrary.wiley.com/doi/pdf/10.1111/jsr.12356

   https://www.researchgate.net/publication/234090531_Energy_Types_of_Snoring_Sounds_in_Patients_with_Obstructive_Sleep_Apnea_Syndrome_A_Preliminary_Observation#pf4
*/

#include "arduinoFFT.h"

arduinoFFT FFT = arduinoFFT(); /* Create FFT object */
/*
  These values can be changed in order to evaluate the functions
*/
#define CHANNEL 36
const uint16_t samples = 256; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 1000; //Hz, must be less than 10000 due to ADC

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

void setup()
{
  sampling_period_us = round(1000000 * (1.0 / samplingFrequency));
  Serial.begin(115200);

  pinMode(26, OUTPUT);
  digitalWrite(26, LOW);
  pinMode(18, OUTPUT);
  digitalWrite(18, HIGH);

  while (!Serial);
  Serial.println("Ready");
}

void loop()
{
  /*SAMPLING*/
  microseconds = micros();
  for (int i = 0; i < samples; i++)
  {
    vReal[i] = analogRead(CHANNEL) - 2048;
    vImag[i] = 0;
    while (micros() - microseconds < sampling_period_us) {
      //empty loop
    }
    microseconds += sampling_period_us;
  }
  /* Print the results of the sampling according to time */
  //  Serial.println("Data:");
  //  PrintVector(vReal, samples, SCL_TIME);
  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
  //  Serial.println("Weighed data:");
  //  PrintVector(vReal, samples, SCL_TIME);
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
  //  Serial.println("Computed Real values:");
  //  PrintVector(vReal, samples, SCL_INDEX);
  //  Serial.println("Computed Imaginary values:");
  //  PrintVector(vImag, samples, SCL_INDEX);
  FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
  //  Serial.println("Computed magnitudes:");
  PrintVector(vReal, (samples >> 1), SCL_FREQUENCY);
  SnoringDetect(vReal, (samples >> 1));
  //  double x = FFT.MajorPeak(vReal, samples, samplingFrequency);
  //  Serial.println(x, 6); //Print out what frequency is the most dominant.
  //  while(1); /* Run Once */
  delay(300); /* Repeat after delay */
}

void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType)
{
//  for (uint16_t i = 0; i < bufferSize; i++)
//  {
//    double abscissa;
//    /* Print abscissa value */
//    switch (scaleType)
//    {
//      case SCL_INDEX:
//        abscissa = (i * 1.0);
//        break;
//      case SCL_TIME:
//        abscissa = ((i * 1.0) / samplingFrequency);
//        break;
//      case SCL_FREQUENCY:
//        abscissa = ((i * 1.0 * samplingFrequency) / samples);
//        break;
//    }
//    Serial.print(abscissa, 3);
//    if (scaleType == SCL_FREQUENCY)
//      Serial.print("Hz\t");
//  }
  //    Serial.println();
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
//    Serial.println(vData[i], 3);
    //        Serial.print(" \t");
  }
//  Serial.println();

}

bool SnoringDetect(double *vData, uint16_t bufferSize) {
  uint8_t Snoring = 0;

  // detect peak
  if (vData[7] + vData[8] > 1000) {
    Snoring++;
  }
  if (vData[15] + vData[16] + vData[17] > 1000) {
    Snoring++;
  }
  if (vData[30] + vData[31] > 1000) {
    Snoring++;
  }
  if (vData[39] + vData[40] > 400) {
    Snoring++;
  }

  // detect min
  if (vData[20] + vData[21] + vData[22] < 800) {
    Snoring++;
  }
  if (vData[36]  < 100) {
    Snoring++;
  }
  if (vData[48]  < 200) {
    Snoring++;
  }

  // print score
  Serial.println("Snoring Score : " + (String)Snoring);

  if (Snoring >= 5)
    return true;
  else
    return false;
}
