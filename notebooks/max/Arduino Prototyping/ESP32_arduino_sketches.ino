#include <ESP32Servo.h>
#include <ESP32PWM.h>
#include <arduinoFFT.h>

#define SAMPLES 128             // Must be a power of 2
#define SAMPLING_FREQUENCY 4000 // Hz, adjust to your signal

int micPin = 4; // GPIO4

double vReal[SAMPLES];
double vImag[SAMPLES];

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);           // 12-bit ADC (0–4095)
  analogSetAttenuation(ADC_11db);     // for 0–3.3V range
}

void loop() {
  // Sample
  unsigned long start = micros();
  for (int i = 0; i < SAMPLES; i++) {
    vReal[i] = analogRead(micPin);
    vImag[i] = 0;
    delayMicroseconds(1000000 / SAMPLING_FREQUENCY); // sampling delay
  }
  unsigned long end = micros();
  
  float samplingFreq = (float)SAMPLES * 1e6 / (end - start);
  Serial.print("Actual Fs: ");
  Serial.println(samplingFreq);

  // Run FFT
  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();

  // Find peak frequency
  double peak = FFT.majorPeak();
  Serial.print("Peak Frequency: ");
  Serial.println(peak);

  delay(100);
}
