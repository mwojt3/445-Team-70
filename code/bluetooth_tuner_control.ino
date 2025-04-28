#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <arduinoFFT.h>
#include <ESP32Servo.h>

BLECharacteristic *pCharacteristic;
bool newNoteReceived = false;
String selectedNote = "";
bool detectMode = false;

constexpr uint8_t micPin = 32;
constexpr uint16_t SAMPLES = 128;
constexpr float FS = 2000.0;
constexpr int actuatorPin = 26;
constexpr int servoPin = 27;
constexpr int greenLED = 13;
constexpr int redLED = 14;


Servo myServo;
bool tuned = false;
int turnTime = 3000;
String prevDirection = "";

float vReal[SAMPLES];
float vImag[SAMPLES];
ArduinoFFT<float> FFT(vReal, vImag, SAMPLES, FS);

struct Note {
  const char* name;
  float frequency;
};

const Note notes[] = {
  {"D3", 146.83},
  {"E3", 164.81},
  {"F3", 174.61},
  {"G3", 196.00},
  {"A3", 220.00},
  {"B3", 246.94},
  {"C4", 261.63},
  {"D4", 293.66},
  {"E4", 329.63},
  {"F4", 349.23},
  {"G4", 392.00},
  {"A4", 440.00},
  {"B4", 493.88},
  {"C5", 523.25},
  {"D5", 587.33},
  {"E5", 659.25},
  {"F5", 698.46},
  {"G5", 783.99}
};



float lookupFrequency(String noteName) {
  for (int i = 0; i < sizeof(notes) / sizeof(notes[0]); i++) {
    if (noteName == String(notes[i].name)) {
      return notes[i].frequency;
    }
  }
  return -1.0; 
}

float readMicrophoneFrequency() {
  float sum = 0;
  for (uint16_t i = 0; i < SAMPLES; ++i) {
    uint32_t t0 = micros();
    vReal[i] = analogRead(micPin);
    sum += vReal[i];
    while (micros() - t0 < (1e6 / FS));
  }

  float dc = sum / SAMPLES;
  for (uint16_t i = 0; i < SAMPLES; ++i) {
    vReal[i] -= dc;
    vImag[i] = 0;
  }

  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  return FFT.majorPeak();
}

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    selectedNote = pCharacteristic->getValue();
    if (selectedNote.length() > 0) {
      if (selectedNote == "DETECT") {
        detectMode = true;  
      } else {
        newNoteReceived = true;
      }
    }
  }
};


void setup() {
  Serial.begin(115200);
  delay(5000);

  //actuator pin setup
  pinMode(actuatorPin, OUTPUT);
  digitalWrite(actuatorPin, LOW);

  //led setup
  pinMode(greenLED, OUTPUT);
  pinMode(redLED, OUTPUT);
  digitalWrite(greenLED, HIGH);
  digitalWrite(redLED, HIGH);

  myServo.setPeriodHertz(50);
  myServo.attach(servoPin, 1000, 2000);

  BLEDevice::init("Drum Tuner");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService("0000ffe0-0000-1000-8000-00805f9b34fb");

  pCharacteristic = pService->createCharacteristic(
    "0000ffe1-0000-1000-8000-00805f9b34fb",
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY
);

  pCharacteristic->addDescriptor(new BLE2902());

  pCharacteristic->setCallbacks(new MyCallbacks());
  pCharacteristic->setValue("Hello");

  pService->start();
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
  Serial.println("Drum Tuner Bluetooth Activated...");
}

void loop() {
  if (newNoteReceived) {
    digitalWrite(redLED,HIGH);
    digitalWrite(greenLED,LOW);
    turnTime = 4000;
    tuned = false;
    prevDirection = "";

    Serial.println("Bluetooth Message Received...");
    Serial.print("Selected Note To Tune To: ");
    Serial.println(selectedNote);

  while(tuned == false){
    //Strike 
    Serial.println("Striking Drum...");
    digitalWrite(actuatorPin, HIGH);
    delay(250);
    digitalWrite(actuatorPin, LOW);

    //Mic
    Serial.println("Reading Microphone...");
    float actualFrequency = readMicrophoneFrequency();

    
    float targetFrequency = lookupFrequency(selectedNote);
    Serial.print("Target Frequency: ");
    Serial.println(targetFrequency);
    Serial.print("Actual Frequency: ");
    Serial.println(actualFrequency);
    String message = "Target: " + String(targetFrequency, 2) + " Hz | Actual: " + String(actualFrequency, 2) + " Hz";
    pCharacteristic->setValue(message.c_str());
    pCharacteristic->notify();

    if (abs(targetFrequency - actualFrequency) <= 5){
      digitalWrite(redLED,LOW);
      digitalWrite(greenLED,HIGH);
      tuned = true;
      break;
    }

    // 4. Decide how to turn
    if (targetFrequency > 0 && actualFrequency > 0) {

      if (actualFrequency < targetFrequency) {
        if(prevDirection == "CCW"){
          turnTime = turnTime / 2;
        }
        Serial.println("Turning Clockwise (Tighten)");
        prevDirection = "CW";
        myServo.writeMicroseconds(1800);
        delay(turnTime); 
        }
      else if (actualFrequency > targetFrequency) {
        if(prevDirection == "CW"){
          turnTime = turnTime / 2;
        }
        Serial.println("Turning Counterclockwise (Loosen)");
        prevDirection = "CCW";
        myServo.writeMicroseconds(1350);
        delay(turnTime); 
      }
    } else {
      Serial.println("Error: Invalid Target or Actual Frequency");
    }

    // 5. Stop Servo
    Serial.println("Stopping Servo...");
    myServo.writeMicroseconds(1500);
    delay(3000);
  }
    newNoteReceived = false;
  }
  if(detectMode){
    Serial.println("Striking Drum...");
    digitalWrite(actuatorPin, HIGH);
    delay(250);
    digitalWrite(actuatorPin, LOW);

    Serial.println("Reading Microphone...");
    float actualFrequency = readMicrophoneFrequency();

    float targetFrequency = lookupFrequency(selectedNote);

    Serial.print("Target Frequency: ");
    Serial.println(targetFrequency);
    Serial.print("Actual Frequency: ");
    Serial.println(actualFrequency);
    String message = "Target: " + String(targetFrequency, 2) + " Hz | Actual: " + String(actualFrequency, 2) + " Hz";
    pCharacteristic->setValue(message.c_str());
    pCharacteristic->notify();

    detectMode = false;
  }
}
