#include <Motoron.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1331.h>
#include <SPI.h>
#include "arduinoFFT.h"

#define SCLK_PIN 13
#define MOSI_PIN 11
#define CS_PIN   10
#define DC_PIN    8
#define RST_PIN   9
#define MIC_PIN   A0
#define IR_PIN    A1
#define ENCODER_A_M2 2
#define ENCODER_B_M2 5
#define ENCODER_A_M1 3
#define ENCODER_B_M1 12
#define SAMPLES         256
#define SAMPLING_FREQ   10000
#define SOFT_GAIN       12.0
#define NOISE_THRESHOLD 9500
#define TARGET_MIN      350
#define TARGET_MAX      600
#define BALL_COLOR 0x0019
#define OBS_COLOR  0xFFFF
#define BLACK      0x0000
#define RED        0xF800

float vReal[SAMPLES];
float vImag[SAMPLES];
ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, SAMPLES, SAMPLING_FREQ);
Adafruit_SSD1331 display = Adafruit_SSD1331(CS_PIN, DC_PIN, MOSI_PIN, SCLK_PIN, RST_PIN);
bool isAwake = false;
float ballY = 32, velocity = 0, gravity = 0.4;
int ballX = 25, ballR = 3;
int obsX = 96, obsW = 8, gapY = 25, gapH = 22;
int score = 0;

MotoronI2C mc;

volatile long encoderCountM2 = 0;
volatile int lastA_M2 = 0;
volatile long encoderCountM1 = 0;
volatile int lastA_M1 = 0;

enum State { BELT, ROTATING, WAITING_IR, PLAYING, GAME_OVER, RETRACTING_MOTOR, RETRACTING_BELT };
State currentState = BELT;
unsigned long gameOverTime = 0;

void encoderISR_M2() {
  int currentA = digitalRead(ENCODER_A_M2);
  int currentB = digitalRead(ENCODER_B_M2);
  if (currentA != lastA_M2) {
    if (currentB != currentA) encoderCountM2++;
    else encoderCountM2--;
  }
  lastA_M2 = currentA;
}

void encoderISR_M1() {
  int currentA = digitalRead(ENCODER_A_M1);
  int currentB = digitalRead(ENCODER_B_M1);
  if (currentA != lastA_M1) {
    if (currentB != currentA) encoderCountM1++;
    else encoderCountM1--;
  }
  lastA_M1 = currentA;
}

long readQTR(int pin) {
  pinMode(pin, OUTPUT); digitalWrite(pin, HIGH); delayMicroseconds(10);
  pinMode(pin, INPUT);
  long start = micros();
  while (digitalRead(pin) == HIGH && (micros() - start) < 3000);
  return micros() - start;
}

void handleGameOver() {
  display.fillScreen(BLACK);
  display.setCursor(20, 25);
  display.setTextColor(0xFFFF);
  display.print("GAME OVER");
  display.setCursor(10, 40);
  display.print("Score: ");
  display.print(score);
  gameOverTime = millis();
  currentState = GAME_OVER;
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  display.begin();
  display.fillScreen(BLACK);
  randomSeed(analogRead(A1));
  Wire.begin();
  mc.reinitialize();
  mc.disableCrc();
  mc.clearResetFlag();
  mc.clearMotorFault(1);
  mc.clearMotorFault(2);
  mc.setMaxAcceleration(1, 200);
  mc.setMaxDeceleration(1, 200);
  mc.setMaxAcceleration(2, 200);
  mc.setMaxDeceleration(2, 200);

  pinMode(ENCODER_A_M2, INPUT_PULLUP);
  pinMode(ENCODER_B_M2, INPUT_PULLUP);
  lastA_M2 = digitalRead(ENCODER_A_M2);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_M2), encoderISR_M2, CHANGE);
  pinMode(ENCODER_A_M1, INPUT_PULLUP);
  pinMode(ENCODER_B_M1, INPUT_PULLUP);
  lastA_M1 = digitalRead(ENCODER_A_M1);
  attachInterrupt(digitalPinToInterrupt(3), encoderISR_M1, CHANGE);

  Serial.println("M1 starting delivering...");
  encoderCountM1 = 0;
  mc.setSpeed(1, 400);
  currentState = BELT;
}

void loop() {
  if (currentState == BELT) {
    Serial.print("M1 encoder: ");
    Serial.println(encoderCountM1);
    if (abs(encoderCountM1) >= 553) {
      mc.setSpeed(1, 0);
      Serial.println("M1 finished. Strating rotate 150°...");
      delay(500);
      encoderCountM2 = 0;
      mc.setSpeed(2, 400);
      currentState = ROTATING;
    }
  } else if (currentState == ROTATING) {
    const int TICKS_PER_REV = 384;
    long targetTicks = (long)(TICKS_PER_REV * 150.0 / 360.0);
    if (abs(encoderCountM2) >= targetTicks) {
      mc.setSpeed(2, 0);
      Serial.println("M2 finished.");
      display.fillScreen(BLACK);
      currentState = WAITING_IR;
    }
  } else if (currentState == WAITING_IR) {
    if (readQTR(IR_PIN) < 800) {
      Serial.println("Game start");
      display.fillScreen(BLACK);
      score = 0;
      ballY = 32; velocity = 0; obsX = 96;
      currentState = PLAYING;
    } else {
      display.fillScreen(BLACK);
      display.setCursor(15, 25);
      display.setTextColor(0xFFFF);
      display.print("TOUCH TO START");
      delay(100);
    }
  } else if (currentState == PLAYING) {
    float sum = 0;
    for (int i = 0; i < SAMPLES; i++) {
      int raw = analogRead(MIC_PIN);
      vReal[i] = raw; sum += raw; vImag[i] = 0;
      delayMicroseconds(100);
    }
    float avg = sum / SAMPLES;
    for (int i = 0; i < SAMPLES; i++) vReal[i] = (vReal[i] - avg) * SOFT_GAIN;

    FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.compute(FFT_FORWARD);
    FFT.complexToMagnitude();

    float absoluteMaxMag = 0;
    float absolutePeakFreq = 0;
    for (int i = 2; i < SAMPLES / 2; i++) {
      if (vReal[i] > absoluteMaxMag) {
        absoluteMaxMag = vReal[i];
        absolutePeakFreq = (i * 1.0 * SAMPLING_FREQ) / SAMPLES;
      }
    }
    if (absolutePeakFreq >= TARGET_MIN && absolutePeakFreq <= TARGET_MAX) {
      if (absoluteMaxMag > NOISE_THRESHOLD) {
        velocity = -3.5;
      }
    }

    display.fillCircle(ballX, (int)ballY, ballR, BLACK);
    display.fillRect(obsX, 0, obsW, 64, BLACK);
    velocity += gravity;
    ballY += velocity;
    obsX -= 3;

    if (obsX < -obsW) { obsX = 96; gapY = random(5, 35); score++; }
    if (ballY > 64 - ballR) { ballY = 64 - ballR; velocity = 0; }
    if (ballY < ballR) { ballY = ballR; velocity = 0; }
    if (ballX + ballR > obsX && ballX - ballR < obsX + obsW) {
      if (ballY - ballR < gapY || ballY + ballR > gapY + gapH) {
        display.fillScreen(RED);
        delay(300);
        handleGameOver();
        return;
      }
    }

    display.fillRect(obsX, 0, obsW, gapY, OBS_COLOR);
    display.fillRect(obsX, gapY + gapH, obsW, 64, OBS_COLOR);
    display.fillCircle(ballX, (int)ballY, ballR, BALL_COLOR);
    display.setCursor(0, 0);
    display.setTextColor(0xFFFF);
    display.print("Score: "); display.print(score);

  } else if (currentState == GAME_OVER) {
    if (readQTR(IR_PIN) < 800) {
      Serial.println("Game start");
      display.fillScreen(BLACK);
      score = 0;
      ballY = 32; velocity = 0; obsX = 96;
      currentState = PLAYING;
    } else if (millis() - gameOverTime >= 5000) {
      display.fillScreen(BLACK);
      encoderCountM2 = 0;
      mc.setSpeed(2, -400);
      currentState = RETRACTING_MOTOR;
    }
  } else if (currentState == RETRACTING_MOTOR) {
    const int TICKS_PER_REV = 384;
    long targetTicks = (long)(TICKS_PER_REV * 150.0 / 360.0);
    if (abs(encoderCountM2) >= targetTicks) {
      mc.setSpeed(2, 0);
      delay(500);
      encoderCountM1 = 0;
      mc.setSpeed(1, -400);
      currentState = RETRACTING_BELT;
    }
  } else if (currentState == RETRACTING_BELT) {
    if (abs(encoderCountM1) >= 553) {
      mc.setSpeed(1, 0);
      display.fillScreen(BLACK);
      encoderCountM1 = 0;
      mc.setSpeed(1, 400);
      currentState = BELT;
    }
  }
}