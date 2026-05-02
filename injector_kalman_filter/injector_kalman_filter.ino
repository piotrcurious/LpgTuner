// Injector State Estimator using Kalman Filter
// Estimates engine parameters based on sensor inputs.

#ifndef UNIT_TEST
#include <Arduino.h>
#endif

#define N 4 // State: [PulseWidth, AFR, FuelFlow, Torque]
#define M 4 // Measurements: [Throttle, MAP, Lambda, RPM]

float x[N];
float P[N][N];
float Q[N][N];
float R[M][M];
float K[N][M];

void setup_kalman() {
    x[0] = 5.0; // PW
    x[1] = 14.7; // AFR
    x[2] = 0.5; // Flow
    x[3] = 10.0; // Torque

    for(int i=0; i<N; i++) {
        for(int j=0; j<N; j++) {
            P[i][j] = (i==j) ? 1.0 : 0.0;
            Q[i][j] = (i==j) ? 0.01 : 0.0;
        }
    }
    for(int i=0; i<M; i++) {
        for(int j=0; j<M; j++) {
            R[i][j] = (i==j) ? 0.1 : 0.0;
        }
    }
}

void predict(float dt) {
    // Identity transition (random walk)
    for(int i=0; i<N; i++) {
        for(int j=0; j<N; j++) {
            P[i][j] += Q[i][j] * dt;
        }
    }
}

void update(float z[]) {
    // Simplified Kalman Update
    // Residual y = z - Hx (H is identity here)
    float y[M];
    for(int i=0; i<M; i++) y[i] = z[i] - x[i];

    // S = HPH' + R = P + R
    float S[M][M];
    for(int i=0; i<M; i++) {
        for(int j=0; j<M; j++) {
            S[i][j] = P[i][j] + R[i][j];
        }
    }

    // Invert S (simple 4x4 diagonal inversion as approximation for now)
    // In a real vector KF, we'd use Gauss-Jordan or similar.
    float Sinv[M][M];
    for(int i=0; i<M; i++) {
        for(int j=0; j<M; j++) {
            if (i == j) Sinv[i][j] = 1.0 / S[i][j];
            else Sinv[i][j] = 0;
        }
    }

    // K = P * H' * Sinv = P * Sinv
    for(int i=0; i<N; i++) {
        for(int j=0; j<M; j++) {
            K[i][j] = 0;
            for(int k=0; k<M; k++) {
                K[i][j] += P[i][k] * Sinv[k][j];
            }
        }
    }

    // x = x + Ky
    float dx[N];
    for(int i=0; i<N; i++) {
        dx[i] = 0;
        for(int j=0; j<M; j++) dx[i] += K[i][j] * y[j];
    }
    for(int i=0; i<N; i++) x[i] += dx[i];

    // P = (I - KH)P
    float IKH[N][N];
    for(int i=0; i<N; i++) {
        for(int j=0; j<N; j++) {
            float kh = K[i][j]; // H is identity
            IKH[i][j] = (i==j ? 1.0 : 0.0) - kh;
        }
    }

    float newP[N][N];
    for(int i=0; i<N; i++) {
        for(int j=0; j<N; j++) {
            newP[i][j] = 0;
            for(int k=0; k<N; k++) newP[i][j] += IKH[i][k] * P[k][j];
        }
    }
    for(int i=0; i<N; i++) {
        for(int j=0; j<N; j++) P[i][j] = newP[i][j];
    }
}

// Rest of the code...
#ifndef UNIT_TEST
const int throttlePin = A0;
const int mapPin = A1;
const int lambdaPin = A2;
const int camPin = 2;

volatile unsigned long lastCamTime = 0;
volatile unsigned long camPeriod = 0;

void camISR() {
    unsigned long now = micros();
    if (lastCamTime > 0) camPeriod = now - lastCamTime;
    lastCamTime = now;
}

void setup() {
    Serial.begin(115200);
    pinMode(camPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(camPin), camISR, RISING);
    setup_kalman();
}

void loop() {
    float z[M];
    z[0] = analogRead(throttlePin) / 102.3;
    z[1] = analogRead(mapPin) / 10.0;
    z[2] = analogRead(lambdaPin) / 1023.0 + 0.5;
    if (camPeriod > 0) z[3] = 60000000.0 / camPeriod;
    else z[3] = 0;

    static unsigned long last_t = 0;
    unsigned long now = millis();
    float dt = (now - last_t) / 1000.0;
    if (dt > 0) {
        predict(dt);
        update(z);
        last_t = now;
    }

    static unsigned long last_p = 0;
    if (now - last_p > 500) {
        Serial.print("PW: "); Serial.print(x[0]);
        Serial.print(" AFR: "); Serial.println(x[1]);
        last_p = now;
    }
}
#endif
