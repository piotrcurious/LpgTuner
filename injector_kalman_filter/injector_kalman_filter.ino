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
float R[M][M]; // Only diagonal used in scalar update

// Physical constants
const float K_PW = 0.8;    // PW gain
const float K_FLOW = 0.5;  // Flow gain factor from PW
const float K_TORQUE = 1.2; // Torque gain from Flow/AFR

// Measurement matrix H
float H[M][N] = {
    {1.0, 0.0, 0.0, 0.0},      // Throttle ~ PW
    {0.0, 0.5, 10.0, 0.0},     // MAP ~ f(AFR, Flow)
    {0.0, 1.0/14.7, 0.0, 0.0}, // Lambda ~ AFR/14.7
    {0.0, 0.0, 0.0, 100.0}     // RPM ~ Torque * factor
};

void setup_kalman() {
    x[0] = 5.0;  // PW
    x[1] = 14.7; // AFR
    x[2] = 0.5;  // Flow
    x[3] = 10.0; // Torque

    for(int i=0; i<N; i++) {
        for(int j=0; j<N; j++) {
            P[i][j] = (i==j) ? 1.0 : 0.0;
            Q[i][j] = (i==j) ? 0.01 : 0.0;
        }
    }
    // Set higher process noise for state transitions that are more dynamic
    Q[2][2] = 0.05; // Flow
    Q[3][3] = 0.1;  // Torque

    for(int i=0; i<M; i++) {
        for(int j=0; j<M; j++) {
            R[i][j] = (i==j) ? 0.1 : 0.0;
        }
    }
}

void predict(float dt) {
    // Prediction step with physical coupling:
    // d(PW)/dt = -K_PW * (PW - target) -> target implicitly from measurements
    // d(Flow)/dt = K_FLOW * PW
    // d(Torque)/dt = K_TORQUE * Flow * AFR / 14.7

    float x_old[N];
    for(int i=0; i<N; i++) x_old[i] = x[i];

    // Simple Euler integration of physics
    // PW and AFR act as random walks (influenced by sensors in update)
    // Flow is driven by PW
    x[2] += (K_FLOW * x_old[0] - 0.2 * x_old[2]) * dt;
    // Torque is driven by Flow and AFR
    x[3] += (K_TORQUE * x_old[2] * (x_old[1]/14.7) - 0.1 * x_old[3]) * dt;

    // Jacobian F = df/dx
    float F[N][N] = {0};
    F[0][0] = 1.0;
    F[1][1] = 1.0;
    F[2][0] = K_FLOW * dt;
    F[2][2] = 1.0 - 0.2 * dt;
    F[3][2] = K_TORQUE * (x_old[1]/14.7) * dt;
    F[3][1] = K_TORQUE * x_old[2] * (1.0/14.7) * dt;
    F[3][3] = 1.0 - 0.1 * dt;

    // P = FPF' + Q*dt
    float FPFt[N][N] = {0};
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            for (int k = 0; k < N; k++) {
                for (int l = 0; l < N; l++) {
                    FPFt[i][j] += F[i][k] * P[k][l] * F[j][l];
                }
            }
        }
    }

    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            P[i][j] = FPFt[i][j] + Q[i][j] * dt;
        }
    }
}

// Sequential Scalar Update leveraging orthogonality of measurement noise
void update(float z[]) {
    for (int i = 0; i < M; i++) {
        float hx = 0;
        for (int j = 0; j < N; j++) {
            hx += H[i][j] * x[j];
        }
        float y = z[i] - hx;

        float PHt[N];
        for (int j = 0; j < N; j++) {
            PHt[j] = 0;
            for (int k = 0; k < N; k++) {
                PHt[j] += P[j][k] * H[i][k];
            }
        }

        float S = R[i][i];
        for (int j = 0; j < N; j++) {
            S += H[i][j] * PHt[j];
        }

        float K[N];
        for (int j = 0; j < N; j++) {
            K[j] = PHt[j] / S;
        }

        for (int j = 0; j < N; j++) {
            x[j] += K[j] * y;
        }

        float newP[N][N];
        for (int r = 0; r < N; r++) {
            for (int c = 0; c < N; c++) {
                float HP_c = 0;
                for (int k = 0; k < N; k++) {
                    HP_c += H[i][k] * P[k][c];
                }
                newP[r][c] = P[r][c] - K[r] * HP_c;
            }
        }
        for (int r = 0; r < N; r++) {
            for (int c = 0; c < N; c++) {
                P[r][c] = newP[r][c];
            }
        }
    }
}

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
    if (dt > 0 && dt < 1.0) { // Sanity check on dt
        predict(dt);
        update(z);
        last_t = now;
    }

    static unsigned long last_p = 0;
    if (now - last_p > 500) {
        Serial.print("PW: "); Serial.print(x[0]);
        Serial.print(" AFR: "); Serial.print(x[1]);
        Serial.print(" Flow: "); Serial.print(x[2]);
        Serial.print(" Torque: "); Serial.println(x[3]);
        last_p = now;
    }
}
#endif
