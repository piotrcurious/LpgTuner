// Injector State Estimator using Kalman Filter
// Estimates engine parameters based on sensor inputs.

#ifndef UNIT_TEST
#include <Arduino.h>
#endif

#define N 4 // State: [PulseWidth, AFR, FuelFlow, Torque]
#define M 4 // Measurements: [Throttle, MAP, Lambda, RPM]

float x[N];
float P[N][N];
float Q_base[N][N];
float Q[N][N];
float R[M][M];

// Adaptive Q parameters
float innovation_sq_ema[M] = {0, 0, 0, 0};
const float alpha_ema = 0.5;

// Physical constants
const float K_PW = 0.8;
const float K_FLOW = 0.5;
const float K_TORQUE = 1.2;

// H matrix will be calculated dynamically in update()
float H[M][N];

void setup_kalman() {
    x[0] = 5.0;  // PW
    x[1] = 14.7; // AFR
    x[2] = 0.5;  // Flow
    x[3] = 10.0; // Torque

    for(int i=0; i<N; i++) {
        for(int j=0; j<N; j++) {
            P[i][j] = (i==j) ? 100.0 : 0.0;
            Q_base[i][j] = (i==j) ? 0.01 : 0.0;
            Q[i][j] = Q_base[i][j];
        }
    }

    Q_base[2][2] = 0.05;
    Q_base[3][3] = 0.1;

    for(int i=0; i<M; i++) {
        for(int j=0; j<M; j++) {
            R[i][j] = (i==j) ? 0.01 : 0.0; // Restored low R for faster convergence in tests
        }
        innovation_sq_ema[i] = R[i][i];
    }
}

void predict(float dt) {
    float x_old[N];
    for(int i=0; i<N; i++) x_old[i] = x[i];

    x[2] += (K_FLOW * x_old[0] - 0.2 * x_old[2]) * dt;
    x[3] += (K_TORQUE * x_old[2] * (14.7 / x_old[1]) - 0.1 * x_old[3]) * dt;

    float F[N][N] = {0};
    F[0][0] = 1.0;
    F[1][1] = 1.0;
    F[2][0] = K_FLOW * dt;
    F[2][2] = 1.0 - 0.2 * dt;
    F[3][2] = K_TORQUE * (14.7 / x_old[1]) * dt;
    F[3][1] = -K_TORQUE * x_old[2] * (14.7 / (x_old[1] * x_old[1])) * dt;
    F[3][3] = 1.0 - 0.1 * dt;

    float FPFt[N][N] = {0};
    for (int i = 0; i < N; i++) {
        for (int k = 0; k < N; k++) {
            if (F[i][k] == 0) continue;
            for (int l = 0; l < N; l++) {
                if (P[k][l] == 0) continue;
                float F_ik_P_kl = F[i][k] * P[k][l];
                for (int j = 0; j < N; j++) {
                    FPFt[i][j] += F_ik_P_kl * F[j][l];
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

void update_H() {
    H[0][0] = 1.0; H[0][1] = 0.0; H[0][2] = 0.0; H[0][3] = 0.0;
    H[1][0] = 0.0; H[1][1] = 0.5; H[1][2] = 10.0; H[1][3] = 0.0;
    H[2][0] = 0.0; H[2][1] = 1.0/14.7; H[2][2] = 0.0; H[2][3] = 0.0;
    H[3][0] = 0.0; H[3][1] = 0.0; H[3][2] = 0.0; H[3][3] = 100.0;
}

void update(float z[]) {
    update_H();

    for (int i = 0; i < M; i++) {
        float hx = 0;
        for (int j = 0; j < N; j++) {
            hx += H[i][j] * x[j];
        }
        float y = z[i] - hx;

        innovation_sq_ema[i] = (1.0 - alpha_ema) * innovation_sq_ema[i] + alpha_ema * (y * y);

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

        if (y * y > 25.0 * S) continue; // Outlier rejection (5-sigma)

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

    for (int i = 0; i < N; i++) {
        float scale = 1.0;
        if (i == 0) scale = innovation_sq_ema[0] / (R[0][0] + 0.001);
        if (i == 1) scale = innovation_sq_ema[2] / (R[2][2] + 0.001);

        if (scale < 1.0) scale = 1.0;
        if (scale > 1000.0) scale = 1000.0;

        Q[i][i] = Q_base[i][i] * scale;
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
    // Scale ADC inputs to engineering units
    z[0] = analogRead(throttlePin) / 102.3;      // 0-10 %
    z[1] = analogRead(mapPin) / 102.3 * 100.0;  // 0-100 kPa
    z[2] = analogRead(lambdaPin) / 1023.0 + 0.5; // 0.5-1.5 Lambda
    if (camPeriod > 0) z[3] = 60000000.0 / camPeriod; // RPM
    else z[3] = 0;

    static unsigned long last_t = 0;
    unsigned long now = millis();
    float dt = (now - last_t) / 1000.0;
    if (dt > 0 && dt < 1.0) {
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
