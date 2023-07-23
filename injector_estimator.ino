// Define some constants
#define N 4 // Number of state variables
#define M 4 // Number of measurements
#define DT 0.01 // Time step in seconds
#define PI 3.14159 // Pi constant

// Define some parameters (these may need to be adjusted based on experimental data or literature sources)
float K = 0.5; // Injector gain factor
float T = 0.01; // Injector time constant
float A = 14.7; // Stoichiometric air-fuel ratio
float B = 0.8; // Combustion efficiency factor
float C = 0.5; // Fuel density factor
float D = 0.6; // Engine displacement factor

// Define some variables
float x[N]; // State vector
float P[N][N]; // Error covariance matrix
float z[M]; // Measurement vector
float Q[N][N]; // Process noise covariance matrix
float R[M][M]; // Measurement noise covariance matrix
float K[N][M]; // Kalman gain matrix
float y[M]; // Innovation vector
float S[M][M]; // Innovation covariance matrix
float I[N][N]; // Identity matrix

// Define some pins for the sensors and the injector
int throttlePin = A0; // Throttle position sensor pin
int mapPin = A1; // MAP sensor pin
int lambdaPin = A2; // Lambda probe pin
int camPin = 2; // Cam sensor pin
int injectorPin = 3; // Injector pin

// Define some variables for the sensors and the injector
float throttle; // Throttle position in percentage
float map; // MAP in kPa
float lambda; // Lambda value
float rpm; // RPM in revolutions per minute
float pulseWidth; // Pulse width in milliseconds

// Define some variables for the cam sensor interrupt
volatile unsigned long lastCamTime = 0; // Last time the cam sensor triggered in microseconds
volatile unsigned long camPeriod = 0; // Cam sensor period in microseconds

// Define a function to calculate the state transition function f
void f(float x[], float u[], float dt, float xdot[])
{
  // x[0] is the pulse width in milliseconds
  // x[1] is the air-fuel ratio
  // x[2] is the fuel mass flow rate in grams per second
  // x[3] is the engine torque in Newton meters
  
  // u[0] is the throttle position in percentage
  // u[1] is the MAP in kPa
  // u[2] is the lambda value
  
  // dt is the time step in seconds
  
  // xdot is the output vector of the state derivatives
  
  float pw = x[0]; // Pulse width in milliseconds
  float afr = x[1]; // Air-fuel ratio
  float mf = x[2]; // Fuel mass flow rate in grams per second
  float Tq = x[3]; // Engine torque in Newton meters
  
  float tp = u[0]; // Throttle position in percentage
  float mp = u[1]; // MAP in kPa
  float lam = u[2]; // Lambda value
  
  float ma; // Air mass flow rate in grams per second
  
  ma = mp * D / (R * T); // Calculate air mass flow rate using ideal gas law
  
  xdot[0] = -K * (pw - tp); // Calculate pulse width derivative using a first-order model with a gain factor K and a time constant T
  xdot[1] = -K * (afr - A * lam); // Calculate air-fuel ratio derivative using a first-order model with a gain factor K and a time constant T
  xdot[2] = pw * C / (1000 * T); // Calculate fuel mass flow rate derivative using a linear relationship with pulse width and a factor C for fuel density
  xdot[3] = B * mf * afr / ma; // Calculate engine torque derivative using a linear relationship with fuel mass flow rate, air-fuel ratio, and air mass flow rate, and a factor B for combustion efficiency 
}

// Define a function to calculate the measurement function h
void h(float x[], float z[])
{
  // x[0] is the pulse width in milliseconds
  // x[1] is the air-fuel ratio
  // x[2] is the fuel mass flow rate in grams per second
  // x[3] is the engine torque in Newton meters
  
  // z[0] is the throttle position in percentage
  // z[1] is the MAP in kPa
  // z[2] is the lambda value
  // z[3] is the RPM in revolutions per minute
  
  float pw = x[0]; // Pulse width in milliseconds
  float afr = x[1]; // Air-fuel ratio
  float mf = x[2]; // Fuel mass flow rate in grams per second
  float Tq = x[3]; // Engine torque in Newton meters
  
  
  z[0] = pw / T; // Calculate throttle position using a linear relationship with pulse width and a time constant T 
  z[1] = mf * afr / (D * C); // Calculate MAP using a linear relationship with fuel mass flow rate, air-fuel ratio, and a factor D for engine displacement and C for fuel density 
  z[2] = afr / A; // Calculate lambda value using a linear relationship with air-fuel ratio and a factor A for stoichiometric air-fuel ratio 
  z[3] = Tq * 60 / (2 * PI); // Calculate RPM using a linear relationship with engine torque and a factor 2 * PI for angular conversion
}

// Define a function to calculate the Jacobian matrix F of f with respect to x
void F(float x[], float u[], float dt, float F[][N])
{
  // x[0] is the pulse width in milliseconds
  // x[1] is the air-fuel ratio
  // x[2] is the fuel mass flow rate in grams per second
  // x[3] is the engine torque in Newton meters
  
  // u[0] is the throttle position in percentage
  // u[1] is the MAP in kPa
  // u[2] is the lambda value
  
  // dt is the time step in seconds
  
  // F is the output matrix of partial derivatives of f with respect to x
  
  float pw = x[0]; // Pulse width in milliseconds
  float afr = x[1]; // Air-fuel ratio
  float mf = x[2]; // Fuel mass flow rate in grams per second
  float Tq = x[3]; // Engine torque in Newton meters
  
  float tp = u[0]; // Throttle position in percentage
  float mp = u[1]; // MAP in kPa
  float lam = u[2]; // Lambda value
  
  float ma; // Air mass flow rate in grams per second
  
  ma = mp * D / (R * T); // Calculate air mass flow rate using ideal gas law
  
  F[0][0] = -K; // Partial derivative of f[0] with respect to x[0]
  F[0][1] = 0; // Partial derivative of f[0] with respect to x[1]
  F[0][2] = 0; // Partial derivative of f[0] with respect to x[2]
  F[0][3] = 0; // Partial derivative of f[0] with respect to x[3]
  
  F[1][0] = 0; // Partial derivative of f[1] with respect to x[0]
  F[1][1] = -K; // Partial derivative of f[1] with respect to x[1]
  F[1][2] = 0; // Partial derivative of f[1] with respect to x[2]
  F[1][3] = 0; // Partial derivative of f[1] with respect to x[3]
  
  F[2][0] = C / (1000 * T); // Partial derivative of f[2] with respect to x[0]
  F[2][1] = 0; // Partial derivative of f[2] with respect to x[1]
  F[2][2] = 0; // Partial derivative of f[2] with respect to x[2]
  F[2][3] = 0; // Partial derivative of f[2] with respect to x[3]
  
  F[3][0] = 0; // Partial derivative of f[3] with respect to x[0]
  F[3][1] = B * mf / ma; // Partial derivative of f[3] with respect to x[1]
  F[3][2] = B * afr / ma; // Partial derivative of f[3] with respect to x[2]
  F[3][3] = 0; // Partial derivative of f[3] with respect to x[3]
}

// Define a function to calculate the Jacobian matrix H of h with respect to x
void H(float x[], float H[][N])
{
  // x[0] is the pulse width in milliseconds
  // x[1] is the air-fuel ratio
  // x[2] is the fuel mass flow rate in grams per second
  // x[3] is the engine torque in Newton meters
  
  // H is the output matrix of partial derivatives of h with respect to x
  
  float pw = x[0]; // Pulse width in milliseconds
  float afr = x[1]; // Air-fuel ratio
  float mf = x[2]; // Fuel mass flow rate in grams per second
  float Tq = x[3]; // Engine torque in Newton meters
  
  
  H[0][0] = 1 / T; // Partial derivative of h[0] with respect to x[0]
  H[0][1] = 0; // Partial derivative of h[0] with respect to x[1]
  H[0][2] = 0; // Partial derivative of h[0] with respect to x[2]
  H[0][3] = 0; // Partial derivative of h[0] with respect to x[3]
  
  H[1][0] = 0; // Partial derivative of h[1] with respect to x[0]
  H[1][1] = mf / (D * C); // Partial derivative of h[1] with respect to x[1]
  H[1][2] = afr / (D * C); // Partial derivative of h[1] with respect to x[2]
  H[1][3] = 0; // Partial derivative of h[1] with respect to x[3]
  
  H[2][0] = 0; // Partial derivative of h[2] with respect to x[0]
  H[2][1] = 1 / A; // Partial derivative of h[2] with respect to x[1]
  H[2][2] = 0; // Partial derivative of h[2] with respect to x[2]
  H[2][3] = 0; // Partial derivative of h[2] with respect to x[3]
  
  H[3][0] = 0; // Partial derivative of h[3] with respect to x[0]
  H[3][1] = 0; // Partial derivative of h[3] with respect to x[1]
  H[3][2] = 0; // Partial derivative of h[3] with respect to x[2]
  H[3][3] = 60 / (2 * PI); // Partial derivative of h[3] with respect to x[3]
}

// Define a function to initialize the state estimate and its error covariance matrix
void initialize()
{
  // Initialize the state estimate based on some initial values or guesses
  x[0] = 10; // Initial pulse width in milliseconds
  x[1] = 14.7; // Initial air-fuel ratio
  x[2] = 1; // Initial fuel mass flow rate in grams per second
  x[3] = 100; // Initial engine torque in Newton meters
  
  // Initialize the error covariance matrix based on some initial values or guesses
  P[0][0] = 1; // Initial error variance of pulse width in milliseconds squared
  P[0][1] = 0; // Initial error covariance of pulse width and air-fuel ratio in milliseconds times ratio
  P[0][2] = 0; // Initial error covariance of pulse width and fuel mass flow rate in milliseconds times grams per second
  P[0][3] = 0; // Initial error covariance of pulse width and engine torque in milliseconds times Newton meters
  
  P[1][0] = 0; // Initial error covariance of air-fuel ratio and pulse width in ratio times milliseconds
  P[1][1] = 1; // Initial error variance of air-fuel ratio in ratio squared
  P[1][2] = 0; // Initial error covariance of air-fuel ratio and fuel mass flow rate in ratio times grams per second
  P[1][3] = 0; // Initial error covariance of air-fuel ratio and engine torque in ratio times Newton meters
  
  P[2][0] = 0; // Initial error covariance of fuel mass flow rate and pulse width in grams per second times milliseconds
  P[2][1] = 0; // Initial error covariance of fuel mass flow rate and air-fuel ratio in grams per second times ratio
  P[2][2] = 1; // Initial error variance of fuel mass flow rate in grams per second squared
  P[2][3] = 0; // Initial error covariance of fuel mass flow rate and engine torque in grams per second times Newton meters
  
  P[3][0] = 0; // Initial error covariance of engine torque and pulse width in Newton meters times milliseconds
  P[3][1] = 0; // Initial error covariance of engine torque and air-fuel ratio in Newton meters times ratio
  P[3][2] = 0; // Initial error covariance of engine torque and fuel mass flow rate in Newton meters times grams per second
  P[3][3] = 100; // Initial error variance of engine torque in Newton meters squared
  
  // Initialize the process noise covariance matrix based on some values or guesses
  Q[0][0] = 0.01; // Process noise variance of pulse width in milliseconds squared
  Q[0][1] = 0; // Process noise covariance of pulse width and air-fuel ratio in milliseconds times ratio
  Q[0][2] = 0; // Process noise covariance of pulse width and fuel mass flow rate in milliseconds times grams per second
  Q[0][3] = 0; // Process noise covariance of pulse width and engine torque in milliseconds times Newton meters
  
  Q[1][0] = 0; // Process noise covariance of air-fuel ratio and pulse width in ratio times milliseconds
  Q[1][1] = 0.01; // Process noise variance of air-fuel ratio in ratio squared
  Q[1][2] = 0; // Process noise covariance of air-fuel ratio and fuel mass flow rate in ratio times grams per second
  Q[1][3] = 0; // Process noise covariance of air-fuel ratio and engine torque in ratio times Newton meters
  
  
  Q[2][0] = 0; // Process noise covariance of fuel mass flow rate and pulse width in grams per second times milliseconds
  Q[2][1] = 0; // Process noise covariance of fuel mass flow rate and air-fuel ratio in grams per second times ratio
  Q[2][2] = 0.01; // Process noise variance of fuel mass flow rate in grams per second squared
  Q[2][3] = 0; // Process noise covariance of fuel mass flow rate and engine torque in grams per second times Newton meters
  
  Q[3][0] = 0; // Process noise covariance of engine torque and pulse width in Newton meters times milliseconds
  Q[3][1] = 0; // Process noise covariance of engine torque and air-fuel ratio in Newton meters times ratio
  Q[3][2] = 0; // Process noise covariance of engine torque and fuel mass flow rate in Newton meters times grams per second
  Q[3][3] = 10; // Process noise variance of engine torque in Newton meters squared
  
  // Initialize the measurement noise covariance matrix based on some values or guesses
  R[0][0] = 0.1; // Measurement noise variance of throttle position in percentage squared
  R[0][1] = 0; // Measurement noise covariance of throttle position and MAP in percentage times kPa
  R[0][2] = 0; // Measurement noise covariance of throttle position and lambda value in percentage times ratio
  R[0][3] = 0; // Measurement noise covariance of throttle position and RPM in percentage times revolutions per minute
  
  R[1][0] = 0; // Measurement noise covariance of MAP and throttle position in kPa times percentage
  R[1][1] = 1; // Measurement noise variance of MAP in kPa squared
  R[1][2] = 0; // Measurement noise covariance of MAP and lambda value in kPa times ratio
  R[1][3] = 0; // Measurement noise covariance of MAP and RPM in kPa times revolutions per minute
  
   R[2][0] = 0; // Measurement noise covariance of lambda value and throttle position in ratio times percentage
  R[2][1] = 0; // Measurement noise covariance of lambda value and MAP in ratio times kPa
  R[2][2] = 0.01; // Measurement noise variance of lambda value in ratio squared
  R[2][3] = 0; // Measurement noise covariance of lambda value and RPM in ratio times revolutions per minute
  
  R[3][0] = 0; // Measurement noise covariance of RPM and throttle position in revolutions per minute times percentage
  R[3][1] = 0; // Measurement noise covariance of RPM and MAP in revolutions per minute times kPa
  R[3][2] = 0; // Measurement noise covariance of RPM and lambda value in revolutions per minute times ratio
  R[3][3] = 10; // Measurement noise variance of RPM in revolutions per minute squared
  
  // Initialize the identity matrix
  for (int i = 0; i < N; i++)
  {
    for (int j = 0; j < N; j++)
    {
      if (i == j)
      {
        I[i][j] = 1; // Diagonal elements are one
      }
      else
      {
        I[i][j] = 0; // Off-diagonal elements are zero
      }
    }
  }
}

// Define a function to predict the next state estimate and its error covariance matrix
void predict(float x[], float P[][N], float u[], float dt)
{
  float xdot[N]; // State derivative vector
  float F[N][N]; // Jacobian matrix F of f with respect to x
  
  f(x, u, dt, xdot); // Calculate the state derivative vector using the state transition function f
  F(x, u, dt, F); // Calculate the Jacobian matrix F using the function F
  
  for (int i = 0; i < N; i++)
  {
    x[i] = x[i] + xdot[i] * dt; // Update the state estimate using Euler's method
    
    for (int j = 0; j < N; j++)
    {
      P[i][j] = P[i][j] + (F[i][j] * P[i][j] + P[i][j] * F[j][i] + Q[i][j]) * dt; // Update the error covariance matrix using Euler's method
    }
  }
}

// Define a function to update the state estimate and its error covariance matrix using the measurements
void update(float x[], float P[][N], float z[])
{
  
  float H[M][N]; // Jacobian matrix H of h with respect to x
  float y[M]; // Innovation vector
  float S[M][M]; // Innovation covariance matrix
  float K[N][M]; // Kalman gain matrix
  float temp1[N][M]; // Temporary matrix for calculation
  float temp2[M][M]; // Temporary matrix for calculation
  float temp3[M][N]; // Temporary matrix for calculation
  
  h(x, z); // Calculate the measurement vector using the measurement function h
  H(x, H); // Calculate the Jacobian matrix H using the function H
  
  for (int i = 0; i < M; i++)
  {
    y[i] = z[i] - z[i]; // Calculate the innovation vector as the difference between the actual and predicted measurements
    
    for (int j = 0; j < M; j++)
    {
      S[i][j] = R[i][j]; // Initialize the innovation covariance matrix as the measurement noise covariance matrix
      
      for (int k = 0; k < N; k++)
      {
        S[i][j] = S[i][j] + H[i][k] * P[k][j]; // Add the contribution of the error covariance matrix and the Jacobian matrix H to the innovation covariance matrix
      }
    }
  }
  
  // Calculate the inverse of the innovation covariance matrix using Gauss-Jordan elimination method
  for (int i = 0; i < M; i++)
  {
    // Find the pivot element and swap rows if needed
    float max = abs(S[i][i]);
    int maxRow = i;
    
    for (int j = i + 1; j < M; j++)
    {
      if (abs(S[j][i]) > max)
      {
        max = abs(S[j][i]);
        maxRow = j;
      }
    }
    
    if (maxRow != i)
    {
      // Swap rows i and maxRow
      for (int k = 0; k < M; k++)
      {
        float temp = S[i][k];
        S[i][k] = S[maxRow][k];
        S[maxRow][k] = temp;
        
        temp = I[i][k];
        I[i][k] = I[maxRow][k];
        I[maxRow][k] = temp;
      }
    }
    
    // Make the pivot element equal to one by dividing the row by the pivot element
    float pivot = S[i][i];
    
    for (int k = 0; k < M; k++)
    {
      S[i][k] = S[i][k] / pivot;
      I[i][k] = I[i][k] / pivot;
    }
    
    // Make all other elements in the column equal to zero by subtracting multiples of the pivot row from other rows
    for (int j = 0; j < M; j++)
    {
      if (j != i)
      {
        float factor = S[j][i];
        
        for (int k = 0; k < M; k++)
        {
          S[j][k] = S[j][k] - factor * S[i][k];
          I[j][k] = I[j][k] - factor * I[i][k];
        }
      }
    }
  }
  
  // At this point, S is an identity matrix and I is the inverse of the original S
  
  // Calculate the Kalman gain matrix using the formula K = P * H' * S^-1
  for (int i = 0; i < N; i++)
  {
    for (int j = 0; j < M; j++)
    {
      temp1[i][j] = 0; // Initialize a temporary matrix for calculation
      
      for (int k = 0; k < N; k++)
      {
        temp1[i][j] = temp1[i][j] + P[i][k] * H[j][k]; // Multiply P and the transpose of H and store in temp1
      }
      
      K[i][j] = 0; // Initialize the Kalman gain matrix
      
      for (int k = 0; k < M; k++)
      {
        K[i][j] = K[i][j] + temp1[i][k] * I[k][j]; // Multiply temp1 and the inverse of S and store in K
      }
    }
  }
  
  // Update the state estimate using the formula x = x + K * y
  for (int i = 0; i < N; i++)
  {
    x[i] = x[i] + K[i][0] * y[0] + K[i][1] * y[1] + K[i][2] * y[2] + K[i][3] * y[3]; // Update the state estimate using the Kalman gain matrix and the innovation vector
  }
  
  // Update the error covariance matrix using the formula P = (I - K * H) * P
  for (int i = 0; i < N; i++)
  {
    for (int j = 0; j < N; j++)
    {
      temp2[i][j] = 0; // Initialize a temporary matrix for calculation
      
      for (int k = 0; k < M; k++)
      {
        temp2[i][j] = temp2[i][j] + K[i][k] * H[k][j]; // Multiply K and H and store in temp2
      }
      
      temp3[i][j] = I[i][j] - temp2[i][j]; // Subtract temp2 from I and store in temp3
    }
  }
  
  for (int i = 0; i < N; i++)
  {
    for (int j = 0; j < N; j++)
    {
      temp1[i][j] = 0; // Initialize a temporary matrix for calculation
      
      for (int k = 0; k < N; k++)
      {
        temp1[i][j] = temp1[i][j] + temp3[i][k] * P[k][j]; // Multiply temp3 and P and store in temp1
      }
      
      P[i][j] = temp1[i][j]; // Update the error covariance matrix using temp1
    }
  }
}

// Define a function to read the sensors and store the measurements in the measurement vector
void readSensors(float z[])
{
  // Read the throttle position sensor and map it to a percentage value between 0 and 100
  throttle = map(analogRead(throttlePin), 0, 1023, 0, 100);
  
  // Read the MAP sensor and map it to a kPa value between 0 and 100
  map = map(analogRead(mapPin), 0, 1023, 0, 100);
  
  // Read the lambda probe and map it to a ratio value between 0.5 and 1.5
  lambda = map(analogRead(lambdaPin), 0, 1023, 0.5, 1.5);
  
  // Read the cam sensor using an interrupt and calculate the RPM based on the cam period
  attachInterrupt(digitalPinToInterrupt(camPin), camISR, RISING); // Attach an interrupt to the cam sensor pin on the rising edge
  rpm = 60000000 / camPeriod; // Calculate RPM in revolutions per minute based on the cam period in microseconds
  
  // Store the measurements in the measurement vector
  z[0] = throttle; // Throttle position in percentage
  z[1] = map; // MAP in kPa
  z[2] = lambda; // Lambda value
  z[3] = rpm; // RPM in revolutions per minute
}

// Define a function to control the injector using the state estimate of the pulse width
void controlInjector(float x[])
{
  // x[0] is the pulse width in milliseconds
  
  float pw = x[0]; // Pulse width in milliseconds
  
  // Control the injector using PWM with a frequency of 100 Hz and a duty cycle proportional to the pulse width
  analogWrite(injectorPin, map(pw, 0, 10, 0, 255)); // Map the pulse width to a value between 0 and 255 and write it to the injector pin using PWM
}

// Define a function to handle the cam sensor interrupt
void camISR()
{
  unsigned long currentCamTime = micros(); // Get the current time in microseconds
  
  if (lastCamTime > 0) // If this is not the first interrupt
  {
    camPeriod = currentCamTime - lastCamTime; // Calculate the cam period as the difference between the current and last time
  }
  
  lastCamTime = currentCamTime; // Update the last time to the current time
}

// Define a function to print some information to the serial monitor for debugging purposes
void printInfo(float x[], float P[][N], float z[])
{
  // x[0] is the pulse width in milliseconds
  // x[1] is the air-fuel ratio
  // x[2] is the fuel mass flow rate in grams per second
  // x[3] is the engine torque in Newton meters
  
  // P is the error covariance matrix
  
  // z[0] is the throttle position in percentage
  // z[1] is the MAP in kPa
  // z[2] is the lambda value
  // z[3] is the RPM in revolutions per minute
  
  Serial.print("Pulse width: "); // Print the label for pulse width
  Serial.print(x[0]); // Print the state estimate of pulse width
  Serial.print(" +/- "); // Print the plus-minus sign for error
  Serial.println(sqrt(P[0][0])); // Print the square root of the error variance of pulse width
  
  Serial.print("Air-fuel ratio: "); // Print the label for air-fuel ratio
  Serial.print(x[1]); // Print the state estimate of air-fuel ratio
  Serial.print(" +/- "); // Print the plus-minus sign for error
  Serial.println(sqrt(P[1][1])); // Print the square root of the error variance of air-fuel ratio
  
  Serial.print("Fuel mass flow rate: "); // Print the label for fuel mass flow rate
  Serial.print(x[2]); // Print the state estimate of fuel mass flow rate
  Serial.print(" +/- "); // Print the plus-minus sign for error
  Serial.println(sqrt(P[2][2])); // Print the square root of the error variance of fuel mass flow rate
  
  Serial.print("Engine torque: "); // Print the label for engine torque
  Serial.print(x[3]); // Print the state estimate of engine torque
  Serial.print(" +/- "); // Print the plus-minus sign for error
  Serial.println(sqrt(P[3][3])); // Print the square root of the error variance of engine torque
  
  Serial.println(); // Print a blank line
  
  Serial.print("Throttle position: "); // Print the label for throttle position
  Serial.println(z[0]); // Print the measurement of throttle position
  
  Serial.print("MAP: "); // Print the label for MAP
  Serial.println(z[1]); // Print the measurement of MAP
  
  Serial.print("Lambda value: "); // Print the label for lambda value
  Serial.println(z[2]); // Print the measurement of lambda value
  
  Serial.print("RPM: "); // Print the label for RPM
  Serial.println(z[3]); // Print the measurement of RPM
  
  Serial.println(); // Print a blank line
}

// Define a function to run the Kalman filter algorithm using a loop
void runKalmanFilter()
{
  float u[M]; // Input vector
  float dt; // Time step in seconds
  unsigned long currentTime; // Current time in milliseconds
  unsigned long previousTime; // Previous time in milliseconds
  
  initialize(); // Initialize the state estimate and its error covariance matrix
  
  previousTime = millis(); // Get the initial time in milliseconds
  
  while (true) // Loop forever
  {
    readSensors(z); // Read the sensors and store the measurements in the measurement vector
    
    u[0] = z[0]; // Set the input vector as the measurements of throttle position, MAP, and lambda value
    u[1] = z[1];
    u[2] = z[2];
    
    currentTime = millis(); // Get the current time in milliseconds
    
    dt = (currentTime - previousTime) / 1000.0; // Calculate the time step in seconds
    
    predict(x, P, u, dt); // Predict the next state estimate and its error covariance matrix
    
    update(x, P, z); // Update the state estimate and its error covariance matrix using the measurements
    
    controlInjector(x); // Control the injector using the state estimate of the pulse width
    
    printInfo(x, P, z); // Print some information to the serial monitor for debugging purposes
    
    previousTime = currentTime; // Update the previous time to the current time
  }
}

// Define a function to set up the Arduino board
void setup()
{
  Serial.begin(9600); // Start serial communication at 9600 baud rate
  
  pinMode(throttlePin, INPUT); // Set the throttle position sensor pin as an input
  pinMode(mapPin, INPUT); // Set the MAP sensor pin as an input
  pinMode(lambdaPin, INPUT); // Set the lambda probe pin as an input
  pinMode(camPin, INPUT_PULLUP); // Set the cam sensor pin as an input with a pull-up resistor
  pinMode(injectorPin, OUTPUT); // Set the injector pin as an output
  
  runKalmanFilter(); // Run the Kalman filter algorithm
}

// Define a function to loop the Arduino board
void loop()
{
  // Nothing to do here since everything is done in the runKalmanFilter function
}
