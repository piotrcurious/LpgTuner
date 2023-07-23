// Define some constants
#define N 1 // Number of state variables
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
uint32_t pulseWidthISR; // Pulse width in microseconds

// Define some variables for the cam sensor interrupt
volatile unsigned long lastCamTime = 0; // Last time the cam sensor triggered in microseconds
volatile unsigned long camPeriod = 0; // Cam sensor period in microseconds

// Define a function to calculate the state transition function f
void f(float x[], float u[], float dt, float xdot[])
{
  // x[0] is the pulse width in milliseconds
  
  // u[0] is the throttle position in percentage
  
  // dt is the time step in seconds
  
  // xdot is the output vector of the state derivatives
  
  float pw = x[0]; // Pulse width in milliseconds
  
  float tp = u[0]; // Throttle position in percentage
  
  
  xdot[0] = -K * (pw - tp); // Calculate pulse width derivative using a first-order model with a gain factor K and a time constant T
  
}

// Define a function to calculate the measurement function h
void h(float x[], float z[])
{
  // x[0] is the pulse width in milliseconds
  
  // z[0] is the throttle position in percentage
  // z[1] is the MAP in kPa
  // z[2] is the lambda value
  // z[3] is the RPM in revolutions per minute
  
  float pw = x[0]; // Pulse width in milliseconds
  
  
  z[0] = pw / T; // Calculate throttle position using a linear relationship with pulse width and a time constant T 
}

// Define a function to calculate the Jacobian matrix F of f with respect to x
void F(float x[], float u[], float dt, float F[][N])
{
  // x[0] is the pulse width in milliseconds
  
  // u[0] is the throttle position in percentage
  
  // dt is the time step in seconds
  
  // F is the output matrix of partial derivatives of f with respect to x
  
  float pw = x[0]; // Pulse width in milliseconds
  
  float tp = u[0]; // Throttle position in percentage
  
  
  F[0][0] = -K; // Partial derivative of f[0] with respect to x[0]
}

// Define a function to calculate the Jacobian matrix H of h with respect to x
void H(float x[], float H[][N])
{
  // x[0] is the pulse width in milliseconds
  
  // H is the output matrix of partial derivatives of h with respect to x
  
  float pw = x[0]; // Pulse width in milliseconds
  
  
  H[0][0] = 1 / T; // Partial derivative of h[0] with respect to x[0]
}

// Define a function to initialize the state estimate and its error covariance matrix
void initialize()
{
  // Initialize the state estimate based on some initial values or guesses
  x[0] = 10; // Initial pulse width in milliseconds
  
  // Initialize the error covariance matrix based on some initial values or guesses
  P[0][0] = 1; // Initial error variance of pulse width in milliseconds squared
  
  // Initialize the process noise covariance matrix based on some values or guesses
  Q[0][0] = 0.01; // Process noise variance of pulse width in milliseconds squared
  
  // Initialize the measurement noise covariance matrix based on some values or guesses
  R[0][0] = 0.1; // Measurement noise variance of throttle position in percentage squared
  R[1][1] = 1; // Measurement noise variance of MAP in kPa squared
  R[2][2] = 0.01; // Measurement noise variance of lambda value in ratio squared
  R[3][3] = 10; // Measurement noise variance of RPM in revolutions per minute squared
  
  // Initialize the identity matrix
  I[0][0] = 1; // Diagonal element is one
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

// Define a function to measure the actual pulse width of the injector using an interrupt
void measureInjectorPulseWidth()
{
  //TODO sync with interrupts 
  pulseWidth = pulseWidthISR/ 1000.0; // Calculate the pulse width in milliseconds based on the injector on and off times in microseconds
}

// Define a function to handle the injector interrupt
void injectorISR()
{
  unsigned long currentInjectorTime = micros(); // Get the current time in microseconds
  
  if (digitalRead(injectorPin) == HIGH) // If the injector pin is high
  {
    injectorOnTime = currentInjectorTime; // Update the injector on time to the current time
  }
  else // If the injector pin is low
  {
    injectorOffTime = currentInjectorTime; // Update the injector off time to the current time
    pulseWidthISR = (injectorOffTime - injectorOnTime) // Calculate the pulse width in micros based on the injector on and off times in microseconds and double buffer to reduce chances of glitches
  }
}

// Define a function to adjust the process noise covariance matrix based on the predictor and other inputs
void adjustProcessNoise(float x[], float Q[][N], float z[])
{
  // x[0] is the pulse width in milliseconds
  
  // Q[0][0] is the process noise variance of pulse width in milliseconds squared
  
  // z[0] is the throttle position in percentage
  // z[1] is the MAP in kPa
  // z[2] is the lambda value
  // z[3] is the RPM in revolutions per minute
  
  float pw = x[0]; // Pulse width in milliseconds
  
  float tp = z[0]; // Throttle position in percentage
  float mp = z[1]; // MAP in kPa
  float lam = z[2]; // Lambda value
  float rpm = z[3]; // RPM in revolutions per minute
  
  float pwThreshold = 0.5; // Threshold for pulse width difference in milliseconds
  float mpThreshold = 5; // Threshold for MAP difference in kPa
  float lamThreshold = 0.1; // Threshold for lambda value difference in ratio
  float rpmThreshold = 100; // Threshold for RPM difference in revolutions per minute
  
  float increaseFactor = 0.1; // Factor to increase the process noise variance by
  
  //.     -----pivot------
  // Check if the difference between the predicted and measured pulse width is larger than the threshold
  if (abs(pw - pulseWidth) > pwThreshold)
  {
    // Increase the process noise variance by a factor
    Q[0][0] = Q[0][0] * (1 + increaseFactor);
  }
  
  // Check if the difference between the current and previous measurements of MAP is larger than the threshold
  if (abs(mp - previousMp) > mpThreshold)
  {
    // Increase the process noise variance by a factor
    Q[0][0] = Q[0][0] * (1 + increaseFactor);
  }
  
  // Check if the difference between the current and previous measurements of lambda value is larger than the threshold
  if (abs(lam - previousLam) > lamThreshold)
  {
    // Increase the process noise variance by a factor
    Q[0][0] = Q[0][0] * (1 + increaseFactor);
  }
  
  // Check if the difference between the current and previous measurements of RPM is larger than the threshold
  if (abs(rpm - previousRpm) > rpmThreshold)
  {
    // Increase the process noise variance by a factor
    Q[0][0] = Q[0][0] * (1 + increaseFactor);
  }
  
  // Update the previous measurements of MAP, lambda value, and RPM
  previousMp = mp;
  previousLam = lam;
  previousRpm = rpm;
}

// Setup function to set up interrupts and other things 

void setup() {
  attachInterrupt(digitalPinToInterrupt(injectorPin), injectorISR, CHANGE); // Attach an interrupt to the injector pin on both edges
}
