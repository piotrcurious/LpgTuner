// Define six variables to be controlled
int var1 = 0;
int var2 = 0;
int var3 = 0;
int var4 = 0;
int var5 = 0;
int var6 = 0;

// Define a variable to store the selected variable
int selected = 0;

void setup() {
  // Initialize serial communication at 9600 baud
  Serial.begin(9600);
}

void loop() {
  // Check if there is any serial input
  if (Serial.available() > 0) {
    // Read the incoming byte
    char input = Serial.read();

    // Check if the input is a digit from 1 to 6
    if (input >= '1' && input <= '6') {
      // Set the selected variable to the corresponding one
      selected = input - '0';
    }

    // Check if the input is a minus sign
    else if (input == '-') {
      // Decrement the selected variable by 1
      switch (selected) {
        case 1:
          var1--;
          break;
        case 2:
          var2--;
          break;
        case 3:
          var3--;
          break;
        case 4:
          var4--;
          break;
        case 5:
          var5--;
          break;
        case 6:
          var6--;
          break;
      }
    }

    // Check if the input is an equal sign
    else if (input == '=') {
      // Increment the selected variable by 1
      switch (selected) {
        case 1:
          var1++;
          break;
        case 2:
          var2++;
          break;
        case 3:
          var3++;
          break;
        case 4:
          var4++;
          break;
        case 5:
          var5++;
          break;
        case 6:
          var6++;
          break;
      }
    }

    // Print all the variables to the serial monitor
    Serial.print("var1 = ");
    Serial.print(var1);
    Serial.print(", var2 = ");
    Serial.print(var2);
    Serial.print(", var3 = ");
    Serial.print(var3);
    Serial.print(", var4 = ");
    Serial.print(var4);
    Serial.print(", var5 = ");
    Serial.print(var5);
    Serial.print(", var6 = ");
    Serial.print(var6);

    // Print the selected variable with an asterisk
    Serial.print(", selected = var");
    Serial.print(selected);
    Serial.println("*");
  }
}
