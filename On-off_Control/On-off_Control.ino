int prescaler = 256; // Set this to match whatever prescaler value you set in CS registers below

// Initialize values for the PWM duty cycle set by pots
float potDC1 = 0;
float potDC2 = 0;
float potDC3 = 0;
float potDC4 = 0;
float potDC5 = 0;
int max = 1000;
int delay_1 = 2000;

// Pin mapping (Arduino MEGA 2560):
// Timer3: OC3A=5, OC3B=2, OC3C=3
// Timer4: OC4A=6, OC4B=7
// Manual switches (digital): 49, 50, 51, 52, 53
// Pots (analog): A1-A5
// Pressure sensors (analog): A8-A12

void setup() {

  Serial.begin(9600);

  // Input pins for valve switches
  pinMode(49, INPUT);
  pinMode(50, INPUT);
  pinMode(51, INPUT);
  pinMode(52, INPUT);
  pinMode(53, INPUT);

  // Output pins for valve PWM
  pinMode(5, OUTPUT);  //OC3A
  pinMode(2, OUTPUT);  //OC3B
  pinMode(3, OUTPUT);  //OC3C
  pinMode(6, OUTPUT);  //OC4A
  pinMode(7, OUTPUT);  //OC4B

  int eightOnes = 255;  // 11111111 in binary

  // Set the eight bits in TCCR registers to 0
  TCCR3A &= ~eightOnes;
  TCCR3B &= ~eightOnes;
  TCCR4A &= ~eightOnes;
  TCCR4B &= ~eightOnes;

  // Phase and frequency correct PWM, non-inverting, prescaler 256
  TCCR3A = _BV(COM3A1) | _BV(COM3B1) | _BV(COM3C1);
  TCCR3B = _BV(WGM33) | _BV(CS32);
  TCCR4A = _BV(COM4A1) | _BV(COM4B1);
  TCCR4B = _BV(WGM43) | _BV(CS42);
}

void pPWM(float pwmFreq, float pwmDC1, float pwmDC2, float pwmDC3, float pwmDC4, float pwmDC5) {

  // Set PWM frequency by adjusting ICR (top of triangle waveform)
  ICR3 = F_CPU / (prescaler * pwmFreq * 2);
  ICR4 = F_CPU / (prescaler * pwmFreq * 2);

  // Set duty cycles
  OCR3A = (ICR3 * (pwmDC1 * 0.01));
  OCR3B = (ICR3 * (pwmDC2 * 0.01));
  OCR3C = (ICR3 * (pwmDC3 * 0.01));
  OCR4A = (ICR4 * (pwmDC4 * 0.01));
  OCR4B = (ICR4 * (pwmDC5 * 0.01));
}

void loop() {

  potDC1 = 0;
  potDC2 = 0;
  potDC3 = 0;
  potDC4 = 0;
  potDC5 = 0;

  float potPWMfreq = 100.0;

  // If statements for manual switch override
  // Scale values from pot to 0-100, which gets used for duty cycle percentage
  if (digitalRead(49) == HIGH) { potDC1 = analogRead(A1) * 100.0 / 1024.0; }
  if (digitalRead(50) == HIGH) { potDC2 = analogRead(A2) * 100.0 / 1024.0; }
  if (digitalRead(51) == HIGH) { potDC3 = analogRead(A3) * 100.0 / 1024.0; }
  if (digitalRead(52) == HIGH) { potDC4 = analogRead(A4) * 100.0 / 1024.0; }
  if (digitalRead(43) == HIGH) { potDC5 = analogRead(A5) * 100.0 / 1024.0; }

  // Print manual inputs
  Serial.print(potDC1);
  Serial.print("\n");
  Serial.print(potDC2);
  Serial.print("\n");
  Serial.print(potDC3);
  Serial.print("\n");
  Serial.print(potDC4);
  Serial.print("\n");
  Serial.print(potDC5);
  Serial.print("\n");

  // Update PWM outputs based on the above values from pots
  pPWM(potPWMfreq, potDC1, potDC2, potDC3, potDC4, potDC5);

  // Scale values from pot to 0-100, which gets used for frequency (Hz)
  // potPWMfreq = round(potPWMfreq / 5) * 5 - 1;
  // 1 to 91 Hz in increments of 5 (rounding helps to deal with noisy pot)

  int status = 0;
  // status = digitalRead(49) * digitalRead(50) * digitalRead(51) * digitalRead(52) * digitalRead(53);

  if (status == 0) {
    for (int n = 0; n <= 2; n++) {
      
      // Step 1
      potDC3 = max;
      pPWM(potPWMfreq, potDC1, potDC2, potDC3, potDC4, potDC5);
      delay(4000);
      
      // Step 2
      potDC4 = 82;
      potDC5 = 85;
      pPWM(potPWMfreq, potDC1, potDC2, potDC3, potDC4, potDC5);
      delay(delay_1 * 2.5);

      // Step 3
      potDC2 = max;
      potDC1 = max;
      pPWM(potPWMfreq, potDC1, potDC2, potDC3, potDC4, potDC5);
      delay(delay_1);

      // Step 4
      potDC4 = 0;
      potDC5 = 0;
      potDC3 = 0;
      pPWM(potPWMfreq, potDC1, potDC2, potDC3, potDC4, potDC5);
      delay(delay_1);

      // Step 5
      potDC2 = 0;
      potDC1 = 0;
      pPWM(potPWMfreq, potDC1, potDC2, potDC3, potDC4, potDC5);
      delay(delay_1);
    }
  }

  // Transfer function for sensor Honeywell ASDXRRX100PGAA5 (100 psi, 5V, A-calibration)
  // V_out = 0.8 * V_supply / (P_max - P_min) * (P_applied - P_min) + 0.1 * V_supply
  // Rearrange: P_applied = (V_out / V_supply - 0.1) * (P_max - P_min) / 0.8 + P_min
  // Read output voltages from sensors and convert to pressure readings in PSI
  float P1 = (analogRead(A8) / 1024.0 - 0.1) * 100.0 / 0.8;
  float P2 = (analogRead(A9) / 1024.0 - 0.1) * 100.0 / 0.8;
  float P3 = (analogRead(A10) / 1024.0 - 0.1) * 100.0 / 0.8;
  float P4 = (analogRead(A11) / 1024.0 - 0.1) * 100.0 / 0.8;
  float P5 = (analogRead(A12) / 1024.0 - 0.1) * 100.0 / 0.8;

  // Print pressure readings
  Serial.print(P1);
  Serial.print("\t");
  Serial.print(P2);
  Serial.print("\t");
  Serial.print(P3);
  Serial.print("\t");
  Serial.print(P4);
  Serial.print("\t");
  Serial.print(P5);
  Serial.print("\n");

  delay(100);
}
