#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define DIR_PIN 23
#define STEP_PIN 19
#define SWITCH_RIGHT_PIN 26
#define SWITCH_LEFT_PIN 12
#define POWER_RIGHT_SWITCH 25
#define POWER_LEFT_SWITCH 14
#define MOTOR_CONTROL_PIN 18
const int pinBuzzer = 27;
const int pinPower1 = 33;
const int pinPower2 = 34;
const int pinLectura = 32;
const int pinVoltaje = 4;
const float factorEscala = 4.0;
const int umbral = 200;
unsigned long previousStepTime = 0;
int stepsRemaining = 0;
unsigned long microStepDelay = 0;
unsigned long displayUpdateInterval = 500;
unsigned long lastDisplayUpdate = 0;
unsigned long lastBuzzerTime = 0;
bool buzzerActive = false;
bool screenLocked = false;
float Inf = 0.0;
const int frecuencia = 2000;
const int duracionTono = 100;
const int pausaEntreTonos = 900;

float SENSIBILITY = 0.042;
int SAMPLESNUMBER = 100;

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

unsigned long lastInfusionTime = 0;
unsigned long infusionInterval = 0;

int ultimoEstadoPIC = -1;
unsigned long ultimoTiempoPIC = 0;

bool enCalibracion = false;
bool finDeCarrera = false;

void displayWarning(String message);
void displayWarning2(String message);
float getCorriente(int samplesNumber);
float getVoltaje(int samplesNumber);
void calibrateMotor();
void stepMotor();
void infuseVolume(float mlPerHour);
void handleInfusion();
void monitorSensors();
void triggerBuzzer(String mensaje);
void manageBuzzer();
void updateDisplayIfNeeded();
void checkSerialInput();
void setDelay(float mlPerHour);
float leerInf();
void emitirBuzzer();

void beepSimple(int duracionMs) {
  unsigned long inicio = millis();
  while (millis() - inicio < (unsigned long)duracionMs) {
    digitalWrite(pinBuzzer, HIGH);
    delayMicroseconds(500);
    digitalWrite(pinBuzzer, LOW);
    delayMicroseconds(500);
  }
}

void leerEstadoPIC() {
  bool recibio = false;

  while (Serial2.available() > 0) {
    char c = Serial2.read();
    recibio = true;
    ultimoTiempoPIC = millis();

    if (c == 'A') {
      ultimoEstadoPIC = 1;
      Serial.println("PIC: JERINGA DETECTADA (A)");
    }
    else if (c == 'S') {
      ultimoEstadoPIC = 0;
      Serial.println("PIC: JERINGA NO DETECTADA (S)");
    }
  }

  if (!recibio && millis() - ultimoTiempoPIC >= 1000) {
    if (ultimoEstadoPIC == 1) {
      Serial.println("PIC (repetido): JERINGA DETECTADA");
    }
    else if (ultimoEstadoPIC == 0) {
      Serial.println("PIC (repetido): JERINGA NO DETECTADA");
    }
    ultimoTiempoPIC = millis();
  }
}

void aplicarLogicaPIC() {
  if (ultimoEstadoPIC == 0) {
    digitalWrite(MOTOR_CONTROL_PIN, HIGH);
  }
  else if (ultimoEstadoPIC == 1) {
    if (!finDeCarrera) {
      digitalWrite(MOTOR_CONTROL_PIN, LOW);
    }
  }
}

void setup() {
  pinMode(pinPower1, OUTPUT);
  pinMode(pinPower2, OUTPUT);
  pinMode(pinLectura, INPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(MOTOR_CONTROL_PIN, OUTPUT);
  digitalWrite(MOTOR_CONTROL_PIN, LOW);
  pinMode(pinBuzzer, OUTPUT);
  digitalWrite(pinPower1, HIGH);
  digitalWrite(pinPower2, HIGH);

  pinMode(POWER_RIGHT_SWITCH, OUTPUT);
  pinMode(POWER_LEFT_SWITCH, OUTPUT);
  digitalWrite(POWER_RIGHT_SWITCH, HIGH);
  digitalWrite(POWER_LEFT_SWITCH, HIGH);

  pinMode(SWITCH_RIGHT_PIN, INPUT_PULLUP);
  pinMode(SWITCH_LEFT_PIN, INPUT_PULLUP);

  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, -1);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    while (true) { delay(100); }
  }

  display.display();
  delay(2000);
  display.clearDisplay();
  display.display();
  displayWarning("Calibrando");
  calibrateMotor();
  displayWarning("Calibrado");
}

void loop() {
  leerEstadoPIC();
  aplicarLogicaPIC();
  checkSerialInput();

  unsigned long currentTime = millis();

  if (currentTime - lastInfusionTime >= infusionInterval && Inf > 0) {
    handleInfusion();
    lastInfusionTime = currentTime;
  }

  monitorSensors();
  manageBuzzer();
  updateDisplayIfNeeded();
}

void checkSerialInput() { 
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    if (command == "RESET") { 
      ESP.restart(); 
    } else { 
      float valorRecibido = command.toFloat();
      if (valorRecibido != 0.0) { 
        Inf = valorRecibido;
        setDelay(Inf); 
      } 
    } 
  } 
}

void calibrateMotor() {
  enCalibracion = true;
  finDeCarrera = false;

  digitalWrite(DIR_PIN, HIGH);
  while (digitalRead(SWITCH_RIGHT_PIN) == HIGH) {
    stepMotor();
  }
  delay(1000);
  digitalWrite(DIR_PIN, LOW);
  while (digitalRead(SWITCH_LEFT_PIN) == HIGH) {
    stepMotor();
  }
  digitalWrite(DIR_PIN, HIGH);

  enCalibracion = false;
}

void stepMotor() {
  if (finDeCarrera && !enCalibracion) return;
  if (ultimoEstadoPIC == 0 && !enCalibracion) return;

  if (!enCalibracion) {
    if (digitalRead(DIR_PIN) == HIGH && digitalRead(SWITCH_RIGHT_PIN) == LOW) {
      finDeCarrera = true;
      triggerBuzzer("LIMITE DERECHO");
      screenLocked = true;
      return;
    }
    if (digitalRead(DIR_PIN) == LOW && digitalRead(SWITCH_LEFT_PIN) == LOW) {
      finDeCarrera = true;
      triggerBuzzer("LIMITE IZQUIERDO");
      screenLocked = true;
      return;
    }
  }

  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(1000);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(1000);
}

void setDelay(float mlPerHour) {
  int segMl = (1.0/mlPerHour)*3600000;
  infusionInterval = (5 * segMl) / 200;
}

void handleInfusion() {
  if (ultimoEstadoPIC == 0) return;
  if (finDeCarrera) return;

  for (int i = 0; i < 5; i++){ 
    stepMotor();
  }
}

void monitorSensors() {
  float current = getCorriente(SAMPLESNUMBER);
  int valorLectura = analogRead(pinLectura);
  float Voltaje = getVoltaje(SAMPLESNUMBER);

  if (valorLectura < umbral && !buzzerActive) {
    buzzerActive = true;
    screenLocked = true;
    triggerBuzzer("Revisar jeringa");
  } else if (valorLectura >= umbral) {
    buzzerActive = false;
    screenLocked = false;
  }

  if (Voltaje < 10 || current < 0.05) {
    triggerBuzzer("Alimentacion insuficiente");
    digitalWrite(MOTOR_CONTROL_PIN, HIGH);
    screenLocked = true;
  } else if (Voltaje > 12) {
    displayWarning2("VOLTAJE POR ENCIMA DEL LIMITE, INFUSION DETENIDA");
    while (true){
      emitirBuzzer();
      delay(500);
      if (getVoltaje(SAMPLESNUMBER) < 12){
        break;
      }
    }
  } else {
    if (!finDeCarrera && ultimoEstadoPIC == 1) {
      digitalWrite(MOTOR_CONTROL_PIN, LOW);
    }
    display.ssd1306_command(SSD1306_DISPLAYON);
    screenLocked = false;
  }
}

void triggerBuzzer(String mensaje) {
  lastBuzzerTime = millis();
  displayWarning2(mensaje);
}

void manageBuzzer() {
  if (buzzerActive && (millis() - lastBuzzerTime) >= pausaEntreTonos) {
    beepSimple(duracionTono);
    lastBuzzerTime = millis();
  }
}

void updateDisplayIfNeeded() {
  if (screenLocked) return;

  if (millis() - lastDisplayUpdate >= displayUpdateInterval && !buzzerActive) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);

    float current = getCorriente(SAMPLESNUMBER);
    float voltaje = getVoltaje(SAMPLESNUMBER);
    display.print(current, 2);
    display.print(" A ");
    display.print(current * 12, 2);
    display.print(" W ");
    display.print(voltaje, 2);
    display.print(" V ");
    display.setTextSize(2);
    display.setCursor(0, 20);
    display.print(Inf, 2);
    display.print(" ml/h");

    display.display();
    lastDisplayUpdate = millis();
  }
}

float getCorriente(int samplesNumber) {
  float corrienteSum = 0;
  for (int i = 0; i < samplesNumber; i++) {
    float voltage = analogRead(34) * 3.327 / 4095;
    corrienteSum += (voltage - 1.680) / SENSIBILITY;
  }
  return corrienteSum / samplesNumber;
}

float getVoltaje(int samplesNumber) {
  float voltajeSum = 0;
  for (int i = 0; i < samplesNumber; i++) {
    float voltage = analogRead(pinVoltaje) * 3.0 / 4095.0;
    voltajeSum += voltage;
  }
  float voltajeMedido = voltajeSum / samplesNumber;
  float voltajeReal = voltajeMedido * factorEscala;
  return voltajeReal;
}

void displayWarning(String message) {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 20);
  display.print(message);
  display.display();
}

void displayWarning2(String message) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 20);
  display.print(message);
  display.display();
}

float leerInf() {
  float datoRecibido = Serial.parseFloat();
  return datoRecibido;
}

void emitirBuzzer() {
  beepSimple(500);
}
