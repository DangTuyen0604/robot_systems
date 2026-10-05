// Giao thức serial 115200 baud, mỗi gói một dòng (tối đa 32 ký tự):
//   V:<pwm 0..255>;E:<sai số pixel -400..400>   lệnh chạy, từ motor_node (/control/cmd)
//   KS:<kp>;KSI:<ki>;KSD:<kd>                   chỉnh PID lái (/arduino/pid)
//   P:<kp>;I:<ki>;D:<kd>                        chỉnh PID tốc độ (khi USE_SPEED_PID = 1)
// Không nhận được gói V/E hợp lệ trong CMD_TIMEOUT_MS thì dừng động cơ.
// Dòng debug gửi về Pi mỗi 100 ms khi xe chạy:
//   ERR:<pixel>,D:<delta>,PL:<pwm trái>,PR:<pwm phải>,EL:<xung trái>,ER:<xung phải>

// ====== PIN CONFIG ======
#define ENA 3
#define IN1 5
#define IN2 6
#define ENB 11
#define IN3 9
#define IN4 10
#define ENC_R_A 4
#define ENC_L_A 12

// ====== PARAM ======
#define CMD_TIMEOUT_MS   500
#define MIN_PWM          65
#define PID_INTERVAL_MS  20

// ====== PID TỐC ĐỘ (tùy chọn) ======
// 0: PWM vòng hở (mặc định). 1: dùng encoder giữ tốc độ từng bánh theo PWM
// mong muốn. Chỉ bật sau khi EL/ER trên /arduino/telemetry tăng đều khi bánh
// quay, và đo lại MAX_PULSES_PER_INTERVAL trên xe (xung mỗi PID_INTERVAL_MS
// ở PWM 255).
#define USE_SPEED_PID 0
#define MAX_PULSES_PER_INTERVAL 17.6
float Kp = 2.0;
float Ki = 0.05;
float Kd = 0.02;
#define INTEGRAL_MAX 50.0

// ====== PID STEERING ======
float Ks_p = 0.03;
float Ks_i = 0.001;
float Ks_d = 0.05;
#define STEER_INTEGRAL_MAX 200.0

// ====== ENCODER ======
// Chỉ đếm xung (chưa dùng cho điều khiển). Số xung được gửi trong dòng debug
// để đo tốc độ bánh thực tế theo PWM.
volatile long countL = 0;
volatile long countR = 0;

#if defined(__AVR_ATmega328P__)
// Uno/Nano: chỉ chân 2 và 3 có ngắt ngoài, attachInterrupt() trên chân 4 và 12
// bị bỏ qua mà không báo lỗi. Dùng ngắt đổi trạng thái chân (pin-change):
//   chân 12 = PB4 (PCINT4, nhóm PCINT0), chân 4 = PD4 (PCINT20, nhóm PCINT2).
#if ENC_L_A != 12 || ENC_R_A != 4
#error "Ngắt encoder viết cho chân 12 (trái) và 4 (phải); đổi chân thì sửa thanh ghi PCINT."
#endif
volatile uint8_t prev_enc_L = 0;
volatile uint8_t prev_enc_R = 0;

ISR(PCINT0_vect) {  // chân 12, encoder trái
  uint8_t s = PINB & _BV(PB4);
  if (s && !prev_enc_L) countL++;  // cạnh lên
  prev_enc_L = s;
}

ISR(PCINT2_vect) {  // chân 4, encoder phải
  uint8_t s = PIND & _BV(PD4);
  if (s && !prev_enc_R) countR++;  // cạnh lên
  prev_enc_R = s;
}

void setupEncoders() {
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  prev_enc_L = PINB & _BV(PB4);
  prev_enc_R = PIND & _BV(PD4);
  PCMSK0 |= _BV(PCINT4);
  PCMSK2 |= _BV(PCINT20);
  PCIFR = _BV(PCIF0) | _BV(PCIF2);
  PCICR |= _BV(PCIE0) | _BV(PCIE2);
}
#else
void encoderL_ISR() { countL++; }
void encoderR_ISR() { countR++; }

void attachEncoder(uint8_t pin, void (*isr)(), const char *name) {
  pinMode(pin, INPUT_PULLUP);
  if (digitalPinToInterrupt(pin) == NOT_AN_INTERRUPT) {
    Serial.print("WARN: chan encoder ");
    Serial.print(name);
    Serial.println(" khong co ngat ngoai, encoder se khong dem");
    return;
  }
  attachInterrupt(digitalPinToInterrupt(pin), isr, RISING);
}

void setupEncoders() {
  attachEncoder(ENC_L_A, encoderL_ISR, "trai");
  attachEncoder(ENC_R_A, encoderR_ISR, "phai");
}
#endif

void readEncoders(long &left, long &right) {
  noInterrupts();  // biến long 4 byte, đọc nguyên khối
  left = countL;
  right = countR;
  interrupts();
}

// ====== PID STATE ======
float prev_error_L = 0, prev_error_R = 0;
float integral_L   = 0, integral_R   = 0;
long  prev_enc_count_L = 0, prev_enc_count_R = 0;
float steer_prev_error = 0;
float steer_integral   = 0;

// ====== TIME ======
unsigned long last_pid_time = 0;
unsigned long last_cmd_time = 0;

// ====== LỆNH TỪ PI ======
int base_speed  = 0;
int pixel_error = 0;

// ====== SERIAL BUFFER ======
String serial_buffer = "";

// ==============================
// MOTOR
// ==============================
void setMotorL(int dir) {
  if (dir >= 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
  else          { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); }
}
void setMotorR(int dir) {
  if (dir >= 0) { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); }
  else          { digitalWrite(IN1, HIGH);  digitalWrite(IN2, LOW); }
}
void stopMotors() {
  analogWrite(ENA, 0); analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}
void resetPID() {
  integral_L = 0; integral_R = 0;
  prev_error_L = 0; prev_error_R = 0;
  steer_integral = 0; steer_prev_error = 0;
}

// ==============================
// PID
// ==============================
float PID_speed(float target, float current,
                float &prev_error, float &integral) {
  float error = target - current;
  integral += error;
  integral = constrain(integral, -INTEGRAL_MAX, INTEGRAL_MAX);
  float deriv = error - prev_error;
  prev_error = error;
  return Kp * error + Ki * integral + Kd * deriv;
}

float PID_steer(float error_px) {
  steer_integral += error_px;
  steer_integral = constrain(steer_integral, -STEER_INTEGRAL_MAX, STEER_INTEGRAL_MAX);
  float deriv = error_px - steer_prev_error;
  steer_prev_error = error_px;
  return Ks_p * error_px + Ks_i * steer_integral + Ks_d * deriv;
}

// ==============================
// MOVE
// ==============================
void Move() {
  unsigned long now = millis();
  if (now - last_pid_time < PID_INTERVAL_MS) return;
  last_pid_time = now;

  if (base_speed == 0) {
    stopMotors(); resetPID();
    // Bắt đầu chu kỳ chạy tiếp theo từ số xung hiện tại.
    readEncoders(prev_enc_count_L, prev_enc_count_R);
    return;
  }

  float delta = PID_steer((float)pixel_error);

  delta = constrain(delta, -80, 80);

  // MIN_PWM: dưới mức này động cơ không quay; mọi lệnh chạy được nâng lên ít
  // nhất MIN_PWM. Đo xung EL/ER theo PWM trước khi đổi ngưỡng này.
  int pwmL = constrain(base_speed + (int)delta, MIN_PWM, 255);
  int pwmR = constrain(base_speed - (int)delta, MIN_PWM, 255);

  long encL, encR;
  readEncoders(encL, encR);
#if USE_SPEED_PID
  // PWM mong muốn là feed-forward; PID bù chênh lệch giữa số xung đo được và
  // số xung tương ứng với PWM đó trong một chu kỳ.
  float measL = encL - prev_enc_count_L;
  float measR = encR - prev_enc_count_R;
  float targetL = pwmL / 255.0 * MAX_PULSES_PER_INTERVAL;
  float targetR = pwmR / 255.0 * MAX_PULSES_PER_INTERVAL;
  pwmL = constrain(pwmL + (int)PID_speed(targetL, measL, prev_error_L, integral_L), MIN_PWM, 255);
  pwmR = constrain(pwmR + (int)PID_speed(targetR, measR, prev_error_R, integral_R), MIN_PWM, 255);
#endif
  prev_enc_count_L = encL;
  prev_enc_count_R = encR;

  setMotorL(1); setMotorR(1);
  analogWrite(ENA, pwmL);
  analogWrite(ENB, pwmR);

  static unsigned long last_print = 0;
  if (now - last_print >= 100) {
    Serial.print("ERR:"); Serial.print(pixel_error);
    Serial.print(",D:"); Serial.print(delta, 1);
    Serial.print(",PL:"); Serial.print(pwmL);
    Serial.print(",PR:"); Serial.print(pwmR);
    Serial.print(",EL:"); Serial.print(encL);
    Serial.print(",ER:"); Serial.println(encR);
    last_print = now;
  }
}

// ==============================
// PARSE & SERIAL
// ==============================
void parseLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line.startsWith("P:")) {
    int i_idx = line.indexOf(";I:");
    int d_idx = line.indexOf(";D:");
    if (i_idx != -1 && d_idx != -1) {
      Kp = line.substring(2, i_idx).toFloat();
      Ki = line.substring(i_idx + 3, d_idx).toFloat();
      Kd = line.substring(d_idx + 3).toFloat();
    }
    return;
  }

  if (line.startsWith("KS:")) {
    int i_idx = line.indexOf(";KSI:");
    int d_idx = line.indexOf(";KSD:");
    if (i_idx != -1 && d_idx != -1) {
      Ks_p = line.substring(3, i_idx).toFloat();
      Ks_i = line.substring(i_idx + 5, d_idx).toFloat();
      Ks_d = line.substring(d_idx + 5).toFloat();
    }
    return;
  }

  int v_idx = line.indexOf("V:");
  int e_idx = line.indexOf(";E:");
  if (v_idx == -1 || e_idx == -1) return;

  int v = line.substring(v_idx + 2, e_idx).toInt();
  int e = line.substring(e_idx + 3).toInt();

  if (v < 0 || v > 255) return;
  if (e < -400 || e > 400) return;

  base_speed  = v;
  pixel_error = e;
  last_cmd_time = millis();
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      parseLine(serial_buffer);
      serial_buffer = "";
    } else if (c != '\r') {
      if (serial_buffer.length() < 32) serial_buffer += c;
      else serial_buffer = "";
    }
  }
}

// ==============================
// SETUP & LOOP
// ==============================
void setup() {
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

  stopMotors();
  Serial.begin(115200);
  setupEncoders();
  Serial.println("Arduino ready!");
  last_cmd_time = millis();
}

void loop() {
  readSerial();

  if (millis() - last_cmd_time > CMD_TIMEOUT_MS) {
    base_speed = 0; pixel_error = 0; resetPID();
  }

  Move();
}
