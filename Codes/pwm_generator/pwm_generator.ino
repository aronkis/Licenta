#define max_pwm_time 2000
unsigned long pwm_time = 1000;

void setup() {
  Serial.begin(19200);
  Serial.println("start");
  pinMode(7, OUTPUT);
  pinMode(A0, INPUT);
  digitalWrite(7, LOW);
}

void loop() {

  //analogRead(A0);
  pwm_time = map(analogRead(A0), 310, 1024, 1000, 2000);
  Serial.println(pwm_time);
  
  digitalWrite(7, HIGH);
  delayMicroseconds(pwm_time);
  digitalWrite(7, LOW);
  delayMicroseconds(max_pwm_time - pwm_time);
}
