#define TRIG 3
#define ECHO 2

float prevX = 0;
float alpha = 0.7;
/* 가중치. 0과 1사이의 값을 가진다. 
 * alpha가 클수록 측정값을 작게 반영하여 노이즈를 잘 잡지만 측정값의 변화에 민감하게 반응하지 못한다. 
 * alpha가 작을수록 측정값을 크게 반영하여 측정값의 변화에 빠르게 반응하지만 노이즈가 크다. 
 */

void setup() {
  Serial.begin(9600);
  pinMode(TRIG,OUTPUT);
  pinMode(ECHO,INPUT);
}

void loop() {
  float duration, distance;

  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  duration = pulseIn (ECHO, HIGH);
  distance = duration * 17 / 1000; 
  
  Serial.print(distance);
  Serial.print(",");
  Serial.print(LowPassFilter(distance));
  Serial.println();
}

float LowPassFilter(float x){
  x = alpha * prevX + (1 - alpha) * x;
  prevX = x;
  return x;
}
