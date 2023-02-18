#define TRIG 3
#define ECHO 2

float prevX = 0;

float A,H,Q,R;  // 시스템 모델 변수
float P = 6;  // 오차 공분산 추정값


void setup() {
  Serial.begin(9600);
  pinMode(TRIG,OUTPUT);
  pinMode(ECHO,INPUT);
  set();
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
  SimpleKalmanFilter(distance);
  Serial.println(prevX);
  Serial.println();
}

void set(){  // 시스템 모델 변수 초기화
  A = 1.0; H = 1.0;
  
  Q = 1.0; R = 4.0;
  
  P = 6.0;
}

float SimpleKalmanFilter(float z){
  float xp = A * prevX;  // 거리 예측값
  float Pp = A*P*A + Q;  // 오차 공분산 예측값

  float K = Pp*H/(H*Pp*H + R);

  prevX = xp + K*(z - H*xp);
  P = Pp - K*H*Pp;

  return K;
}
