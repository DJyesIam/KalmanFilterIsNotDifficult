#define TRIG 3
#define ECHO 2
#define n 10  // 저장해둘 데이터의 개수

float prevAvg = 0;
float data[n] = {0,};

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
  Serial.print(MovAvgFilter(distance));
  Serial.println();

}

float MovAvgFilter(float x){
  float avg = prevAvg + (x - data[0]) / n;
  updateData(x);
  prevAvg = avg;
  return avg;
}

void updateData(float x){
  for (int i = 0; i < n - 1; i++){
    data[i] = data[i + 1];
  }
  data[n - 1] = x;
}
