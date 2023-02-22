// 노이즈가 낀 위치 데이터를 칼만 필터를 돌려 실제 위치와 속도를 추정한다.

// 아두이노 라이브러리 관리에서 MatrixMath 라이브러리(by Charlie Matlack)를 다운받는다.
#include <MatrixMath.h>
#define N 2  // 행렬의 행(열)의 개수
double dt = 0.1;  // 측정 시간 간격
double PosP = 0;  // 실제 위치
double VelP = 80;  // 실제 속도

// 시스템 모델 변수
mtx_type A[N][N];
mtx_type H[N];
mtx_type Q[N][N];
mtx_type R[1][1];

mtx_type x[N][1];
mtx_type P[N][N];

mtx_type AT[N][N];  // A의 전치행렬
mtx_type HT[N][1];  // H의 전치행렬

mtx_type tmpMat1[2][1];  // 행렬 연산결과를 임시 저장할 행렬
mtx_type tmpMat2[2];  
mtx_type tmpMat3[1][1];
mtx_type tmpMat4[N][N];

double tmp;

void setup() {
  Serial.begin(9600);
  setSystemModel();
  Matrix.Transpose((mtx_type*)A, 2, 2, (mtx_type*)AT);
  Matrix.Transpose((mtx_type*)H, 1, 2, (mtx_type*)HT);
  randomSeed(analogRead(A0));
}

void loop() {  // 노이즈가 낀 속도 측정값을 칼만 필터를 돌려 위치값을 추정한다.
  IntKalman(getVel());
  Serial.print(millis() / 1000);
  Serial.print(",");
  Serial.println(x[0][0]);
  delay(1000 * dt);
}

void setSystemModel(){  // 시스템 모델 변수 초기화
  A[0][0] = 1;
  A[0][1] = dt;
  A[1][0] = 0;
  A[1][1] = 1;

  H[0] = 0;
  H[1] = 1;

  Q[0][0] = 1;
  Q[0][1] = 0;
  Q[1][0] = 0;
  Q[1][1] = 3;

  R[0][0] = 10;

  x[0][0] = 0;
  x[1][0] = 20;

  P[0][0] = 5;
  P[0][1] = 0;
  P[1][0] = 0;
  P[1][1] = 5;
}

void IntKalman(double z){
  mtx_type xp[N][1];
  mtx_type Pp[N][N];
  mtx_type K[N][1];
  
  // xp = A*x
  Matrix.Multiply((mtx_type*)A, (mtx_type*)x, 2, 2, 1, (mtx_type*)xp);

  // Pp = A*P*AT + Q 
  Matrix.Multiply((mtx_type*)A, (mtx_type*)P, 2, 2, 2, (mtx_type*)tmpMat4);
  Matrix.Multiply((mtx_type*)tmpMat4, (mtx_type*)AT, 2, 2, 2, (mtx_type*)tmpMat4);
  Matrix.Add((mtx_type*)tmpMat4, (mtx_type*)Q, 2, 2, (mtx_type*)Pp);

  // K = Pp*HT*(H*Pp*HT + R)^-1
  Matrix.Multiply((mtx_type*)Pp, (mtx_type*)HT, 2, 2, 1, (mtx_type*)tmpMat1);
  Matrix.Multiply((mtx_type*)H, (mtx_type*)Pp, 1, 2, 2, (mtx_type*)tmpMat2);
  Matrix.Multiply((mtx_type*)tmpMat2, (mtx_type*)HT, 1, 2, 1, (mtx_type*)tmpMat3);
  Matrix.Add((mtx_type*)tmpMat3, (mtx_type*)R, 1, 1, (mtx_type*)tmpMat3);
  Matrix.Invert((mtx_type*)tmpMat3, 1);
  tmp = tmpMat3[0][0];
  Matrix.Scale((mtx_type*)tmpMat1, 2, 1, tmp);
  Matrix.Copy((mtx_type*)tmpMat1, 2, 1, (mtx_type*)K);

  // x = xp + K*(z - H*xp)
  Matrix.Multiply((mtx_type*)H, (mtx_type*)xp, 1, 2, 1, (mtx_type*)tmpMat3);
  tmp = tmpMat3[0][0];
  tmp = z - tmp;
  tmpMat1[0][0] = K[0][0]; tmpMat1[1][0] = K[1][0]; 
  Matrix.Scale((mtx_type*)tmpMat1, 2, 1, tmp);
  Matrix.Add((mtx_type*)xp, (mtx_type*)tmpMat1, 2, 1, (mtx_type*)x);

  // P = Pp - K*H*Pp
  Matrix.Multiply((mtx_type*)K, (mtx_type*)H, 2, 1, 2, (mtx_type*)tmpMat4);
  Matrix.Multiply((mtx_type*)tmpMat4, (mtx_type*)Pp, 2, 2, 2, (mtx_type*)tmpMat4);
  Matrix.Subtract((mtx_type*)Pp, (mtx_type*)tmpMat4, 2, 2, (mtx_type*)P);
}

double getVel(){  // 노이즈가 낀 속도 값을 반환한다.
  double v = random(0,11);  // 시스템 노이즈
  if (random(0,2) % 2) v = -v;
  
  PosP = PosP + VelP*dt;
  VelP = 80 + v;

//  Serial.print(VelP);  // 노이즈가 낀 속도 출력
  return VelP;  // 노이즈가 낀 속도 반환
}
