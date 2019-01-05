#include<Wire.h>
const int MPU_addr=0x68; 
float AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
float SampleFrequency = 200, dt = 1/SampleFrequency, loop_timer = 1000000*dt, t;
float GyXOff, GyYOff, GyZOff;
float acc_sensitivity=4096, gyro_sensitivity=65.5;
float p, q, r;
float phi_d_gyro, th_d_gyro;
float phi_gyro, th_gyro;
float AcX_P,AcY_P,AcZ_P;
float amx, amy, amz;
float phi_acc, th_acc;
float k=0.15, k2=0.05;
int start;
float sin_phi_gyro, cos_phi_gyro;

int ml1=2, ml2=3, mr1=4, mr2=7, enl=5, enr=6;

float setpoint_angle = -5;
float angle;
float error;
float previous_error;
float I_error;
float kp = 70; // 50
float kd = 70; // 100
float ki = 0; // 0.01
int pwm, pwm_l=0, pwm_r=0;

void setup(){
  Serial.begin(57600);
  Wire.begin();
  TWBR = 12;     
  setup_mpu_6050_registers();
  //calc_gyro_offset();
  float GyXOff = -179.27;
  float GyYOff = 58.37;
  float GyZOff = -63.08;

  pinMode(enl,OUTPUT);
  pinMode(enr,OUTPUT);
  pinMode(ml1,OUTPUT);
  pinMode(ml2,OUTPUT);
  pinMode(mr1,OUTPUT);
  pinMode(mr2,OUTPUT);
  
  analogWrite(enr,0);
  analogWrite(enl,0);
  
  t=micros();
}

void loop(){
  calculate_angle();
  //print_tilt_angle();
  calculate_motor_commands();
  drive_motors();

  Serial.println(pwm);
  //Serial.println(micros()-t);
  
  while(micros()-t < loop_timer){}
  t=micros();
}

void print_tilt_angle(){
  Serial.println(angle);
}

void calculate_angle(){
  read_mpu_6050_data();
  //LPF_acc();
  acc_angle_calc();
  gyro_angle_calc();
  gyro_drift_correction();
  angle = th_gyro*57.2957795131;
}

void setup_mpu_6050_registers(){
  //Activate the MPU-6050
  Wire.beginTransmission(MPU_addr);                                    //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission(true);                                          //End the transmission
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(MPU_addr);                                    //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission(true);                                          //End the transmission
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(MPU_addr);                                    //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission(true);                                          //End the transmission
  //low pass filter
  Wire.beginTransmission(MPU_addr);                                    //Start communicating with the MPU-6050
  Wire.write(0x1A);                                                    //Send the requested starting register
  Wire.write(0x03);                                                    //Set the requested starting register
  Wire.endTransmission(true); 
}

void calc_gyro_offset(){
  float n=500;
  Serial.println("");
  Serial.println("Calliberating Gyroscope !");
  for(int i=0;i<n;i++){
    if(i % 100 == 0){
      Serial.print("- ");
    }
    read_mpu_6050_data();
    GyXOff=GyXOff+GyX;
    GyYOff=GyYOff+GyY;
    GyZOff=GyZOff+GyZ;
    delay(5); 
  }
  GyXOff=GyXOff/n;
  GyYOff=GyYOff/n;
  GyZOff=GyZOff/n;
  Serial.println("");
  Serial.println("Gyro Caliberation Complete!!");
  Serial.println("Offsets are:");
  Serial.print("GyXOff = "); Serial.println(GyXOff);
  Serial.print("GyYOff = "); Serial.println(GyYOff);
  Serial.print("GyZOff = "); Serial.println(GyZOff);
  Serial.print("\n");
}

void read_mpu_6050_data(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  while(Wire.available() < 14); 
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

void LPF_acc(){ 
  AcX = AcX*k + (1-k)*AcX_P;
  AcY = AcY*k + (1-k)*AcY_P;
  AcZ = AcZ*k + (1-k)*AcZ_P;
  AcX_P = AcX;
  AcY_P = AcY;
  AcZ_P = AcZ;
}

void acc_angle_calc(){
  amx = AcX/acc_sensitivity;
  amy = AcY/acc_sensitivity;
  amz = AcZ/acc_sensitivity;

  if(amx>1)amx=1;
  if(amy>1)amy=1;
  if(amz>1)amz=1;
  
  phi_acc = atan2(amy,amz);
  th_acc = -asin(amx/sqrt(amx*amx + amy*amy + amz*amz));
}

void gyro_angle_calc(){
  p = (GyX-GyXOff)*0.00026646248122050833235476194938757; // PI = 3.1415926535897932384626433832795
  q = (GyY-GyYOff)*0.00026646248122050833235476194938757; // PI/(gyro_sensitivity*180) = 0.00026646248122050833235476194938757
  r = (GyZ-GyZOff)*0.00026646248122050833235476194938757;

  sin_phi_gyro = sin(phi_gyro);
  cos_phi_gyro = cos(phi_gyro);
  phi_gyro = phi_gyro + (p + (q*sin_phi_gyro + r*cos_phi_gyro)*tan(th_gyro))*dt;
  th_gyro  = th_gyro + (q*cos_phi_gyro - r*sin_phi_gyro)*dt;
  if(start==0){
    phi_gyro = phi_acc;
    th_gyro = th_acc;
    start = 1;
  }
}

void gyro_drift_correction(){
  phi_gyro = phi_gyro + k2*(phi_acc - phi_gyro);
  th_gyro = th_gyro + k2*(th_acc - th_gyro);
}

void calculate_motor_commands(){
  error = setpoint_angle - angle;
  I_error = I_error + ki*error;
  if(abs(error)<0.5){
    I_error = 0;
  }
  pwm = kp*error + kd*(error-previous_error) + I_error;
  previous_error = error;
  pwm = constrain(pwm,-150,150);
  pwm_l = pwm;
  pwm_r = pwm;
}

void drive_motors(){
  if(pwm_l>=0){
    analogWrite(enl,pwm_l);
    digitalWrite(ml1,HIGH);
    digitalWrite(ml2,LOW);
  }
  else if(pwm_l<0){
    analogWrite(enl,-pwm_l);
    digitalWrite(ml1,LOW);
    digitalWrite(ml2,HIGH);
  }

  if(pwm_r>=0){
    analogWrite(enr,pwm_r);
    digitalWrite(mr1,HIGH);
    digitalWrite(mr2,LOW);
  }
  else if(pwm_r<0){
    analogWrite(enr,-pwm_r);
    digitalWrite(mr1,LOW);
    digitalWrite(mr2,HIGH);
  }
}
