int ml1=2, ml2=3, mr1=4, mr2=7, enl=5, enr=6;

void setup(){
  pinMode(enl,OUTPUT);
  pinMode(enr,OUTPUT);
  pinMode(ml1,OUTPUT);
  pinMode(ml2,OUTPUT);
  pinMode(mr1,OUTPUT);
  pinMode(mr2,OUTPUT);
  
  analogWrite(enr,50);
  analogWrite(enl,50);
}

void hold(){
  digitalWrite(ml1,LOW);
  digitalWrite(ml2,LOW);
  digitalWrite(mr1,LOW);
  digitalWrite(mr2,LOW);
  delay(100);
}

void front(){
  digitalWrite(ml1,HIGH);
  digitalWrite(ml2,LOW);
  digitalWrite(mr1,HIGH);
  digitalWrite(mr2,LOW);
  delay(100);
}

void back(){
  digitalWrite(ml1,LOW);
  digitalWrite(ml2,HIGH);
  digitalWrite(mr1,LOW);
  digitalWrite(mr2,HIGH);
  delay(100);
}

void left(){
  digitalWrite(ml1,LOW);
  digitalWrite(ml2,HIGH);
  digitalWrite(mr1,HIGH);
  digitalWrite(mr2,LOW);
  delay(50);
}

void right(){
  digitalWrite(ml1,HIGH);
  digitalWrite(ml2,LOW);
  digitalWrite(mr1,LOW);
  digitalWrite(mr2,HIGH);
  delay(50);
}

void loop(){
  front();
  delay(5000);
  back();
  delay(5000);
  left();
  delay(5000);
  right();
  delay(5000);
  hold();
  delay(5000);
  }
