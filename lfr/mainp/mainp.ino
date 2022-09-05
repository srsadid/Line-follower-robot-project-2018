const int s1 = 2 ;
const int s2 = 4;
const int s3 = 7;
const int s4 = 8;
const int s5 = 12;
const int kp = 25;
int v1 = 255 ;
int v2 = 0 ;
void setup() {
  pinMode(s1, INPUT);
  pinMode(s2, INPUT);
  pinMode(s3, INPUT);
  pinMode(s4, INPUT);
  pinMode(s5, INPUT);
 Serial.begin(9600);

}





void loop() {
float error = 0 ;
   
   
  if   ((digitalRead(s1))&&(digitalRead(s2))&&(digitalRead(s3))&&(digitalRead(s4))&&(!digitalRead(s5)))  { error = 4;}
  else if((digitalRead(s1))&&(digitalRead(s2))&&(digitalRead(s3))&&(!digitalRead(s4))&&(!digitalRead(s5)))  { error = 3;}
  else if((digitalRead(s1))&&(digitalRead(s2))&&(digitalRead(s3))&&(!digitalRead(s4))&&(digitalRead(s5)))  { error = 2;}
  else if((digitalRead(s1))&&(digitalRead(s2))&&(!digitalRead(s3))&&(!digitalRead(s4))&&(digitalRead(s5)))  { error = 1;}
  else if((digitalRead(s1))&&(digitalRead(s2))&&(!digitalRead(s3))&&(digitalRead(s4))&&(digitalRead(s5)))  { error = 0;}
  else if((digitalRead(s1))&&(!digitalRead(s2))&&(!digitalRead(s3))&&(digitalRead(s4))&&(digitalRead(s5)))  { error = -1;}  
  else if((digitalRead(s1))&&(!digitalRead(s2))&&(digitalRead(s3))&&(digitalRead(s4))&&(digitalRead(s5)))  { error = -2;}
  else if((!digitalRead(s1))&&(!digitalRead(s2))&&(digitalRead(s3))&&(digitalRead(s4))&&(digitalRead(s5)))  { error = -3;}
  else if((!digitalRead(s1))&&(digitalRead(s2))&&(digitalRead(s3))&&(digitalRead(s4))&&(digitalRead(s5)))  { error = -4;}
   else if((digitalRead(s1))&&(digitalRead(s2))&&(digitalRead(s3))&&(digitalRead(s4))&&(digitalRead(s5)))  { error = 6.2;  }
  
  float rms = 100+error*kp;
 float lms = 100-error*kp;
   
   if (rms>255 ) {rms=255;}
   if (lms<0) {lms=0;}
  
   analogWrite (9,rms);
   analogWrite (10,0);
   analogWrite (5,lms);
   analogWrite (6,0);
  
  
  
}
  
