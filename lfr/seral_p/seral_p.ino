const int s1 = 12 ;
const int s2 = 8;
const int s3 = 7;
const int s4 = 4;
const int s5 = 2;

void setup() {
  pinMode(s1, INPUT);
  pinMode(s2, INPUT);
  pinMode(s3, INPUT);
  pinMode(s4, INPUT);
  pinMode(s5, INPUT);
 Serial.begin(9600);

}





void loop() {
int error = 0 ;
 
  if   ((digitalRead(s1))&&(digitalRead(s2))&&(digitalRead(s3))&&(digitalRead(s4))&&(!digitalRead(s5)))  { error = 4;}
  else if((digitalRead(s1))&&(digitalRead(s2))&&(digitalRead(s3))&&(!digitalRead(s4))&&(!digitalRead(s5)))  { error = 3;}
  else if((digitalRead(s1))&&(digitalRead(s2))&&(digitalRead(s3))&&(!digitalRead(s4))&&(digitalRead(s5)))  { error = 2;}
  else if((digitalRead(s1))&&(digitalRead(s2))&&(!digitalRead(s3))&&(!digitalRead(s4))&&(digitalRead(s5)))  { error = 1;}
  else if((digitalRead(s1))&&(digitalRead(s2))&&(!digitalRead(s3))&&(digitalRead(s4))&&(digitalRead(s5)))  { error = 0;}
  else if((digitalRead(s1))&&(!digitalRead(s2))&&(!digitalRead(s3))&&(digitalRead(s4))&&(digitalRead(s5)))  { error = -1;}  
  else if((digitalRead(s1))&&(!digitalRead(s2))&&(digitalRead(s3))&&(digitalRead(s4))&&(digitalRead(s5)))  { error = -2;}
  else if((!digitalRead(s1))&&(!digitalRead(s2))&&(digitalRead(s3))&&(digitalRead(s4))&&(digitalRead(s5)))  { error = -3;}
  else if((!digitalRead(s1))&&(digitalRead(s2))&&(digitalRead(s3))&&(digitalRead(s4))&&(digitalRead(s5)))  { error = -4;}
  
  
  Serial.print (error);
  
}
  
