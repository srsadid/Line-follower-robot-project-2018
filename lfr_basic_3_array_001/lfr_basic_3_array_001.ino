/*------ Arduino Line Follower Code----- */
/*-------definning Inputs------*/
#define LS 2      // left sensor
#define RS 3      // right sensor
#define MS 4
    int val_1 = 60;
    int val_2 = 10;
    int val_3 = 80;
    int val_4 = 0;
/*-------definning Outputs------*/
// 9-10 IN1-IN2 (right)
// 5-6 IN3-IN4 (left)
void setup()
{
  pinMode(LS, INPUT);
  pinMode(RS, INPUT);
  pinMode (MS,INPUT);
}

void loop()
{
  if((!digitalRead(LS)) && digitalRead(MS) &&  (!digitalRead(RS)))     // Move Forward
  {
    
    
   analogWrite (9,val_1);
   analogWrite (10,0);
   analogWrite (5,val_1);
   analogWrite (6,0);
  }
  
  if((!digitalRead(LS)) && digitalRead(MS) &&  (digitalRead(RS)))     // Turn right
  {
    
   analogWrite (9,val_2);
   analogWrite (10,0);
   analogWrite (5,val_1);
   analogWrite (6,0);
  }
  
  if((!digitalRead(LS)) && (!digitalRead(MS)) &&  (digitalRead(RS)))     //  extreme right
  {
    
   analogWrite (9,val_4);
   analogWrite (10,0);
   analogWrite (5,val_3);
   analogWrite (6,0);
  }
  
  if  (digitalRead(LS) && digitalRead(MS) &&  (!digitalRead(RS)))     // left
  {
   
    
   analogWrite (9,val_1);
   analogWrite (10,0);
   analogWrite (5,val_2);
   analogWrite (6,0);
  }
   if  (digitalRead(LS) && (!digitalRead(MS)) &&  (!digitalRead(RS)))     // extreme left
  {
   
    
   analogWrite (9,val_3);
   analogWrite (10,0);
   analogWrite (5,val_4);
   analogWrite (6,0);
  }
   if  (digitalRead(LS) && (!digitalRead(MS)) &&  (digitalRead(RS)))     // reverse forward
  {
   
    
   analogWrite (9,val_1);
   analogWrite (10,0);
   analogWrite (5,val_1);
   analogWrite (6,0);
  }
   if  ((!digitalRead(LS)) && (!digitalRead(MS)) &&  (!digitalRead(RS)))     // reverse slow*
  {
   
    
   analogWrite (9,val_2);
   analogWrite (10,0);
   analogWrite (5,val_2);
   analogWrite (6,0);
  }
   if  (digitalRead(LS) && digitalRead(MS) &&  (digitalRead(RS)))     // slow
  {
   
    
   analogWrite (9,val_2);
   analogWrite (10,0);
   analogWrite (5,val_2);
   analogWrite (6,0);
  }
}


