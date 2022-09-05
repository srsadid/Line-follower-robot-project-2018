int redled = 9;
int greenled = 10;
int blueled = 11;

void setup ()

{
  pinMode(redled , OUTPUT);
  pinMode(greenled, OUTPUT);
  pinMode(blueled, OUTPUT);
}


void loop ()

{
 digitalWrite(redled, HIGH);
 delay (100 );
 digitalWrite(redled, LOW);
 delay (100);
 digitalWrite(greenled, HIGH);
 delay (100 ); 
 digitalWrite(greenled, LOW);
 delay (100);
 digitalWrite(blueled, HIGH);
 delay (100);
 digitalWrite(blueled, LOW) ;
 delay(100);
}

