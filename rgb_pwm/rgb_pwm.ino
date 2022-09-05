int redled;
int greenled;
int blueled;

void setup ()

{
 pinMode(redled, OUTPUT);
 pinMode(greenled, OUTPUT);
 pinMode( blueled, OUTPUT);
}


void loop ()

{
 setColor (255, 0, 0);
 delay(500);
 setColor(0,255,0);
 delay(500);
 setColor (0,0,255);
 delay(500);
 setColor(150,0,0);
 delay(500);
 setColor(150,150,150);
 delay(500);
 setColor(125,125,125);
 delay(500);
 
}


void setColor (int red, int green, int blue)

{
 analogWrite(redled, red);
 analogWrite(greenled, green);
 analogWrite(blueled, blue);  
}

