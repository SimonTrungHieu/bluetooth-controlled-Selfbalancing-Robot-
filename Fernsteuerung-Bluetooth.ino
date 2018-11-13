//--Backup fkt nur wenn angeschlossen werden----
#define button 12
#define button2 10
#define button3 7
#define button4 8
#define button5 5
#define button6 3
int buttonState = 0;
int buttonState2 = 0;
int buttonState3 = 0;
int buttonState4 = 0;
int buttonState5 = 0;
int buttonState6 = 0;
void setup() {
  pinMode(button, INPUT);
  pinMode(button2, INPUT);
  pinMode(button3, INPUT);
  pinMode(button4, INPUT);
  pinMode(button5, INPUT);
  pinMode(button6, INPUT);
  Serial.begin(9600); // Default communication rate of the Bluetooth module
}

void loop() {
 
 buttonState = digitalRead(button);
 buttonState2 = digitalRead(button2);
 buttonState3 = digitalRead(button3);
 buttonState4 = digitalRead(button4);
  buttonState5 = digitalRead(button5);
 buttonState6 = digitalRead(button6);
if (buttonState == LOW && buttonState2 == LOW && buttonState3 == LOW && buttonState4 == LOW && buttonState5 == LOW && buttonState6 == LOW) {
   Serial.println('0'); // Sends '0' to serialout
   delay(100);
   buttonState = LOW;
   buttonState2 = LOW;
   buttonState3 = LOW;
   buttonState4 = LOW;
   buttonState5 = LOW;
   buttonState6 = LOW;

   
   
 }
else if ((buttonState == HIGH && buttonState2 == LOW && buttonState3 == LOW && buttonState4 == LOW && buttonState5 == LOW && buttonState6 == LOW)) {
   Serial.println('1'); // Sends '1' to serialout
   
   buttonState = LOW;
   buttonState2 = LOW;
   buttonState3 = LOW;
   buttonState4 = LOW;
   buttonState5 = LOW;
   buttonState6 = LOW;
   delay(50);
 }
 
else if (buttonState == LOW && buttonState2 == HIGH && buttonState3 == LOW && buttonState4 == LOW && buttonState5 == LOW && buttonState6 == LOW) {
   Serial.println('2'); // Sends '2' to serialou
   
   buttonState = LOW;
   buttonState2 = LOW;
   buttonState3 = LOW;
   buttonState4 = LOW;
   buttonState5 = LOW;
   buttonState6 = LOW;
   delay(50);
 }
 else if (buttonState == LOW && buttonState2 == LOW && buttonState3 == HIGH && buttonState4 == LOW && buttonState5 == LOW && buttonState6 == LOW) {
   Serial.println('3'); // Sends '3' to serialout
   delay(50);
   buttonState = LOW;
   buttonState2 = LOW;
   buttonState3 = LOW;
   buttonState4 = LOW;
   buttonState5 = LOW;
   buttonState6 = LOW;
 }
  else if (buttonState == LOW && buttonState2 == LOW && buttonState3 == LOW && buttonState4 == HIGH && buttonState5 == LOW && buttonState6 == LOW) {
   Serial.println('4'); // Sends '4' to serialout
   delay(50);
   buttonState = LOW;
   buttonState2 = LOW;
   buttonState3 = LOW;
   buttonState4 = LOW;
   buttonState5 = LOW;
   buttonState6 = LOW;
 }
   else if (buttonState == LOW && buttonState2 == LOW && buttonState3 == LOW && buttonState4 == LOW && buttonState5 == HIGH && buttonState6 == LOW) {
   Serial.println('5'); // Sends '5' to serialout
   delay(50);
      buttonState = LOW;
   buttonState2 = LOW;
   buttonState3 = LOW;
   buttonState4 = LOW;
   buttonState5 = LOW;
   buttonState6 = LOW;
 }
   else if (buttonState == LOW && buttonState2 == LOW && buttonState3 == LOW && buttonState4 == LOW && buttonState5 == LOW && buttonState6 == HIGH) {
   Serial.println('6'); // Sends '6' to serialout
      delay(50);
   buttonState = LOW;
   buttonState2 = LOW;
   buttonState3 = LOW;
   buttonState4 = LOW;
   buttonState5 = LOW;
   buttonState6 = LOW;


 }
}
