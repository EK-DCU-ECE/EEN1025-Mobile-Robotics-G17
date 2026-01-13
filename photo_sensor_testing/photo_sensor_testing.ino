int AnalogValue[5] = {0,0,0,0,0};
int AnalogPin[5] = {4,5,6,7,15}; // keep 8 free for tone O/P music

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  int i;
  for (i=0;i<5;i++) {
    AnalogValue[i]=analogRead(AnalogPin[i]);
    Serial.print(AnalogValue[i]); // This prints the actual analog sensor reading
    Serial.print("\t"); //tab over on screen
    if(i==4) {
      Serial.println(""); //carriage return
      delay(600); // display new set of readings every 600mS
    }
  }
}
