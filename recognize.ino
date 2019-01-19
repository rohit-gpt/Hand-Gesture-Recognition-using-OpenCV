char data;

void setup() 
{ 
  Serial.begin(9600); 
  pinMode(LED_BUILTIN, OUTPUT); 
  pinMode(7, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);
  digitalWrite (LED_BUILTIN, LOW);
  digitalWrite(7, LOW);
  digitalWrite(6, LOW);
  digitalWrite(5, LOW); 
}
 
void loop() 
{

  while(!Serial.available()) {}

  while (Serial.available())
  {
    data = Serial.read();
    Serial.println(data);

  }


  if(data == '1') {
    digitalWrite(7, HIGH);
  }
  else if(data == '2') {
    digitalWrite(7, HIGH);
    digitalWrite(6, HIGH);
  }
  else if(data == '3') {
    digitalWrite(7, HIGH);
    digitalWrite(6, HIGH);
    digitalWrite(5, HIGH);
  }
  else {
//    digitalWrite(7, HIGH);
//    digitalWrite(6, HIGH);
//    digitalWrite(5, HIGH);
    digitalWrite(LED_BUILTIN, HIGH);
  }

  digitalWrite(7, LOW);
  digitalWrite(6, LOW);
  digitalWrite(5, LOW);
  digitalWrite(LED_BUILTIN, LOW);
  
}
