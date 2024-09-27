void setup() {
  pinMode(4,OUTPUT);//in 1 or 3
  pinMode(6,OUTPUT);// en A or B
  pinMode(5,OUTPUT);// in 2 or 4
  pinMode(8,INPUT);// input from other arduino, please never put this as Output even by mistake

}

void loop() {
  if(digitalRead(8) != 1){
    digitalWrite(4, HIGH);
    digitalWrite(5,LOW);
    analogWrite(6, 200);
    // put your main code here, to run repeatedly:

}
else{

while(1){
  digitalWrite(4,LOW);
  digitalWrite(5,LOW);
  analogWrite(6, 0);
}
}}
