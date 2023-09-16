const int trigPin = 5;
const int echoPin = 2;

// define variables
long duration;
int distance;

void setup()
{
  pinMode(4, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
  digitalWrite(4. HIGH);
}

void loop()
{
  //put your main code
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034/2;
  Serial.println(distance);
  if (distance<8)
  {
    digitalWrite(4, HIGH)
  }
  else
  {
    digitalWrite(4, LOW)
  }
}