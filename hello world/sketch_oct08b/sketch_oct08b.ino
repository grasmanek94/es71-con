void setup() 
{
  Serial.begin(9600);
  Serial.println("Hello from arduino 2");
}

void loop()
{
    while(Serial.available())
    {
        Serial.write(Serial.read());
    }
}
