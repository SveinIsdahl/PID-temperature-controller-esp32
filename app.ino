
//Length of array
#define length 30
//Array for a moving average filter
float tempArray [length];
// Voltage out
int Vo;
float temperature;

void setup() {
  Serial.begin(9600);
  
  //Adding values to the array to make the filter stable quicker
  for (int i = 0; i < length; i++) {
    tempArray[i] = voltageToTemp(analogRead(36)-180);
  }

}

void loop() {
  
  Vo = analogRead(36)-180;
  temperature = voltageToTemp(Vo);

  //Wating to add values to the array
  //to make sure there are no wrong values from startup
  if (millis() > 2000) {
    addToArray();
  }
  
  Serial.println("Moving average:");
  Serial.println(averageOfArray(tempArray));
  Serial.print("\n");
  Serial.println("Temp-ish:");
  Serial.println(temperature);
  Serial.print("\n");
  Serial.println("Spenning (Vo):");
  Serial.println(Vo);
  Serial.print("\n");

  delay(300);
}

void addToArray() {
  tempArray[0] = temperature;
  for (int i = length-1; i >= 1; i--){
    tempArray[i] = tempArray[i-1];
  }
  
}

float averageOfArray(float array[]) {
  float sum = 0;
  for (int i = 0; i < length; i++) {
    sum += array[i];

  }
  return sum/length;
}

float voltageToTemp(int Vo){
  const float c1 = 0.001129148, c2 = 0.000234125, c3 = 0.0000000876741; //steinhart-hart coefficients
  const float R1 = 10000;
  float logR2, R2;

  R2 = R1 * (3330.0 / (float)Vo - 1.0); //Resistance in thermistor
  logR2 = log(R2);
  return (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2)) - 273.15; 
}