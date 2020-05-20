
// Length of array
#define length 30
#define pwm_pin 16

const int frequency = 50;
const int ledChannel = 0;
const int resolution = 8;

// Array for a moving average filter
float tempArray[length];
// Voltage out
int Vo;
int dutycycle;
float temperature;

class PID {
   public:
    // Constants
    int Kp = 40;
    int Ki = 0.5;
    float Kd = 0.1;
    float proportional = 0;
    float integral = 0;
    float derivative = 0;
    float elapsedTime, Time, prevTime;

    int setpoint = 30;
	int Vo;
    float error, prevError;
    float temperature, sum;

   public:
    void updateController() {
		Vo = analogRead(36) - 180;
		temperature = voltageToTemp(Vo);

        error = setpoint - temperature;

        proportional = Kp * error;

        /*
                Wind-up prevention:

        if ((error >= 0.001) || (error < 4)) {
            integral = integral + (Ki * error);
        } else {
            integral = 0;
        }
        
		*/
        integral = integral + (Ki * error);

        prevTime = Time;
        Time = millis();
        elapsedTime = (Time - prevTime);
        // Dy/Dx = (error2 - error1) /
        derivative = Kd * ((error - prevError) / elapsedTime);

        sum = proportional + integral + derivative;
        if (sum < 0) {
            dutycycle = 0;
        }
		else if (sum > 255) {
			dutycycle = 255;
		}
		else {
			dutycycle = sum;
		}
        prevError = error;
    }

   private:
    float voltageToTemp(int Vo) {
        const float c1 = 0.001129148, c2 = 0.000234125,
                    c3 = 0.0000000876741;  // steinhart-hart coefficients
        const float R1 = 10000;
        float logR2, R2;

        R2 = R1 * (3330.0 / (float)Vo - 1.0);  // Resistance in thermistor
        logR2 = log(R2);
        return (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2)) - 273.15;
    }
};

PID pid;

void setup() {
    Serial.begin(9600);
	ledcSetup(ledChannel, frequency, resolution);
	ledcAttachPin(pwm_pin, ledChannel);
    // Adding values to the array to make the filter stable quicker
    for (int i = 0; i < length; i++) {
        //tempArray[i] = pid.temperature;
    }
}

void loop() {
    pid.updateController();

    // Wating to add values to the array
    // to make sure there are no wrong values from startup
    if (millis() > 2000) {
        //addToArray();
    }
    Serial.println("Temp:");
    Serial.println(pid.temperature);
    //Serial.println(averageOfArray(tempArray));
    Serial.print("\n");
    Serial.println("Error:");
    Serial.println(pid.error);
    Serial.print("\n");
    Serial.println("Sum:");
    Serial.println(pid.sum);
    Serial.print("\n");
	Serial.println(dutycycle);

	ledcWrite(ledChannel, dutycycle);
    delay(200);
}

// Moving average filter
/*
void addToArray() {
    tempArray[0] = temperature;
    for (int i = length - 1; i >= 1; i--) {
        tempArray[i] = tempArray[i - 1];
    }
}

float averageOfArray(float array[]) {
    float sum = 0;
    for (int i = 0; i < length; i++) {
        sum += array[i];
    }
    return sum / length;
}
*/
