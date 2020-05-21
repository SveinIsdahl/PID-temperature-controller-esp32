
// Length of tempArray (MA-filter)
#define length 30
#define pwm_pin 16

//Pwm-signal settings
const int frequency = 50;
const int pwmChannel = 0;
const int resolution = 8;

int Vo;         // Voltage out
int dutycycle;  // From 0-255, 0%-100% power

class PID {
   public:
    int Vo;
    float temperature, sum, temperatureReading, error;

   private:
   	//Constants
    int Kp = 40;
    float Ki = 0.1;
    float Kd = 0.1;

    int setpoint = 25; //Setpoint = target temperature
    float proportional = 0;
    float integral = 0;
    float derivative = 0;
    float elapsedTime, Time, prevTime, prevError;
    float tempArray[length];  // Array for a moving average filter

   public:
    void setup() {
        for (int i = 0; i < length; i++) {
            tempArray[i] = 20;
        }
    }
    void updateController() {
        Vo = analogRead(36) - 160;
        temperatureReading = voltageToTemp(Vo);

        // Wating to add values to the array
        // to make sure there are no wrong values from startup
        if (millis() > 4000) {
            temperature = averageOfArray(tempArray);
            addToArray();
        } else {
            temperature = temperatureReading;
        }

        error = setpoint - temperature;

        proportional = Kp * error;

        // Integral wind-up prevention:
        if ((error > -3) && (error < 3)) {
            integral = integral + (Ki * error);
        } else {
            integral = 0;
        }

        prevTime = Time;
        Time = millis();
        elapsedTime = (Time - prevTime);
        derivative = Kd * ((error - prevError) / elapsedTime);

        sum = proportional + integral + derivative;

        if (sum < 0) {
            dutycycle = 0;
        } else if (sum > 255) {
            dutycycle = 255;
        } else {
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

    // Add new temperature reading to beginning of the array, and discard the
    // last value in the array
    void addToArray() {
        tempArray[0] = temperatureReading;
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
};

PID pid;

void setup() {
    Serial.begin(9600);
    ledcSetup(pwmChannel, frequency, resolution);
    ledcAttachPin(pwm_pin, pwmChannel);
    // Adding values to the array to make the filter stable quicker
    pid.setup();
}

void loop() {
    pid.updateController();

    Serial.println("Temperature:");
    Serial.println(pid.temperature);
    Serial.println("Dutycycle:");
    Serial.println(dutycycle);
    Serial.println("Voltage Out:");
    Serial.println(pid.Vo);

    ledcWrite(pwmChannel, dutycycle);
    delay(500);
}
