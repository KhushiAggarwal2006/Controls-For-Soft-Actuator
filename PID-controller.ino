#include <Wire.h>
#include <INA226.h>

// ============================================================
//  Hardware / sensor configuration
// ============================================================
INA226 ina(0x40);

#define LPWM_PIN 26
#define RPWM_PIN 27

const float    V_SOURCE       = 10.0f;
const uint32_t PWM_FREQ       = 20000;
const uint8_t  PWM_RESOLUTION = 8;
const uint16_t PWM_MAX        = 255;

const float    SAMPLE_RATE_HZ   = 400.0f;
const uint32_t SENSOR_PERIOD_US = (uint32_t)(1000000.0f / SAMPLE_RATE_HZ + 0.5f);

const uint32_t CONTROL_PERIOD_US = SENSOR_PERIOD_US;  // 100 Hz, Ts = 10 ms

//#define PROFILE_WITH_GPIO
//#define PROFILE_PIN 25

// ============================================================
//  Shared buffer - written by sensorTask, read by controlISR.
//  20-sample rolling window at 400 Hz = 50 ms of history.
//  rollingAvg is updated incrementally (O(1)) on every sensor tick
//  so controlISR only needs to read a single float.
// ============================================================
#define BUFFER_SIZE 20   // not a power of two - use modulo increment below

volatile float   shuntBuffer[BUFFER_SIZE] = {};
volatile uint8_t bufferIndex = 0;
volatile float   rollingAvg  = 0.0f;
static portMUX_TYPE bufferMux = portMUX_INITIALIZER_UNLOCKED;

// ============================================================
//  Timers + task handle
// ============================================================
hw_timer_t  *sensorTimer  = NULL;
hw_timer_t  *controlTimer = NULL;
TaskHandle_t sensorTaskHandle = NULL;

// ============================================================
//  PID state
// ============================================================
float Kp = 50000.0f;
float Ki = 0.0f;
float Kd = 100.0f;

float integral   = 0.0f;
float error_prev = 0.0f;

const float Ts = CONTROL_PERIOD_US * 1e-6f;   // 0.01 s

struct LogEntry {
    uint32_t time_us;
    float reference;
    float measurement;
    float duty;
    float error;
};

const uint32_t LOG_SIZE = 5000;

volatile LogEntry logBuffer[LOG_SIZE];
volatile uint32_t logIndex = 0;
volatile bool loggingFinished = false;

volatile float deltaV = 0.0f;

const float REF_AMPLITUDE = 0.02f;
const float REF_FREQ      = 1.0f;

volatile uint32_t controlExecCycles_last = 0;
volatile uint32_t controlExecCycles_max  = 0;
volatile uint32_t controlOverrunCount    = 0;

// ============================================================
//  INTERRUPT 1: sensor trigger
// ============================================================
void IRAM_ATTR sensorISR()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(sensorTaskHandle, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

// ============================================================
//  Sensor task
// ============================================================
void sensorTask(void *pvParameters)
{
    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (!ina.isConversionReady()) {
            continue;
        }

        float shunt = ina.getShuntVoltage_mV() / 100.0f;  // Amps, R = 0.1 Ω

        // Read the value being evicted BEFORE taking the lock.
        // Safe because bufferIndex and shuntBuffer are only written here
        // (single writer), so this is a stable snapshot.
        float oldVal = shuntBuffer[bufferIndex];

        portENTER_CRITICAL(&bufferMux);
            shuntBuffer[bufferIndex] = shunt;
            bufferIndex = (bufferIndex + 1 >= BUFFER_SIZE) ? 0 : bufferIndex + 1;
            // Incremental O(1) update: add new sample, subtract evicted sample
            rollingAvg += (shunt - oldVal) * (1.0f / BUFFER_SIZE);
        portEXIT_CRITICAL(&bufferMux);
    }
}

// ============================================================
//  INTERRUPT 2: controller (100 Hz)
// ============================================================
void IRAM_ATTR controlISR()
{
    uint32_t cyclesStart = ESP.getCycleCount();
#ifdef PROFILE_WITH_GPIO
    digitalWrite(PROFILE_PIN, HIGH);
#endif

    // Read the pre-computed rolling average - single float read
    float measurement;
    portENTER_CRITICAL_ISR(&bufferMux);
        measurement = rollingAvg;
    portEXIT_CRITICAL_ISR(&bufferMux);timer

    // 2. Reference signal
    float t = micros() * 1e-6f;

    float reference;
    if (t < 2.0f) {
        reference = 0.0f;
    } else {
        reference = 0.006f;
    }

    // 3. Error
    float error = reference - measurement;

    // 4. PID
    integral += error * Ts;
    float derivative = (error - error_prev) / Ts;

    float Vdesired = Kp * error + Ki * integral + Kd * derivative;
    Vdesired += deltaV;

    error_prev = error;

    // 5. Saturation + anti-windup
    if (Vdesired > V_SOURCE) {
        Vdesired = V_SOURCE;
        if (error > 0.0f)
            integral -= error * Ts;
    }

    if (Vdesired < -V_SOURCE) {
        Vdesired = -V_SOURCE;
        if (error < 0.0f)
            integral -= error * Ts;
    }

    float duty = fabsf(Vdesired) / V_SOURCE;
    if (duty > 1.0f)
        duty = 1.0f;

    uint16_t pwmCounts = (uint16_t)(duty * PWM_MAX);

    if (Vdesired >= 0.0f) {
        ledcWrite(RPWM_PIN, pwmCounts);
        ledcWrite(LPWM_PIN, 0);
    } else {
        ledcWrite(RPWM_PIN, 0);
        ledcWrite(LPWM_PIN, pwmCounts);
    }

    if (!loggingFinished && logIndex < LOG_SIZE) {
        logBuffer[logIndex].time_us    = micros();
        logBuffer[logIndex].reference  = reference;
        logBuffer[logIndex].measurement = measurement;
        logBuffer[logIndex].duty       = duty;
        logBuffer[logIndex].error      = error;
        logIndex++;
    } else {
        loggingFinished = true;
        timerStop(controlTimer);
    }

#ifdef PROFILE_WITH_GPIO
    digitalWrite(PROFILE_PIN, LOW);
#endif
    uint32_t cycles = ESP.getCycleCount() - cyclesStart;
    controlExecCycles_last = cycles;
    if (cycles > controlExecCycles_max) {
        controlExecCycles_max = cycles;
    }
    if (cycles > (uint32_t)CONTROL_PERIOD_US * 240) {
        controlOverrunCount++;
    }
}

// ============================================================
//  Setup
// ============================================================
void setup()
{
    Serial.begin(115200);
    Wire.begin();
    ina.begin();

#ifdef PROFILE_WITH_GPIO
    pinMode(PROFILE_PIN, OUTPUT);
    digitalWrite(PROFILE_PIN, LOW);
#endif

    ledcAttach(RPWM_PIN, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(LPWM_PIN, PWM_FREQ, PWM_RESOLUTION);

    xTaskCreatePinnedToCore(
        sensorTask, "sensorTask", 4096, NULL,
        configMAX_PRIORITIES - 1,
        &sensorTaskHandle,
        1
    );

    sensorTimer = timerBegin(1000000);
    timerAttachInterrupt(sensorTimer, &sensorISR);
    timerAlarm(sensorTimer, SENSOR_PERIOD_US, true, 0);

    controlTimer = timerBegin(1000000);
    timerAttachInterrupt(controlTimer, &controlISR);
    timerAlarm(controlTimer, CONTROL_PERIOD_US, true, 0);
}

// ============================================================
//  Loop
// ============================================================
void loop()
{
    static bool dumped = false;

    if (loggingFinished && !dumped)
    {
        dumped = true;

        for (uint32_t i = 0; i < logIndex; i++)
        {
            Serial.print(logBuffer[i].time_us);
            Serial.print(",");
            Serial.print(logBuffer[i].reference, 5);
            Serial.print(",");
            Serial.print(logBuffer[i].measurement, 5);
            Serial.print(",");
            Serial.print(logBuffer[i].duty, 5);
            Serial.print(",");
            Serial.println(logBuffer[i].error, 5);
        }
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
}