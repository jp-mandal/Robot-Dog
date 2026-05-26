#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// --- PINS ---
#define TRIG_PIN 5
#define ECHO_PIN 18
#define LED_WALK 2    
#define LED_TURN 4    
#define LED_AVOID 16  

// --- CALIBRATION ---
const int HIP_C[]  = {50, 130, 70, 110}; 
const int KNEE_C[] = {5, 110, 15, 125};  
const int SH_RAW[] = {25, 60, 80, 60};   
const int MIRROR[] = {1, -1, 1, -1};     
const int H_CH[] = {0, 2, 4, 6}, K_CH[] = {1, 3, 5, 7}, S_CH[] = {8, 9, 10, 11};
int cur[12]; 

// --- TUNING ---
const int STEP_SIZE = 8;          
const int BACKWARD_STEP_SIZE = 4; 
const int LIFT_HEIGHT = 12; 
const int TURN_PULL = 15; 

// Task Handles
TaskHandle_t xWalkHandle = NULL;
TaskHandle_t xTurnHandle = NULL;

// --- SHARED FUNCTIONS ---
void setServo(int ch, int angle) {
  cur[ch] = angle;
  pwm.setPWM(ch, 0, map(constrain(angle, 0, 180), 0, 180, 120, 620));
}

void moveBatch(const int channels[], const int targets[], int n, int steps, int ms) {
  int starts[n];
  for (int i = 0; i < n; i++) starts[i] = cur[channels[i]];
  for (int s = 1; s <= steps; s++) {
    float t = (1.0 - cos((float)s / steps * PI)) / 2.0; 
    for (int i = 0; i < n; i++) setServo(channels[i], starts[i] + (int)((targets[i] - starts[i]) * t));
    vTaskDelay(pdMS_TO_TICKS(ms)); 
  }
}

void stand() {
  for (int i = 0; i < 4; i++) {
    setServo(H_CH[i], HIP_C[i]); 
    setServo(K_CH[i], KNEE_C[i]); 
    setServo(S_CH[i], SH_RAW[i]);
  }
}

// --- NEW TASK: TELEMETRY (For Timing Graph) ---
void TaskTelemetry(void *pvParameters) {
  Serial.begin(115200); // Use high baud for Serial Studio
  for (;;) {
    // Read LED states to visualize task execution
    int walk = digitalRead(LED_WALK);
    int turn = digitalRead(LED_TURN);
    int avoid = digitalRead(LED_AVOID);

    // Format: "WalkVal,TurnVal,AvoidVal"
    Serial.print(walk);
    Serial.print(",");
    Serial.print(turn);
    Serial.print(",");
    Serial.println(avoid);

    vTaskDelay(pdMS_TO_TICKS(20)); // High sampling rate for smooth graph
  }
}

// --- TASK 1: SENSING (PRIORITY 4) ---
void TaskSensing(void *pvParameters) {
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
  pinMode(LED_AVOID, OUTPUT);
  for (;;) {
    digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long duration = pulseIn(ECHO_PIN, HIGH, 30000);
    int distance = (duration > 0) ? (duration * 0.034 / 2) : 999;

    if (distance > 0 && distance < 15) {
      digitalWrite(LED_AVOID, HIGH);
      vTaskSuspend(xWalkHandle); 
      xTaskNotifyGive(xTurnHandle); 
      vTaskDelay(pdMS_TO_TICKS(2000)); 
      digitalWrite(LED_AVOID, LOW);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// --- TASK 2: WALKING (PRIORITY 3) ---
void TaskWalking(void *pvParameters) {
  pinMode(LED_WALK, OUTPUT);
  for (;;) {
    digitalWrite(LED_WALK, HIGH);
    int sA=0, sB=3, pA=1, pB=2; 
    for(int phase=0; phase<2; phase++) {
      int kCh[] = {K_CH[sA], K_CH[sB]}, kTg[] = {KNEE_C[sA]+MIRROR[sA]*LIFT_HEIGHT, KNEE_C[sB]+MIRROR[sB]*LIFT_HEIGHT};
      moveBatch(kCh, kTg, 2, 12, 20);
      int hCh[] = {H_CH[sA], H_CH[sB], H_CH[pA], H_CH[pB]};
      int hTg[] = {HIP_C[sA]+MIRROR[sA]*-1*STEP_SIZE, HIP_C[sB]+MIRROR[sB]*-1*STEP_SIZE, 
                   HIP_C[pA]+MIRROR[pA]*1*STEP_SIZE, HIP_C[pB]+MIRROR[pB]*1*STEP_SIZE};
      moveBatch(hCh, hTg, 4, 12, 20);
      int kPl[] = {KNEE_C[sA], KNEE_C[sB]};
      moveBatch(kCh, kPl, 2, 12, 20);
      vTaskDelay(pdMS_TO_TICKS(80));
      sA=1; sB=2; pA=0; pB=3;
    }
  }
}

// --- TASK 3: TURNING (PRIORITY 2) ---
void TaskTurning(void *pvParameters) {
  pinMode(LED_TURN, OUTPUT);
  int pairs[2][2] = {{0, 3}, {1, 2}};
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    digitalWrite(LED_WALK, LOW);
    digitalWrite(LED_TURN, HIGH);

    stand(); vTaskDelay(pdMS_TO_TICKS(400));
    
    for (int p = 0; p < 2; p++) {
      int legA = pairs[p][0], legB = pairs[p][1];
      int kCh[] = {K_CH[legA], K_CH[legB]}, kUp[] = {KNEE_C[legA]+MIRROR[legA]*LIFT_HEIGHT, KNEE_C[legB]+MIRROR[legB]*LIFT_HEIGHT};
      moveBatch(kCh, kUp, 2, 10, 15);
      int hCh[] = {H_CH[legA], H_CH[legB]}, hTg[] = {HIP_C[legA]+(MIRROR[legA]*-1*TURN_PULL), HIP_C[legB]+(MIRROR[legB]*-1*TURN_PULL)};
      moveBatch(hCh, hTg, 2, 10, 15);
      int kDown[] = {KNEE_C[legA], KNEE_C[legB]};
      moveBatch(kCh, kDown, 2, 10, 15);
      vTaskDelay(pdMS_TO_TICKS(50));
    }
    stand(); vTaskDelay(pdMS_TO_TICKS(500));
    
    digitalWrite(LED_TURN, LOW);
    vTaskResume(xWalkHandle); 
  }
}

void setup() {
  pwm.begin(); pwm.setPWMFreq(50);
  for (int l = 0; l < 4; l++) { cur[H_CH[l]]=HIP_C[l]; cur[S_CH[l]]=SH_RAW[l]; cur[K_CH[l]]=KNEE_C[l]; }
  stand();
  vTaskDelay(pdMS_TO_TICKS(1000));

  // Core 0 handles Sensing and Telemetry
  xTaskCreatePinnedToCore(TaskSensing, "Sense", 4096, NULL, 4, NULL, 0);
  xTaskCreatePinnedToCore(TaskTelemetry, "Log", 2048, NULL, 5, NULL, 0); // Highest priority for graph accuracy

  // Core 1 handles movement tasks
  xTaskCreatePinnedToCore(TaskWalking, "Walk", 4096, NULL, 3, &xWalkHandle, 1);
  xTaskCreatePinnedToCore(TaskTurning, "Turn", 4096, NULL, 2, &xTurnHandle, 1);
}

void loop() {}
