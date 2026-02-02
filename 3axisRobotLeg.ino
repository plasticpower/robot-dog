#include <SCServo.h>

float load = 0;
int currentPos = 2048;
int i = 0;

SMS_STS st;

void stepForward();
void walkForward();
void positionCalculatorFrontRight();
void positionCalculatorFrontLeft();
void handleSerialCommand(const char* command);
void readSerial();




int lead1Ofset = 4095/8;
int follower1Ofset = -4095/8;

int center;

float thighLength = 60;
float shinLength = 131.23;

int targetX[8] = {0, 0, 0, 200, 0, 0, 0, -0};
int targetY[8] = {0, 0, 150, 0, 0, 150, 0, 0};
int targetZ[8] = {200, 80, 150, 130, 200, 80, 150, 130};

int currentTargetX = 0;
int currentTargetY = 0;
int currentTargetZ = 160;



int count = 0;


float legLength;

float servoOffset = 16.63;

float straightDistLeadServo;
float straightDistFollowServo;

float lead1Rad;
float follow1Rad;
float hip1Rad;

float lead2Rad;
float follow2Rad;
float hip2Rad;

float lead3Rad;
float follow3Rad;
float hip3Rad;

float lead4Rad;
float follow4Rad;
float hip4Rad;

float leadRad[4] = {0, 0, 0, 0};
float followRad[4] = {0, 0, 0, 0};
float hipRad[4] = {0, 0, 0, 0};

float lead1StraightRad;
float follow1StraightRad;

//float leadToStraightToFootRad;
//float followToStraightToFootRad;
//float leadThighStraightRad;
//float followThighStraightRad;
//float leadThighFromVertRad;
//float followThighFromVertRad;


float leadThighAngleFromVert;
float followThighAngleFromVert;
float leadThighAngleFromStraightToFoot;
float followThighAngleFromStraightToFoot;
float leadStraightAngleFromVert;
float followStraightAngleFromVert;

float lead1TargetX;

bool safeToMove = true;

float stepRadius = 20;
float circleAngle = 0;
float circleCenterX = 0;
float circleCenterZ = 160;



bool finalStep = false;
bool firstStep;

int walkingStepCount = 30;

int smallCircleCenterX;


float tick = 0;

bool walkingForwardBool = 0;

bool phase = 0;
int steppingLeg = 0;
int gaitOrder[4] = {0, 3, 2, 1};

int legPhaseOffset[4] = {0, 150, 100, 50}; //leg 1 steps first, then leg 4, then leg 2, then leg 3


float shiftY;

float shiftX[4] = {0, 0, 0, 0};

float ratioTickThruCycle;
float voltage = 120;
float current;
float temp;
// the uart used to control servos.
// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.
#define S_RXD 18
#define S_TXD 19

#define PI 3.14159

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;

  safeToMove = false;
 

  defaultPosition();

}
void enableTorque() {
  st.EnableTorque(1, 1);
  st.EnableTorque(2, 1);
  st.EnableTorque(3, 1);
  st.EnableTorque(4, 1);
  st.EnableTorque(5, 1);
  st.EnableTorque(6, 1);
  st.EnableTorque(7, 1);
  st.EnableTorque(8, 1);
  st.EnableTorque(9, 1);
  st.EnableTorque(10, 1);
  st.EnableTorque(11, 1);
  st.EnableTorque(12, 1);

}
void disableTorque() {
  st.EnableTorque(1, 0);
  st.EnableTorque(2, 0);
  st.EnableTorque(3, 0);
  st.EnableTorque(4, 0);
  st.EnableTorque(5, 0);
  st.EnableTorque(6, 0);
  st.EnableTorque(7, 0);
  st.EnableTorque(8, 0);
  st.EnableTorque(9, 0);
  st.EnableTorque(10, 0);
  st.EnableTorque(11, 0);
  st.EnableTorque(12, 0);

}
void getServoData() {
  voltage = st.ReadVoltage(1);
  current = st.ReadCurrent(1);
  temp = st.ReadTemper(1);

}
void writePositionToServo() {
  st.WritePosEx(1, 1970 + (leadRad[0] / (2*3.14159))*4095, 3000);
  st.WritePosEx(2, 1980 - (followRad[0] / (2*3.14159))*4095, 3000);
  st.WritePosEx(3, 2010 - (hipRad[0] / (2*3.14159))*4095, 3000);

  st.WritePosEx(4, 2010 - (leadRad[1] / (2*3.14159))*4095, 3000);
  st.WritePosEx(5, 1980 +  (followRad[1] / (2*3.14159))*4095, 3000);
  st.WritePosEx(6, 1955 - (hipRad[1] / (2*3.14159))*4095, 3000);

  st.WritePosEx(7, 2080 - (hipRad[3] / (2*3.14159))*4095, 3000);
  st.WritePosEx(8, 1980 - (leadRad[3] / (2*3.14159))*4095, 3000);
  st.WritePosEx(9, 2010 + (followRad[3] / (2*3.14159))*4095, 3000);

  st.WritePosEx(10, 1990 - (hipRad[2] / (2*3.14159))*4095, 3000);
  st.WritePosEx(11, 2060 + (leadRad[2] / (2*3.14159))*4095, 3000);
  st.WritePosEx(12, 2070 - (followRad[2] / (2*3.14159))*4095, 3000);
}
void defaultPosition() {
  st.WritePosEx(1, 1970, 300);
  st.WritePosEx(2, 1980, 300);
  st.WritePosEx(3, 2010, 300);
  st.WritePosEx(4, 2010, 300);
  st.WritePosEx(5, 1980, 300);
  st.WritePosEx(6, 1955, 300);
  st.WritePosEx(7, 2080, 300);
  st.WritePosEx(8, 1980, 300);
  st.WritePosEx(9, 2010, 300);
  st.WritePosEx(10, 1990, 300);
  st.WritePosEx(11, 2060, 300);
  st.WritePosEx(12, 2070, 300);
}
void findLegTarget(int legIndex) {
  int localTick = (int(tick) - legPhaseOffset[legIndex] + 200) % 200;

  if (localTick < 50) { // stepping phase (quarter of cycle)
    float t = localTick / 50.0;  // 0 → 1
    float angle = t * PI;

    currentTargetX = circleCenterX - stepRadius * cos(angle);
    currentTargetZ = circleCenterZ - stepRadius * sin(angle);
    currentTargetY = 0;

  } else { // stance / return phase
    float t = (localTick - 50) / 150.0; // 0 → 1
    float x = 1 - 2 * t;

    shiftX = 10 * sin(t * PI);
    

    currentTargetX = circleCenterX + x * stepRadius;
    currentTargetZ = circleCenterZ;
    currentTargetY = shiftX;
  }
}

void walkForward() {
  
  
  if (safeToMove == false) { //skips function if not safe to move
    defaultPosition();
    return;
  }
  
  findLegTarget(0);
  positionCalculator(0);

  findLegTarget(3);
  positionCalculator(3);

  findLegTarget(2);
  positionCalculator(2);

  findLegTarget(1);
  positionCalculator(1);

  writePositionToServo();

  Serial.print(current);
  Serial.print(" | ");
  Serial.print(voltage);
  Serial.print(" | ");
  Serial.print(tick);
  Serial.print(" | ");
  Serial.print(1980 + (leadRad[0] / (2*3.14159))*4095);
  Serial.print(" | ");
  Serial.print(1980 - (followRad[0] / (2*3.14159))*4095);
  Serial.print(" | ");
  Serial.print(2010 - (hipRad[0] / (2*3.14159))*4095);
  Serial.print(" | ");
  Serial.println(sin((tick / 100) * PI));
  tick += 1;

  

}
void handleSerialCommand(const char* command) {

  if (strcmp(command, "stepforward") == 0) {
    stepForward();
    return;
  }

  if (strcmp(command, "e") == 0) {
    safeToMove = false;
    walkingForwardBool = 0;
    delay(300);
    defaultPosition();
    return;
  }

  if (strcmp(command, "r") == 0) {
    safeToMove = true;
    return;
  }

  if (strcmp(command, "walkforward") == 0) {
    walkingForwardBool = 1;
    return;
  }

  
  char* end;
  long inputZ = strtol(command, &end, 10);

 
  if (*end != '\0') {
    return;
  }

  if (inputZ > 100 && inputZ < 180) {
    currentTargetZ = inputZ;
    

    positionCalculator(0);
    positionCalculator(1);
    positionCalculator(2);
    positionCalculator(3);

    if (safeToMove == true) {
      writePositionToServo();
    }
  }
}

 
char cmdBuf[30]; // buffer to hold characters
uint8_t idx = 0; 

void readSerial() {
    while (Serial.available()) {
        char c = Serial.read();

        if (c == '\n' || c == '\r') {
            cmdBuf[idx] = '\0';
            idx = 0;
            handleSerialCommand(cmdBuf);
        }
        else if (idx < sizeof(cmdBuf) - 1) {
            cmdBuf[idx++] = c;
        }
    }
}

void stepForward() {
  circleAngle = 2*PI;
  
  while (circleAngle >= 0) {

    if (Serial.available() > 0) {
    char cmd = Serial.read();
      if (cmd == 'e') {
        safeToMove = false;
        Serial.print("E clicked | ");
      
      }
      if (cmd == 'r') {
        safeToMove = true;
        Serial.print("R clicked | ");
      
      }
    
    }
    if (safeToMove == false) {
      defaultPosition();
      return;
    }
    
    
    currentTargetY = 0;
    currentTargetX = circleCenterX + stepRadius * cos(circleAngle) * 1.2;
    currentTargetZ = circleCenterZ + stepRadius * sin(circleAngle) * .5;
  
    positionCalculatorFrontLeft();
    
    //if (firstStep == true) {
      smallCircleCenterX = circleCenterX - stepRadius / 2;
      currentTargetY = 0;
      currentTargetX = circleCenterX + stepRadius * cos(circleAngle + PI/2) * 1.2;
      currentTargetZ = circleCenterZ + stepRadius * sin(circleAngle + PI/2) * .5;
      
    //} else {      
      //currentTargetY = 0;
      //currentTargetX = circleCenterX + stepRadius * cos(circleAngle + PI/2) * 1.2;
      //currentTargetZ = circleCenterZ + stepRadius * sin(circleAngle + PI/2) * .5;
    //}

    positionCalculatorFrontRight();

    currentTargetY = 0;
    currentTargetX = circleCenterX + stepRadius * cos(circleAngle + PI) * 1.2;
    currentTargetZ = circleCenterZ + stepRadius * sin(circleAngle + PI) * .5;

    positionCalculatorBackLeft();


    currentTargetY = 0;
    currentTargetX = circleCenterX + stepRadius * cos(circleAngle + (3*PI)/2) * 1.2;
    currentTargetZ = circleCenterZ + stepRadius * sin(circleAngle + (3*PI)/2) * .5;

    positionCalculatorBackRight();
    


  //if (currentTargetZ >= circleCenterZ) currentTargetZ = circleCenterZ;
  
    
    if (safeToMove == true) {
      writePositionToServo();
      

    //  if (finalStep == true) return;

      
    } else { //resets leg to default position
      defaultPosition();
      
    }

    Serial.print(circleCenterX + stepRadius * cos(circleAngle));
    Serial.print(" | ");
    Serial.print(circleCenterZ + stepRadius * sin(circleAngle));
    Serial.print(" | ");
    Serial.print(1980 + (lead1Rad / (2*3.14159))*4095);
    Serial.print(" | ");
    Serial.print(1980 - (follow1Rad / (2*3.14159))*4095);
    Serial.print(" | ");
    Serial.println(2010 - (hip1Rad / (2*3.14159))*4095);
    
    if (circleAngle > PI && circleAngle < 2*PI) {
      circleAngle -= PI/5;
    } else {
      circleAngle -= PI/30;
    }
    //circleAngle -= PI/15;
    delay(50);
    
  }
}

void positionCalculator(int legIndex) {
  float x = -currentTargetX;
  float y = currentTargetY;
  float z = currentTargetZ;
  if (legIndex == 2 or legIndex == 3) {
    y = -currentTargetY;

  } 
  straightDistLeadServo = sqrt(pow(x + servoOffset, 2) + (y * y) + (z * z)); // these also act as the effective z for use in atan2
  straightDistFollowServo = sqrt(pow(x - servoOffset, 2) + y * y + z * z);

  //float leadThighAngleFromVert;
  //float followThighAngleFromVert;
  //float leadThighAngleFromStraightToFoot;
  //float followThighAngleFromStraightToFoot;
  //float leadStraightAngleFromHoriz;
  //float followStraightAngleFromHoriz;

  leadStraightAngleFromVert = atan2(servoOffset + x, straightDistLeadServo);
  followStraightAngleFromVert = atan2(servoOffset - x, straightDistFollowServo);

 // --cosine rule--
  leadThighAngleFromStraightToFoot = acos((straightDistLeadServo * straightDistLeadServo + thighLength * thighLength - shinLength * shinLength) / (2 * straightDistLeadServo * thighLength));
  followThighAngleFromStraightToFoot = acos((straightDistFollowServo * straightDistFollowServo + thighLength * thighLength - shinLength * shinLength) / (2 * straightDistFollowServo * thighLength));

  leadThighAngleFromVert = leadThighAngleFromStraightToFoot - leadStraightAngleFromVert;
  followThighAngleFromVert = followThighAngleFromStraightToFoot - followStraightAngleFromVert;

  hipRad[legIndex] = atan2(y, z);
  leadRad[legIndex] = leadThighAngleFromVert - PI / 4;
  followRad[legIndex] = followThighAngleFromVert - PI / 4;
  //Serial.print(" | 1 ");
  //Serial.print(leadThighAngleFromVert - PI / 4);
  //Serial.print(" | 2 ");
  //Serial.print(followThighAngleFromVert - PI / 4);
  //Serial.print(" | Hip ");
  //Serial.println(atan2(y, z));
}
void positionCalculatorFrontLeft() {
    



    
    if (isnan(lead1Rad)) lead1Rad = 0; //makes sure the servos dont get bad 
    if (isnan(follow1Rad)) follow1Rad = 0;
    if (isnan(hip1Rad)) hip1Rad = 0;

    
    lead1Rad   = constrain(lead1Rad, -PI/5, PI/2);
    follow1Rad = constrain(follow1Rad, -PI/2, PI/5);
    hip1Rad    = constrain(hip1Rad, -PI/6, PI/6);
}

void positionCalculatorFrontRight() {
  

  if (isnan(lead2Rad)) lead2Rad = 0; //ensures servos don't recieve bad data
  if (isnan(follow2Rad)) follow2Rad = 0;
  if (isnan(hip2Rad)) hip2Rad = 0;

  lead2Rad = constrain(lead2Rad,-PI/5, PI/2);
  follow2Rad = constrain(follow2Rad, -PI/2, PI/5);
  hip2Rad = constrain(hip2Rad, -PI/6, PI/6);
}

void positionCalculatorBackLeft() {
  

  if (isnan(lead3Rad)) lead3Rad = 0; //ensures servos don't recieve bad data
  if (isnan(follow3Rad)) follow3Rad = 0;
  if (isnan(hip3Rad)) hip3Rad = 0;

  lead3Rad = constrain(lead3Rad,-PI/5, PI/2);
  follow3Rad = constrain(follow3Rad, -PI/2, PI/5);
  hip3Rad = constrain(hip3Rad, -PI/6, PI/6);
}

void positionCalculatorBackRight() {
  

  if (isnan(lead4Rad)) lead4Rad = 0; //ensures servos don't recieve bad data
  if (isnan(follow4Rad)) follow4Rad = 0;
  if (isnan(hip4Rad)) hip4Rad = 0;

  lead4Rad = constrain(lead4Rad,-PI/5, PI/2);
  follow4Rad = constrain(follow4Rad, -PI/2, PI/5);
  hip4Rad = constrain(hip4Rad, -PI/6, PI/6);
}


void loop() {
  // put your main code here, to run repeatedly:
  
  //for (int i = 1; i <=12; i++){
  //  int ping = st.Ping(i);
  //  Serial.println(ping);
  //  delay(200);
  //}

  readSerial();
  if (walkingForwardBool == 1 ) walkForward();

  if (tick == 200) {
    getServoData();  
  }
  if (voltage < 105) {
    safeToMove = 0;
    disableTorque();
  } else {
    enableTorque();
    safeToMove = 1;
  }
  if (tick >= 200) tick = 0; // resets tick if it hits 400. 
  delay(1);
}
