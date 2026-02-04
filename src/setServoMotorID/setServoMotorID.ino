

#include <SCServo.h>

SMS_STS st;

// the uart used to control servos.
// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.
#define S_RXD 18
#define S_TXD 19
int initialID = 12;
int targetID = 1;
void setup()
{
  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;
  delay(1000);
  st.unLockEprom(initialID);
  delay(100);
  st.writeByte(initialID, 5, targetID);
  delay(100);
  st.LockEprom(targetID);
  st.WritePosEx(targetID, 2048, 3400, 50);
}

void loop()
{
  
  
  delay(2000);
  
  st.WritePosEx(targetID, 2048, 1500, 50);
  delay(2000);
}
