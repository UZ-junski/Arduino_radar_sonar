#include <Servo.h> 
#include <Wire.h>
#include "vl53l1_api.h"
#define INTER_MEASUREMENT_PERIOD_MS 55
#define MEASUREMENT_BUDGET_MS 50
#define USE_BLOCKING_LOOP
#define ZERO_ULTRASONIC 10
#define ZERO_radar 49

Servo myservo;  
VL53L1_Dev_t dev;
VL53L1_DEV                     Dev = &dev;
int status;
int TX3 = 6; //port nadajnik Ultrasonic
int RX3 = 7; //port odbiornik Ultrasonic
int defaultPosition = 10;
int pos = defaultPosition;
bool canRun;

int CM;     //odległość w cm
long RADAR;
 
void setup()
{
  canRun=false;
  uint8_t byteData;
  uint16_t wordData;
  pinMode(TX3, OUTPUT);
  pinMode(RX3, INPUT);
  myservo.attach(9);
  myservo.write(defaultPosition);
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);

  Dev->I2cDevAddr = 0x52;
  VL53L1_software_reset(Dev);
  status = VL53L1_WaitDeviceBooted(Dev);
  status = VL53L1_DataInit(Dev);
  status = VL53L1_StaticInit(Dev);
  status = VL53L1_SetDistanceMode(Dev, VL53L1_DISTANCEMODE_LONG);
  status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(Dev, MEASUREMENT_BUDGET_MS * 1000);
  status = VL53L1_SetInterMeasurementPeriodMilliSeconds(Dev, INTER_MEASUREMENT_PERIOD_MS);
  status = VL53L1_StartMeasurement(Dev);
}
int pomiar_ulstra()
{
    digitalWrite(TX3, HIGH);
    delayMicroseconds(10);
    digitalWrite(TX3, LOW);
    int TIME3 = pulseIn(RX3, HIGH);
    int CMlocal=TIME3/58;
    return CMlocal;  
}
void pomiar_odleglosci_ultra_dzwiekowy() 
{  
//  int CMs[10];
//   for (int i = 0; i <= 10; i++) {    
//    int temp=2200;
//    int wait =10;
//    temp=pomiar_ulstra();      
//    
//    CMs[i] = temp;
//    delay(10);
//   }
//   int positionMax=0;
//   int positionTemp=0;
//   int cntMax=0;
//   int cntTemp=0;
//   for (int i = 0; i <= 10; i++) {
//    positionTemp=i;
//    cntTemp=0;
//      for (int j = 0; j <= 10; j++) {
//        if(CMs[i]<2000)
//        {
//            if(CMs[i]==CMs[j])
//            {
//              cntTemp++;
//            }          
//        }
//      }
//      if(cntTemp>cntMax)
//      {
//        cntMax=cntTemp;
//        positionMax=i;
//      }          
//   }
//   CM=CMs[positionMax];

CM=pomiar_ulstra();
delay(10);
}

void pomiar_odleglosci_radar()
{
    static uint16_t startMs = millis();
    uint8_t isReady;
  
    // non-blocking check for data ready
    status = VL53L1_GetMeasurementDataReady(Dev, &isReady);
  
    if(!status)
    {
      if(isReady)
      {
        printRangingData();
        VL53L1_ClearInterruptAndStartMeasurement(Dev);
        startMs = millis();
      }
      else if((uint16_t)(millis() - startMs) > VL53L1_RANGE_COMPLETION_POLLING_TIMEOUT_MS)
      {
        Serial.print(F("Timeout waiting for data ready."));
        VL53L1_ClearInterruptAndStartMeasurement(Dev);
        startMs = millis();
      }
    }
    else
    {
      Serial.print(F("Error getting data ready: "));
      Serial.println(status);
    } 
}

void printRangingData()
{
  static VL53L1_RangingMeasurementData_t RangingData;

  status = VL53L1_GetRangingMeasurementData(Dev, &RangingData);
  if(!status)
  {
//    Serial.print("Czujnik radarowy: ");  
    RADAR = RangingData.RangeMilliMeter;
//    Serial.println(RADAR);
  }
}
  
void loop()
{

    String message = "";
    while(Serial.available())
    {
        char c = Serial.read();
        if(isControl(c))
        {
          continue;
        }
        message += c;
    }
    if(message == "#START")
    {
      canRun=true;
      pos = defaultPosition; 
      myservo.write(pos);
      delay(1000); 
    }
    else if(message == "#STOP")
    {
      canRun=false;
      return; 
    }
  if(!canRun)
  {
    delay(500);
    return;
  }
  String result="";
  result+=millis();
  result+="|Kat czujnika: ";
  result+=pos;
  result+="|";
//  Serial.println(millis());
//  Serial.print("Kat czujnika: "); 
//  Serial.println(pos); 
  pomiar_odleglosci_radar();
  result+="Czujnik radarowy: ";
  result+=RADAR;
  result+="|";
  do
  {
    pomiar_odleglosci_ultra_dzwiekowy();  
    delay(10);
  }while(CM>2000||CM<0);
  
  // szerokość odbitego impulsu w uS podzielone przez
  // 58 to odleglosc w cm - patrz dokumentacja
  
//  Serial.print("Czujnik ultradzwiekowy: ");  
//  Serial.println(CM);
  result+="Czujnik ultradzwiekowy: ";
  result+=CM;
  result+="|";
  Serial.println(result);

  int dx=1;
  pos=pos+dx;
  if (pos > 180)
  {
    pos = defaultPosition; 
    myservo.write(pos);
    delay(1000); 
    return;
  }
  myservo.write(pos);              // ustawienie silnika serwo w pozycji "pos"
  delay(70); 
}
