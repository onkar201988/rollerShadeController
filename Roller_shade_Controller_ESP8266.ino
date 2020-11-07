#include"WifiandOTA.h"

//--------------------------- Pin Definition ---------------------------
const int limitSwAPin     = 5;
const int sensorAPin      = 4;
const int limitSwBPin     = 12;
const int sensorBPin      = 13;
const int ATN1Pin         = 10;
const int ATN2Pin         = 15;
const int BIN1Pin         = 16;
const int BIN2Pin         = 14;

const int upSwitchPin1    = 1;
const int downSwitchPin1  = 3;
const int upSwitchPin2    = 0;
const int downSwitchPin2  = 2;

//--------------------------- Variable declairation  --------------------
char msg[50];
volatile int masterCountA          = 0;  // Holds the current position of motor A count
volatile int masterCountB          = 0;  // Holds the current position of motor B count

volatile int nextPositionCountA    = 0;  // Number of counts to Up or Down requested for A
volatile int nextPositionCountB    = 0;  // Number of counts to Up or Down requested for B

int maxStepsA             = 22000;  // Lower limit for motor A count
int maxStepsB             = 22000;  // Lower limit for motor B count

volatile bool isLimitReachedA = false;
volatile bool isLimitReachedB = false;

volatile bool isUpSwPressed1  = false;
volatile bool isDownSwPressed1 = false;
volatile bool isUpSwPressed2  = false;
volatile bool isDownSwPressed2 = false;

volatile unsigned long upButtonPressedTimeA;    // Time stamp when button is pressed
volatile unsigned long upButtonReleasedTimeA;   // Time stamp when button is released
volatile bool upButtonPressedFlagA = false;     // Flag to store when button is pressed

volatile unsigned long downButtonPressedTimeA;  // Time stamp when button is pressed
volatile unsigned long downButtonReleasedTimeA; // Time stamp when button is released
volatile bool downButtonPressedFlagA = false;   // Flag to store when button is pressed

volatile unsigned long upButtonPressedTimeB;    // Time stamp when button is pressed
volatile unsigned long upButtonReleasedTimeB;   // Time stamp when button is released
volatile bool upButtonPressedFlagB = false;     // Flag to store when button is pressed

volatile unsigned long downButtonPressedTimeB;  // Time stamp when button is pressed
volatile unsigned long downButtonReleasedTimeB; // Time stamp when button is released
volatile bool downButtonPressedFlagB = false;   // Flag to store when button is pressed

const unsigned long longPressTime   = 1000; // Time for switch must be pressed

//---------------------- External object declairation -------------------
WiFiClient espClient;
PubSubClient client(espClient);

//----------------------- State machine variables -----------------------
enum masterState_E {
  CALIBRATION,
  ROTATE_MOTOR_UP,
  ROTATE_MOTOR_DOWN,
  SW_UP,
  SW_DOWN,
  IDEL_ST
};
volatile enum masterState_E masterStateA = CALIBRATION;
volatile enum masterState_E masterStateB = CALIBRATION;

enum calibrationState_E {
  INIT,
  UP_STATE,
  DONE
};

volatile enum calibrationState_E calibrationStateA = INIT;
volatile enum calibrationState_E calibrationStateB = INIT;

enum motorDirection_E {
  STOP,
  UP,
  DOWN
};

volatile enum motorDirection_E motorDirectionA = STOP;
volatile enum motorDirection_E motorDirectionB = STOP;

//--------------------------- ISR Function declairation -----------------
void ICACHE_RAM_ATTR sensorA_ISR();
void ICACHE_RAM_ATTR sensorB_ISR();
void ICACHE_RAM_ATTR limitSwA_ISR();
void ICACHE_RAM_ATTR limitSwB_ISR();

#ifndef debug
void ICACHE_RAM_ATTR upSwitchPin1_ISR();
void ICACHE_RAM_ATTR downSwitchPin1_ISR();
#endif
void ICACHE_RAM_ATTR upSwitchPin2_ISR();
void ICACHE_RAM_ATTR downSwitchPin2_ISR();

//---------------------------- Setup Function ---------------------------
void setup() {
#ifdef debug
  Serial.begin(115200);
#endif

  pinMode(limitSwAPin, INPUT);
  pinMode(limitSwAPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(limitSwAPin), limitSwA_ISR, FALLING);

  pinMode(limitSwBPin, INPUT);
  pinMode(limitSwBPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(limitSwBPin), limitSwB_ISR, FALLING);

  pinMode(sensorAPin, INPUT);
  pinMode(sensorAPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(sensorAPin), sensorA_ISR, FALLING);

  pinMode(sensorBPin, INPUT);
  pinMode(sensorBPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(sensorBPin), sensorB_ISR, FALLING);

#ifndef debug
  pinMode(upSwitchPin1, INPUT);
  pinMode(upSwitchPin1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(upSwitchPin1), upSwitchPin1_ISR, CHANGE);

  pinMode(downSwitchPin1, INPUT);
  pinMode(downSwitchPin1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(downSwitchPin1), downSwitchPin1_ISR, CHANGE);
#endif

  pinMode(upSwitchPin2, INPUT);
  pinMode(upSwitchPin2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(upSwitchPin2), upSwitchPin2_ISR, CHANGE);

  pinMode(downSwitchPin2, INPUT);
  pinMode(downSwitchPin2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(downSwitchPin2), downSwitchPin2_ISR, CHANGE);

  pinMode(ATN1Pin, OUTPUT);
  pinMode(ATN2Pin, OUTPUT);
  pinMode(BIN1Pin, OUTPUT);
  pinMode(BIN2Pin, OUTPUT);

  setup_wifi();
  setup_OTA();
  
  // setup connection with MQTT server 
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

//-------------------------- ISR functions --------------------------
//-------------------------------------------------------------------
void sensorA_ISR()
{
  nextPositionCountA --;

  if(UP == motorDirectionA)
  {
    masterCountA--;
  }
  else if(DOWN == motorDirectionA)
  {
    masterCountA++;
  }
  else
  {
    //Do nothing
  }

  if(0 == nextPositionCountA)
  {
    masterStateA = IDEL_ST;
  }
}

//-------------------------------------------------------------------
void sensorB_ISR()
{
  nextPositionCountB --;

  if(UP == motorDirectionB)
  {
    masterCountB--;
  }
  else if(DOWN == motorDirectionB)
  {
    masterCount+B+;
  }
  else
  {
    //Do nothing
  }
  nextPositionCountB --;
  if (0 == nextPositionCountB)
  {
    masterStateB = IDEL_ST;
  }
}

//-------------------------------------------------------------------
void limitSwA_ISR()
{
  if(!digitalRead(limitSwAPin))
  {
    #ifdef debug
      Serial.println("Limit SW A ISR: Limit SW pressed");
    #endif
    
    stopShadeA();  // Hard stop motor in any case
    isLimitReachedA = true;
    nextPositionCountA = 5;
    shadeDownA();
    masterCountA = 0;
  }
  else if(DONE == calibrationStateA)
  {
    isLimitReachedA = false;
  }
}

//-------------------------------------------------------------------
void limitSwB_ISR()
{
  if(!digitalRead(limitSwBPin))
  {
    #ifdef debug
      Serial.println("Limit SW B ISR: Limit SW pressed");
    #endif
    
    stopShadeB();  // Hard stop motor in any case
    isLimitReachedB = true;
    nextPositionCountB = 5;
    shadeDownB();
    masterCountB = 0;
  }
  else if(DONE == calibrationStateB)
  {
    isLimitReachedB = false;
  }
}

//-------------------------------------------------------------------
#ifndef debug
void upSwitchPin1_ISR()
{
  if(!digitalRead(upSwitchPin1))
  {
    isUpSwPressed1 = true;
    upButtonPressedTimeA = millis();
  }
  else
  {
    isUpSwPressed1 = false;
    upButtonReleasedTimeA = millis();
  }
}
#endif

#ifndef debug
//--------------------------------------------------------------------
void downSwitchPin1_ISR()
{
  if(!digitalRead(downSwitchPin1))
  {
    isDownSwPressed1 = true;
    downButtonPressedTimeA = millis();
  }
  else
  {
    isDownSwPressed1 = false;
    downButtonReleasedTimeA = millis();
  }
}
#endif

//---------------------------------------------------------------------
void upSwitchPin2_ISR()
{
  if(!digitalRead(upSwitchPin2))
  {
    isUpSwPressed2 = true;
    upButtonPressedTimeB = millis();
  }
  else
  {
    isUpSwPressed2 = false;
    upButtonReleasedTimeB = millis();
  }
}

//----------------------------------------------------------------------
void downSwitchPin2_ISR()
{
  if(!digitalRead(downSwitchPin2))
  {
    isDownSwPressed2 = true;
    downButtonPressedTimeB = millis();
  }
  else
  {
    isDownSwPressed2 = false;
    downButtonReleasedTimeB = millis();
  }
}

//--------------------------- Motor A ------------------------------------
void shadeUpA()
{
  // Set motor direction
  motorDirectionA = UP;
  digitalWrite(ATN1Pin, HIGH);
  digitalWrite(ATN2Pin, LOW);
}

//------------------------------------------------------------------------
void shadeDownA()
{
  // Set motor direction
  motorDirectionA = DOWN;
  digitalWrite(ATN1Pin, LOW);
  digitalWrite(ATN2Pin, HIGH);
}

//------------------------------------------------------------------------
void stopShadeA()
{
  // Set motor direction
  motorDirectionA = STOP;
  digitalWrite(ATN1Pin, LOW);
  digitalWrite(ATN2Pin, LOW);
}

//-------------------------- Motor B -------------------------------------
void shadeUpB()
{
  // Set motor direction
  motorDirectionB = UP;
  digitalWrite(BIN1Pin, HIGH);
  digitalWrite(BIN2Pin, LOW);
}

//------------------------------------------------------------------------
void shadeDownB()
{
  // Set motor direction
  motorDirectionB = DOWN;
  digitalWrite(BIN1Pin, LOW);
  digitalWrite(BIN2Pin, HIGH);
}

//------------------------------------------------------------------------
void stopShadeB()
{
  // Set motor direction
  motorDirectionB = STOP;
  digitalWrite(BIN1Pin, LOW);
  digitalWrite(BIN2Pin, LOW);
}

//----------------------- MQTT reconnect function --------------------------------
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    #ifdef debug
      Serial.print("Attempting MQTT connection...");
    #endif
    // Attempt to connect
    if (client.connect(mqtt_device_name, mqtt_uname, mqtt_pass)) {
      #ifdef debug
        Serial.println("connected");
      #endif
      client.subscribe(mqtt_topic_command);
    } else {
      #ifdef debug
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println(" try again in 5 seconds");
      #endif
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

//----------------------------- MQTT callback function -----------------------------
void callback(char* topic, byte* payload, unsigned int length)
{
  #ifdef debug
    Serial.print("Message arrived new [");
    Serial.print(topic);
    Serial.print("] ");
    for (int i = 0; i < length; i++) {
      Serial.print((char)payload[i]);
    }
    Serial.println();
  #endif

  payload[length] = '\0';                     // Make payload a string by NULL terminating it.
  int blindPosition = atoi((char *)payload);  // Convert string array to int 
  #ifdef debug
    Serial.print("blindPosition int = ");
    Serial.print(blindPosition);
    Serial.println();
  #endif

  // Constrain the value if it is out of range
  blindPosition = constrain(blindPosition, 0, 100);
  
  // Map the value from % to no. steps
  int nextPositionCount = map(blindPosition, 0, 100, 0, maxStepsA);

  // Set state for Motor A
  setNextPositionA(nextPositionCount);

  // Set state for Motor B
  setNextPositionB(nextPositionCount);
}

//-----------------------------------------------------------------------------
void sendShadeStatus()
{
  // Send the current state of the blind in %
  client.publish(mqtt_topic_state, String((int)(((float)masterCountA / (float)maxStepsA)*100)).c_str());
}

//-----------------------------------------------------------------------------
void setNextPositionA(int nextPosition)
{  
  if(nextPosition < masterCountA)
  {
    masterStateA = ROTATE_MOTOR_UP;
  }
  else if(nextPosition > masterCountA)
  {
    masterStateA = ROTATE_MOTOR_DOWN;
  }

  nextPositionCountA = abs(nextPosition - masterCountA);
}

//-----------------------------------------------------------------------------
void setNextPositionB(int nextPosition)
{  
  if(nextPosition < masterCountB)
  {
    masterStateB = ROTATE_MOTOR_UP;
  }
  else if(nextPosition > masterCountB)
  {
    masterStateB = ROTATE_MOTOR_DOWN;
  }

  nextPositionCountB = abs(nextPosition - masterCountB);
}

//------------------------------- Calibration A ----------------------------
void calibrationStateMachineA()
{
  switch (calibrationStateA) {
    case INIT:
      #ifdef debug
        Serial.println("Calibrating: INIT");
      #endif
      // Set direction to Up
      shadeUpA();
      calibrationStateA = UP_STATE;
      #ifdef debug
        Serial.println("Calibrating: UP_STATE");
      #endif
      break;

    case UP_STATE:
     // Run the loop until limit not reached
      if(isLimitReachedA)
      {
        stopShadeA();
        masterCountA = 0;
        calibrationStateA = DONE;
        isLimitReachedA = false;
        sendShadeStatus();
        #ifdef debug
          Serial.println("Calibration: DONE");
        #endif
      }
      else
      {
        // Do nothing
      }
      break;

    case DONE:
        masterStateA = IDEL_ST;
        break;

    default:
        break;
  }  
}

//------------------------------- Calibration B ----------------------------
void calibrationStateMachineB()
{
  switch (calibrationStateB) {
    case INIT:
      // Set direction to Up
      shadeUpB();
      calibrationStateB = UP_STATE;
      break;

    case UP_STATE:
     // Run the loop until limit not reached
      if(isLimitReachedB)
      {
        stopShadeB();
        masterCountB = 0;
        calibrationStateB = DONE;
        isLimitReachedB = false;
      }
      else
      {
        // Do nothing
      }
      break;

    case DONE:
        masterStateB = IDEL_ST;
        break;

    default:
        break;
  }  
}

//----------------------------------- Master State A -----------------------------------------
void masterStateMachineA()
{
  switch (masterStateA) {
    case CALIBRATION:
      calibrationStateMachineA();
      break;

    case ROTATE_MOTOR_UP:
      if(UP != motorDirectionA)
      {
        shadeUpA();
        #ifdef debug
          Serial.println("MasterStateA: ROTATE_MOTOR_UP");
        #endif
      }
      else
      {
        // do nothing
      }
      
      if(isUpSwPressed1 || isDownSwPressed1)
      {
        isUpSwPressed1 = false;
        isDownSwPressed1 = false;
        stopShadeA();
        masterStateA = IDEL_ST;
        sendShadeStatus();
      }
      break;
    
    case ROTATE_MOTOR_DOWN:
      if(DOWN != motorDirectionA)
      {
        shadeDownA();
        #ifdef debug
          Serial.println("MasterStateA: ROTATE_MOTOR_DOWN");
        #endif
      }
      else
      {
        // Do nothing
      }
      
      if(isUpSwPressed1 || isDownSwPressed1)
      {
        isUpSwPressed1 = false;
        isDownSwPressed1 = false;
        stopShadeA();
        masterStateA = IDEL_ST;
        sendShadeStatus();
      }
      break;

    case SW_UP:
      upSwitchPressedA();
      break;

    case SW_DOWN:
      downSwitchPressedA();
      break;

    case IDEL_ST:
      if(STOP != motorDirectionA)
      {
        stopShadeA();
        #ifdef debug
          Serial.println("MasterStateA: IDEL_ST");
        #endif
        sendShadeStatus();
      }
      else if (isUpSwPressed1)
      {
        masterStateA = SW_UP;
        #ifdef debug
          Serial.println("MasterStateA: SW_UP");
        #endif
      }
      else if (isDownSwPressed1)
      {
        masterStateA = SW_DOWN;
        #ifdef debug
          Serial.println("MasterStateA: SW_DOWN");
        #endif
      }
      else
      {
        // Do nothing
      }
      
      break;
  }
}

//------------------------------------- Master State B ----------------------------------
void masterStateMachineB()
{
  switch (masterStateB) {
    case CALIBRATION:
      calibrationStateMachineB();
      break;

    case ROTATE_MOTOR_UP:
      if(UP != motorDirectionB)
      {
        shadeUpB();
        #ifdef debug
          Serial.println("MasterStateB: ROTATE_MOTOR_UP");
        #endif
      }
      else if(isLimitReachedB)
      {
        stopShadeB();
        masterStateB = IDEL_ST;
        isLimitReachedB = false;
        #ifdef debug
          Serial.println("Limit B is reached");
        #endif
      }
      break;
    
    case ROTATE_MOTOR_DOWN:
      if(DOWN != motorDirectionB)
      {
        shadeDownB();
        #ifdef debug
          Serial.println("MasterStateB: ROTATE_MOTOR_DOWN");
        #endif
      }
      else
      {
        // Do nothing
      }
      break;

    case SW_UP:
      upSwitchPressedB();
      break;

    case SW_DOWN:
      downSwitchPressedB();
      break;

    case IDEL_ST:
      if(STOP != motorDirectionB)
      {
        stopShadeB();
        #ifdef debug
          Serial.println("MasterStateB: IDEL_ST");
        #endif
      }
      else if (isUpSwPressed2)
      {
        masterStateB = SW_UP;
        #ifdef debug
          Serial.println("MasterStateB: SW_UP");
        #endif
      }
      else if (isDownSwPressed2)
      {
        masterStateB = SW_DOWN;
        #ifdef debug
          Serial.println("MasterStateB: SW_DOWN");
        #endif
      }
      break;
  }
}

//-----------------------------------------------------------------------------
void upSwitchPressedA()
{
  // if switch is depressed quickly, then roll all the way up
  if(!isUpSwPressed1 && ((millis() - upButtonPressedTimeA) < longPressTime))
  {
    setNextPositionA(0);
  }
  else if(isUpSwPressed1 && ((millis() - upButtonPressedTimeA) > longPressTime))
  {
    if(10 >= masterCountA)
    {
      stopShadeA();
    }
    else if(UP != motorDirectionA)
    {
      shadeUpA();
    }
    else
    {
      // Do nothing
    }
  }
  else if(!isUpSwPressed1)
  {
    stopShadeA();
    masterStateA = IDEL_ST;
    sendShadeStatus();
  }
}

//-----------------------------------------------------------------------------
void downSwitchPressedA()
{
  // if switch is depressed quickly, then roll all the way up
  if(!isDownSwPressed1 && ((millis() - downButtonPressedTimeA) < longPressTime))
  {
    setNextPositionA(maxStepsA);
  }
  else if(isDownSwPressed1 && ((millis() - downButtonPressedTimeA) > longPressTime))
  {
    if((maxStepsA - 10) <= masterCountA)
    {
      stopShadeA();
    }
    else if(DOWN != motorDirectionA)
    {
      shadeDownA();
    }
    else
    {
      // Do nothing
    }
  }
  else if(!isDownSwPressed1)
  {
    stopShadeA();
    masterStateA = IDEL_ST;
    sendShadeStatus();
  }
}

//-----------------------------------------------------------------------------
void upSwitchPressedB()
{
  // if switch is depressed quickly, then roll all the way up
  if(!isUpSwPressed2 && ((millis() - upButtonPressedTimeB) < longPressTime))
  {
    setNextPositionB(0);
  }
  else if(isUpSwPressed2 && ((millis() - upButtonPressedTimeB) > longPressTime))
  {
    if(UP != motorDirectionB)
    {
      shadeUpB();
    }
    else
    {
      // Do nothing
    }
  }
  else if(!isUpSwPressed2)
  {
    stopShadeB();
    masterStateB = IDEL_ST;
  }
}

//-----------------------------------------------------------------------------
void downSwitchPressedB()
{
  // if switch is depressed quickly, then roll all the way up
  if(!isDownSwPressed2 && ((millis() - downButtonPressedTimeB) < longPressTime))
  {
    setNextPositionB(maxStepsB);
  }
  else if(isDownSwPressed2 && ((millis() - downButtonPressedTimeB) > longPressTime))
  {
    if(DOWN != motorDirectionB)
    {
      shadeDownB();
    }
    else
    {
      // Do nothing
    }
  }
  else if(!isDownSwPressed2)
  {
    stopShadeB();
    masterStateB = IDEL_ST;
  }
}

//---------------------------- Loop function ------------------------------
void loop() {
  // This function is called to receive OTA updates
  ArduinoOTA.handle();

  // Loop
  //-----------------------------
  masterStateMachineA();
  //masterStateMachineB();
  //-----------------------------

  // --------------- end Application code ---------------------
  // Required!!!, this is to check for new MQTT updates from server
  if((masterStateA != CALIBRATION) /*&& (masterStateB != CALIBRATION)*/)
  {
    // if MQTT connection lost, reconnect
    if (!client.connected())
    {
      masterStateA = IDEL_ST;
      masterStateB = IDEL_ST;
      yield();
      reconnect();
    }
    
    client.loop();
  }

  // Fixed delay of 10 mSec
  delay(10);
}
