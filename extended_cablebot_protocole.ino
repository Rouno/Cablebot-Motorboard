/* PC->OpenCM904 protocole :
   serialFrameIn = <instruction payload>|<instruction payload>|<instruction payload>\n
   instruction payload :
   <desintation>:<destination data>
   destination = "motorboard" OR "motorboard_cfg" OR "gimbalboard"  //if destination is "gimbalboard" then forward <destination data> to appropriate BT port
      <"motorboard">:<dynamixelID> <"Goal_Position" OR "Goal_Velocity" OR "Goal_PWM"> <value>;<"Goal_Position" OR "Goal_Velocity" OR "Goal_PWM"> <value>;
      <"gimbalboard">:<"yaw" OR "pitch" OR "roll"> <"angle"> <value>; ... ;<"yaw" OR "pitch" OR "roll"> <"angle"> <value>;<"get_height">;
      <"motorboard_cfg">:<"get_dynamixelIDs">;<"Present_Position">;<"Present_Velocity">;<"Present_Load">;<"Goal_Position">;<"Goal_Velocity">;<"Goal_PWM"><"set_home_offset">;
                         <"set_position_mode">;<"set_speed_mode">;<"set_pwm_mode">;<"set_compliant_mode">;<"reboot">;<"no_torque">;<"torque">

   OpenCM904->PC protocole :
   serialFrameOut = <feedback payload>\n
   feedback payload :
   <origin>:<origin data>
   origin = "motorboard" OR "gimbalboard" OR "LOG"
      <"motorboard">:<dynamixelID> <"Present_Position" OR "Present_Velocity" OR "Present_Load"> <value>;<dynamixelID> <"Present_Position" OR "Present_Velocity" OR "Present_Load"> <value>;<"dynamixelIDs"> <X Y ... Z>;
      <"gimbalboard">:<"yaw" OR "pitch" OR "roll"> <"angle"> <value>; ... ;<"yaw" OR "pitch" OR "roll"> <"angle"> <value>;<"height"> <value>;
      <"LOG">:<log message>
*/

#include <DynamixelWorkbench.h>


#if defined(__OPENCM904__)
#define DEVICE_NAME "1" //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
#elif defined(__OPENCR__)
#define DEVICE_NAME ""
#endif

#define BAUDRATE  4500000

#define POSITION_MODE 4
#define SPEED_MODE 1
#define PWM_MODE 16
#define COMPLIANT_MODE 15

#define MAX_DYNAMIXEL_CNT 5
#define MAX_DYNAMIXEL_ID 10

int8_t* dynIdx; //array that stores dynamixel IDs depending on scan
int dynCnt = 0; //size of dynIdx

const String motorboard_dest = "motorboard";
const String motorboard_cfg_dest = "motorboard_cfg";
const String gimbalboard_dest = "gimbalboard";

DynamixelWorkbench dxl_wb;

void setup()
{
  Serial.begin(57600);
  //Serial3.begin(9600);  //Bluetooth to gimbal opebnCM
  while (!Serial); // If this line is activated, you need to open Serial Terminal
  dxl_wb.begin(DEVICE_NAME, BAUDRATE);
  scanIDs(&dxl_wb, &dynIdx, &dynCnt);  //scan all dynamixel IDs on the BUS -> uses constants MAX_DYNAMIXEL_CNT and MAX_DYNAMIXEL_ID
  pingIDs(&dxl_wb, dynIdx, dynCnt);    //need to ping all dynamixel before sending any instruction
}

void loop()
{
  String serialFrameIn = "";

  if (Serial.available()) {
    serialFrameIn = Serial.readStringUntil('\n');

    while (serialFrameIn.length() != 0) {
      String instructionPayload = getNextItem(&serialFrameIn, '|');
      String instructionDest = getNextItem(&instructionPayload, ':');

      if (instructionDest == motorboard_dest) {
        String motorPacket = "";
        while (instructionPayload.length() != 0) {
          motorPacket = getNextItem(&instructionPayload, ';');
          processMotorPacket(&dxl_wb, motorPacket);
        }
        continue;
      }
      if (instructionDest == motorboard_cfg_dest) {
        String configPacket = "";
        while (instructionPayload.length() != 0) {
          configPacket = getNextItem(&instructionPayload, ';');
          processConfigPacket(&dxl_wb, configPacket, dynIdx, dynCnt);
        }
        continue;
      }
      if (instructionDest == gimbalboard_dest) {
        //To do explicitely
        continue;
      }
      Serial.println("LOG:wrong destination in instruction payload");
    }
  }
}

//function to scan dynamixel IDs and store them in a global variable array, storing also size in a global variable
void scanIDs(DynamixelWorkbench *wb, int8_t** IDarray, int* cnt_ptr) {
  const char *log = NULL;
  bool result = false;

  uint8_t scanned_id[MAX_DYNAMIXEL_CNT];
  uint8_t dxl_cnt = 0;
  uint8_t range = MAX_DYNAMIXEL_ID;//highest ID number possible

  Serial.println("LOG:Wait for scan...");
  result = (*wb).scan(scanned_id, &dxl_cnt, range, &log);
  if (result == false)
  {
    Serial.print("LOG:");
    Serial.println(log);
    Serial.println("LOG:Failed to scan");
  }
  else
  {
    *IDarray = new int8_t [dxl_cnt];
    *cnt_ptr = (int) dxl_cnt;
    for (int cnt = 0; cnt < dxl_cnt; cnt++)
    {
      (*IDarray)[cnt] = scanned_id[cnt];
      Serial.println("LOG:ID " + String(scanned_id[cnt]));
    }
  }
}

//function to ping all dynamixels, mandatory before sending instructions
void pingIDs(DynamixelWorkbench *wb, int8_t* IDarray, int cnt) {
  for (int i = 0; i < cnt; i++) {
    const char *log;
    bool result;
    result = dxl_wb.ping(IDarray[i]);
    if (result == false)
    {
      Serial.println(log);
      Serial.println("Failed to ping " + String(IDarray[i]));
    }
  }
}

//get a substring from buffer from beginning until first separator occurence, erase substring from buffer including separator
String getNextItem(String *buffer, char separator) {
  String result = "";
  int separatorIdx = (*buffer).indexOf(separator);
  if (separatorIdx == -1) {                             //if no more separator, return a copy of buffer and erase its content
    result = *buffer;
    *buffer = "";
  } else {
    result = (*buffer).substring(0, separatorIdx);      //extract first substring
    (*buffer).remove(0, separatorIdx + 1);              //erase substring and separator from buffer
  }
  return result;
}


/*
   Packet level management
*/

void processMotorPacket(DynamixelWorkbench *wb, String packet) {
  int dxl_id = (getNextItem(&packet, ' ')).toInt();
  String value_type = getNextItem(&packet, ' ');
  int32_t value = getNextItem(&packet, ' ').toInt();;
  if (packet.length() != 0) {
    Serial.println("LOG:wrong Motor Packet format");
  } else {
    writeDxlValue(wb, dxl_id, value_type, value);
  }
}

void processConfigPacket(DynamixelWorkbench *wb, String instruction, int8_t IDs[], int nbIDs) {
  String serialFrameOut = "";
  if (instruction == "get_dynamixelIDs") {
    serialFrameOut += motorboard_dest + ":";
    for (int i = 0; i < nbIDs; i++) serialFrameOut += String(IDs[i]) + ';';
    Serial.println(serialFrameOut);
    return;
  }

  if (instruction == "Present_Position" || instruction == "Present_Velocity" || instruction == "Present_Load" ||
      instruction == "Goal_Position" || instruction == "Goal_Velocity" || instruction == "Goal_PWM") {
    serialFrameOut += motorboard_dest + ":";
    for (int i = 0; i < nbIDs; i++) {
      serialFrameOut += String(IDs[i]) + " " + instruction + " " + String(readDxlValue(wb, IDs[i], instruction)) + ";";
    }
    Serial.println(serialFrameOut);
    return;
  }

  if (instruction == "set_position_mode" || instruction == "set_speed_mode" ||
      instruction == "set_pwm_mode" || instruction == "set_compliant_mode") {
    const char *log;
    for (int i = 0; i < nbIDs; i++) {
      writeDxlValue(wb, IDs[i], "Torque_Enable", 0);
      writeDxlValue(wb, IDs[i], "Operating_Mode", modeValue(instruction));
      writeDxlValue(wb, IDs[i], "Torque_Enable", 1);
    }
    Serial.println("LOG:" + instruction + " on all dynamixel");
    return;
  }

  if (instruction == "set_home_offset") {
    for (int i = 0; i < nbIDs; i++) {
      int32_t current_position = readDxlValue(wb, IDs[i], "Present_Position"); //read current dynamixel position to offset
      int32_t current_offset = readDxlValue(wb, IDs[i], "Homing_Offset");     //read current dynamixel offset to take it into account
      writeDxlValue(wb, IDs[i], "Torque_Enable", 0);
      writeDxlValue(wb, IDs[i], "Homing_Offset", current_offset - current_position);
      writeDxlValue(wb, IDs[i], "Torque_Enable", 1);
    }
    Serial.println("LOG:" + instruction + " on all dynamixel");
    return;
  }

  if (instruction == "no_torque") {
    for (int i = 0; i < nbIDs; i++) {
      writeDxlValue(wb, IDs[i], "Torque_Enable", 0);
    }
    Serial.println("LOG:" + instruction + " on all dynamixel");
    return;
  }

  if (instruction == "torque") {
    for (int i = 0; i < nbIDs; i++) {
      writeDxlValue(wb, IDs[i], "Torque_Enable", 1);
    }
    Serial.println("LOG:" + instruction + " on all dynamixel");
    return;
  }

  if (instruction == "reboot") {
    for (int i = 0; i < nbIDs; i++) {
      (*wb).reboot(IDs[i]);
    }
    Serial.println("LOG:" + instruction + " all dynamixel");
    return;
  }

  Serial.println("LOG:wrong config Packet format");
}

int32_t readDxlValue(DynamixelWorkbench *wb, int8_t id, String type) {
  const char *log;
  int32_t read_item;
  bool result = (*wb).itemRead(id, type.c_str(), &read_item, &log);
  if (!result) {
    Serial.print("LOG:");
    Serial.println(log);
  }
  return read_item;
}

void writeDxlValue(DynamixelWorkbench *wb, int8_t id, String type, int32_t value) {
  const char *log;
  bool result = (*wb).itemWrite(id, type.c_str(), value, &log);
  if (!result) {
    Serial.print("LOG:");
    Serial.println(log);
  }
}

int32_t modeValue(String mode) {
  if (mode == "set_position_mode") return POSITION_MODE;
  if (mode == "set_speed_mode") return SPEED_MODE;
  if (mode == "set_pwm_mode") return PWM_MODE;
  if (mode == "set_compliant_mode") return COMPLIANT_MODE;
  return 0; //to avoid blocking function, but will trigger an error in dynamixel
}
