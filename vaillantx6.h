#include "esphome.h"
#include <string>

#define CMD_LENGTH 7
#define ANSWER_LENGTH 8
#define RETURN_TYPE_COUNT 3

typedef unsigned char byte;

void logCmd(const char *tag, byte *cmd)
{
  ESP_LOGD("Vaillantx6", "%s: 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x", tag, cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], cmd[6]);
}

enum VaillantReturnTypes
{
  None = 0,
  Temperature,
  SensorState,
  Bool,
  Minutes,
  Hours
};

uint8_t VaillantReturnTypeLength(VaillantReturnTypes t)
{
  switch (t)
  {
  case SensorState:
  case Bool:
  case Minutes:
    return 1;
  case Temperature:
    return 2;
  case Hours:
    return 2;
  default:
    return 0;
  }
}

float VaillantParseTemperature(byte *answerBuff, uint8_t offset)
{
  int16_t i = (answerBuff[offset] << 8) | answerBuff[offset + 1];
  return i / (16.0f);
}
float VaillantParseHours(byte *answerBuff, uint8_t offset)
{
  int16_t i = (answerBuff[offset] << 8) | answerBuff[offset + 1];
  return i;
}

int VaillantParseBool(byte *answerBuff, uint8_t offset)
{
  switch (answerBuff[offset])
  {
  case 0xF0:
  case 0x00:
    return 0;
  case 0x0F:
  case 0x01:
    return 1;
  default:
    ESP_LOGE("VaillantParseBool", "Unable to parse a bool from 0x%.2x", answerBuff[offset]);
    return -1;
  }
}

struct VaillantCommand
{
  std::string Name;
  byte Address;
  VaillantReturnTypes ReturnTypes[RETURN_TYPE_COUNT];
  // SensorID contains the ID of the sensor to use, corresponding to the ReturnType.
  // Use -1 to not assign a sensor.
  int SensorID[RETURN_TYPE_COUNT];
};

const VaillantCommand vaillantCommands[] = {
    {"Vorlauf Ist HK1", 0x18, {Temperature, SensorState, None}, {0, -1, -1}},//vermutlich Temperatur im Kesselkreislauf
    {"Vorlauf Set HK1", 0x19, {Temperature, None, None}, {1, -1, -1}},// Heizkreis 1 sollwert Dehregler, limitiert auch HK2
    {"Vorlauf Soll HK2", 0x39, {Temperature, None, None}, {2, -1, -1}},//Sollwert HK2 als min (VRC/789 und Limit von Drehregler)
    {"Vorlauf 789 Soll HK2", 0x25, {Temperature, None, None}, {3, -1, -1}},//Soll VL temp vom VRC Kreis 2
    {"Rücklauf Ist", 0x98, {Temperature, Temperature, SensorState}, {4, -1, -1}},//-13,9
    {"Brauchwasser Ist", 0x16, {Temperature, SensorState, None}, {5, -1, -1}},//immer 130
    {"Speichertemperatur soll", 0x01, {Temperature, None, None}, {6, -1, -1}},// Drehregler
    {"Speichertemperatur ist", 0x17, {Temperature, None, None}, {7, -1, -1}},//geht
    {"ExtVorRuecktemperatur", 0x6B, {Temperature, None, None}, {8, -1, -1}},//
    {"UnbekTemp", 0x9A, {Temperature, None, None}, {9, -1, -1}},//0,9C
    {"Aussentemp", 0x6A, {Temperature, None, None}, {10, -1, -1}},//3,8C
    {"Verbliebene Brennsperrzeit", 0x38, {Minutes, None, None}, {0, -1, -1}},//geht manchmal
    {"Stunden bis Wartung", 0xAC, {Hours, Hours, None}, {0, -1, -1}},//
    {"Brenner", 0x0d, {Bool, None, None}, {0, -1, -1}},//geht
    {"Winter", 0x08, {Bool, None, None}, {1, -1, -1}}, //geht
    {"Pumpe intern", 0x44, {Bool, None, None}, {2, -1, -1}},//bei brenner an geht pumpe intern aus
    {"Zirkulation", 0xAF, {Bool, None, None}, {3, -1, -1}},//signal vorhanden
    {"Status Ext Pumpe", 0x3F, {Bool, None, None}, {4, -1, -1}},//signal vorhanden = Pumpe intern vermutlich speicherladepumpe
    {"Status Unbekannt", 0x53, {Bool, None, None}, {5, -1, -1}},//geht nicht
    {"Status Speicherladepumpe", 0x9E, {Bool, None, None}, {6, -1, -1}},//geht nicht
};
const byte vaillantCommandsSize = sizeof(vaillantCommands) / sizeof *(vaillantCommands);

class Vaillantx6 : public PollingComponent,
                   public UARTDevice
{
  // Sensors as provided by custom_component lambda call
  Sensor *temperatureSensors[11];
  Sensor *minutesSensor[1];
  Sensor *hourSensor[1];
  BinarySensor *binarySensors[7];
  
  // All command start with startBytes sequence
  const byte startBytes[4] = {0x07, 0x00, 0x00, 0x00};

public:
  Vaillantx6(UARTComponent *parent,
             Sensor *tSensor0, Sensor *tSensor1, Sensor *tSensor2, Sensor *tSensor3,
             Sensor *tSensor4, Sensor *tSensor5, Sensor *tSensor6, Sensor *tSensor7,
			 Sensor *tSensor8, Sensor *tSensor9,Sensor *tSensor10,
			 Sensor *mSensor0,
             Sensor *hSensor0,
             BinarySensor *bSensor0, BinarySensor *bSensor1, BinarySensor *bSensor2, BinarySensor *bSensor3,
			 BinarySensor *bSensor4, BinarySensor *bSensor5, BinarySensor *bSensor6
			 )
      : PollingComponent(10000), UARTDevice(parent)
  {
    // Temperature Sensors
    temperatureSensors[0] = tSensor0; // Vorlauf ist
    temperatureSensors[1] = tSensor1; // Vorlauf set
    temperatureSensors[2] = tSensor2; // Vorlauf soll
    temperatureSensors[3] = tSensor3; // Vorlauf 789 soll
    temperatureSensors[4] = tSensor4; // Ruecklauf ist
    temperatureSensors[5] = tSensor5; // Brauchwasser ist
    temperatureSensors[6] = tSensor6; // Brauchwasser soll
    temperatureSensors[7] = tSensor7; // Speichertemperatur
    temperatureSensors[8] = tSensor8; // ExtVorRuecktemperatur
    temperatureSensors[9] = tSensor9; // UnbekTemp
    temperatureSensors[10] = tSensor10; // UnbekTemp
    // Minute sensors
    minutesSensor[0] = mSensor0; // Verbleibende Brennsperrzeit
    // Hour sensors
    hourSensor[0] = hSensor0; // Zeit bis wartung
    // Binary sensors
    binarySensors[0] = bSensor0; // Brenner
    binarySensors[1] = bSensor1; // Winter
    binarySensors[2] = bSensor2; // Pumpe intern
    binarySensors[3] = bSensor3; // Zirkulation
    binarySensors[4] = bSensor4; // Status Ext Pumpe
    binarySensors[5] = bSensor5; // Status Unbekannt
    binarySensors[6] = bSensor6; // Status Speicherladepumpe

    
  }

  /**
   * Compute the checksum used for Vaillant commands (and responses)
   *
   * @param data Array of bytes to compute the checksump for
   * @param len How many bytes of data to compute the checksum for
   * @return The 1 byte checksum
   **/
  byte checksum(byte *data, byte len)
  {
    byte checksum = 0;
    byte i = 0;
    for (i = 0; i < len; i++)
    {
      if (checksum & 0x80)
      {
        // checksum = ((checksum << 1) | 1) & 0xff;
        checksum = (checksum << 1) | 1;
        checksum = checksum ^ 0x18;
      }
      else
      {
        checksum = checksum << 1;
      }

      checksum = checksum ^ data[i];
    }
    return checksum;
  }

  bool checksumOk(byte *answerBuff, byte len)
  {
    return checksum(answerBuff, len - 1) == answerBuff[len - 1];
  }

  /**
   * Create a command (or request) packet to be send to the Vaillant device
   * @param packet Pointer to an array of CMD_LENGTH bytes where the resulting packet is stored
   * @param address The address of the command/request to be executed on Vaillant
   * @return CMD_LENGTH
   **/
  int buildPacket(byte *packet, byte address)
  {
    int i = 0;

    // Copy start sequence
    while (i < sizeof(startBytes))
    {
      packet[i] = startBytes[i];
      i++;
    }
    // The actual address of the command to call
    packet[i] = address;
    i++;
    // There is one byte of 0x00 before the checksum
    packet[i] = 0x00;
    i++;
    packet[i] = checksum(packet, 6);
    return i;
  }

  //  Allocate an buffer big enough to fit the answer packet
  //  byte *answerBuff = (byte *)malloc(sizeof(byte *) * answerLen >= 8);
  /**
   * Send a command package (from buildPackage) to Vaillant and fetch the answer
   * @param answerBuff Pointer to an array of at least ANSWER_LENGTH bytes wher the answer is stored
   * @param packet The command packet to be send to Vaillant
   * @return Number of bytes read into answerBuff, -1 in case the answer was > 8 bytes long and -2 in case of checksum missmatch
   */
  int sendPacket(byte *answerBuff, byte *packet)
  {
    int answerLen = 0;
    int readRetry = 3;
    // Send the command packet to Vaillant
    write_array(packet, CMD_LENGTH);

    // Wait for the first byte to arrive to parse the lenght of the answer
    while (available() < 1)
    {
      delay(50);
      readRetry--;
      if (readRetry < 0)
      {
        ESP_LOGE("Vaillantx6 sendPacket", "Timed out waiting for bytes from Vaillant");
        // TODO: add a sensor for errors
        return -3;
      }
    }
    answerLen = peek();

    // Safety net. Have not seen anything longer than 8 bytes
    // coming from Vaillant
    if (answerLen > ANSWER_LENGTH)
    {
      ESP_LOGE("Vaillantx6 sendPacket", "Received an answer of unexpected length %d, ignoring", answerLen);
      // Empty the buffer to ensure a clean start on next run
      while (available())
      {
        read();
      }
      return -1;
    }
    // Read the complete answer (including length and checksum)
    read_array(answerBuff, answerLen);
    if (!checksumOk(answerBuff, answerLen))
    {
      ESP_LOGE("Vaillantx6 sendPacket", "Packet has invalid checksum");
      // Can't be sure that the calculated length was correct in the first place,
      // so make sure the buffer is empty before trying again
      while (available())
      {
        read();
      }
      return -2;
    }
    return answerLen;
  }

  void update() override
  {
    int answerLen = 0;
    byte *cmdPacket = (byte *)malloc(sizeof(byte *) * CMD_LENGTH);
    byte *answerBuff = (byte *)malloc(sizeof(byte *) * ANSWER_LENGTH);

    for (int i = 0; i < vaillantCommandsSize; i++)
    {
      buildPacket(cmdPacket, vaillantCommands[i].Address);
      logCmd(vaillantCommands[i].Name.c_str(), cmdPacket);

      answerLen = sendPacket(answerBuff, cmdPacket);
      if (answerLen < 0)
      {
        ESP_LOGE("Vaillantx6", "sendPacket returned an error: %d", answerLen);
        continue;
      }
      else if (answerLen <= 3)
      {
        ESP_LOGW("Vaillantx6", "Anwer is too short (%d bytes)", answerLen);
        continue;
      }

      // Parse data
      for (int t = 0; t < RETURN_TYPE_COUNT; t++)
      {
        int sensorID = vaillantCommands[i].SensorID[t];
        if (sensorID < 0)
        {
          ESP_LOGI("Vaillantx6", "%s: No sensor for type id %d", vaillantCommands[i].Name.c_str(), t);
          continue;
        }

        switch (vaillantCommands[i].ReturnTypes[t])
        {
        case None:
        case SensorState:
          // FIXME: This ignores the sensor state, it probably should not.
          // Exit the loop on first None type, there won't be more
          goto exit_type_loop;
        case Temperature:
          temperatureSensors[sensorID]->publish_state(VaillantParseTemperature(answerBuff, 2));
          // Exit the loop after parsing a temperature (0x98 has two, but I don't know the meaning of the second)
          goto exit_type_loop;
        case Bool:
        {
          int b = VaillantParseBool(answerBuff, 2);
          if (b < 0)
            continue;
          binarySensors[sensorID]->publish_state(b);
          goto exit_type_loop;
        }
        case Minutes:
          minutesSensor[sensorID]->publish_state(answerBuff[2]);
          goto exit_type_loop;
          
        case Hours:
          hourSensor[sensorID]->publish_state(VaillantParseHours(answerBuff, 2));
        }
      }
    exit_type_loop:;
    }

    free(cmdPacket);
    free(answerBuff);
  }
};
