#include "esphome.h"
#include <string>

#define CMD_LENGTH 7
#define ANSWER_LENGTH 8
#define RETURN_TYPE_COUNT 3

typedef unsigned char byte;

void logCmd(const char *tag, byte *cmd)
{
  ESP_LOGD("Vaillantx6 command", "%s: 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x", tag, cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], cmd[6]);//debug
}


void logAnswer(const char *tag, byte *answer, int length)
{
  char logMessage[256];
  int offset = snprintf(logMessage, sizeof(logMessage), "%s: ", tag);
  for (int i = 0; i < length && offset < sizeof(logMessage); i++)
  {
    offset += snprintf(logMessage + offset, sizeof(logMessage) - offset, "0x%02X ", answer[i]);
  }
  ESP_LOGD("---answer---", "%s", logMessage);
}


// Enumerationen (enum) sind eine Möglichkeit, eine Gruppe von konstanten Ganzzahlwerten mit Namen zu versehen.
// Hier wird ein enum namens VaillantReturnTypes definiert.
// Dieser enum enthält verschiedene Rückgabewerttypen: None, Temperature, SensorState, Bool, Signal1Byte, und Signal2Byte.
// None ist explizit auf 0 gesetzt, und die nachfolgenden Werte werden automatisch auf 1, 2, 3, usw. gesetzt
enum VaillantReturnTypes
{
  None = 0,
  Temperature,//1
  SensorState,//2
  Bool,//3
  Signal1Byte,//4
  Signal2Byte//5
  //Temperature2
};

// Diese Funktion nimmt einen Wert des Typs VaillantReturnTypes als Eingabeparameter t an und gibt einen uint8_t (einen 8-Bit-Unsigned-Integer) zurück.
// Mit Hilfe einer switch-Anweisung wird überprüft, welcher Rückgabewerttyp übergeben wurde.
// Falls SensorState, Bool oder Signal1Byte: Die Funktion gibt 1 zurück, was bedeutet, dass diese Typen jeweils 1 Byte lang sind.
// Falls Temperature oder Signal2Byte: Die Funktion gibt 2 zurück, was bedeutet, dass diese Typen jeweils 2 Bytes lang sind.
// Für alle anderen Werte (default): Die Funktion gibt 0 zurück.
// Diese Funktion ermöglicht es, die Länge der Rückgabewerte basierend auf ihrem Typ zu bestimmen, was nützlich sein kann, um die richtige Menge an Speicherplatz für die Verarbeitung dieser Werte zu reservieren.
uint8_t VaillantReturnTypeLength(VaillantReturnTypes t)
{
  switch (t)
  {
  case SensorState:
  case Bool:
  case Signal1Byte://Value1Byte
    return 1;
  case Temperature:
    return 2;
  case Signal2Byte:
    return 2; 
  default:
    return 0;
  }
}

float VaillantParseTemperature(byte *answerBuff, uint8_t offset)
{
  // Combine two bytes from the answer buffer into a 16-bit integer
  int16_t i = (answerBuff[offset] << 8) | answerBuff[offset + 1];
  // Convert the integer value to a float and divide by 16 to get the temperature
  // Beispiel Horlauf ist HK1
  // antwort x06 x00 x03 x27 x00 x22
  // offset = 2
  // es werden x03 + x27 kombiniert --> HEX 0327
  // in dez = 807 /16 = 50.4375
  return i / (16.0f);
}

// Zusammengefasst: Der Code liest zwei aufeinanderfolgende Bytes aus dem answerBuff-Array und kombiniert sie zu einem einzelnen 16-Bit-Integer. 
// Das erste Byte wird nach links verschoben, um das höherwertige Byte des 16-Bit-Werts zu bilden, und das zweite Byte wird als das niedrigwertige Byte verwendet.
// Diese Technik wird häufig verwendet, um Daten aus Byte-Arrays zu lesen, insbesondere bei der Arbeit mit Protokollen und binären Datenformaten.

// Hier ist eine detaillierte Erklärung:
// Bit-Verschiebung (<< 8):
// answerBuff[offset] nimmt den Wert des Bytes am Index offset im answerBuff-Array.
// << 8 verschiebt die Bits dieses Wertes um 8 Positionen nach links. Das bedeutet, dass der ursprüngliche Wert des Bytes mit 256 multipliziert wird, was es effektiv zu einem höheren Byte eines 16-Bit-Wertes macht.

// Bitweise ODER-Operation (|):
// answerBuff[offset + 1] nimmt den Wert des Bytes am Index offset + 1 im answerBuff-Array.
// Die bitweise ODER-Operation (|) kombiniert den höherwertigen Byte-Wert (der durch die Bit-Verschiebung erstellt wurde) mit dem niedrigwertigen Byte-Wert.
float VaillantParse2byte(byte *answerBuff, uint8_t offset)// wie Temp 2 byte ohne Teiler 16
{
  // Combine two bytes from the answer buffer into a 16-bit integer. First byte convert from 8bit to 16 and move 0001 0010 --> 0001 0010 0000 0000 combine with: 0011 0100
  // 0001 0010 0000 0000 ODER 0000 0000 0011 0100 ------------------- 0001 0010 0011 0100 (Binär) oder 0x1234 (Hexadezimal)
  int16_t i = (answerBuff[offset] << 8) | answerBuff[offset + 1];
  // Return the integer value as a float
  return i;
}



int VaillantParseBool(byte *answerBuff, uint8_t offset)
{
  switch (answerBuff[offset])
  {
  case 0xF0:
  case 0x00:
    return 0;  // Return 0 for values 0xF0 and 0x00
  case 0x0F:
  case 0x01:
    return 1;  // Return 1 for values 0x0F and 0x01
  default:
    ESP_LOGE("VaillantParseBool", "Unable to parse a bool from 0x%.2x", answerBuff[offset]);
    return -1;  // Log an error and return -1 for unrecognized values
  }
}

struct VaillantCommand
{
  std::string Name; // Name des Befehls
  byte Address; // Adresse des Befehls 
  VaillantReturnTypes ReturnTypes[RETURN_TYPE_COUNT];
  // SensorID contains the ID of the sensor to use, corresponding to the ReturnType.
  // Use -1 to not assign a sensor.
  int SensorID[RETURN_TYPE_COUNT];// Array von Sensor-IDs
};

//Var_txt_NameTest:5
const VaillantCommand vaillantCommands[] = {
    {"Vorlauf Ist HK1", 0x18, {Temperature, SensorState, None}, {0, -1, -1}},//vermutlich Temperatur im Kesselkreislauf
    {"Speichertemperatur ist", 0x17, {Temperature, None, None}, {1, -1, -1}},//geht    
    {"00 test Temp 1", 0x4b, {Temperature, None, None}, {2, -1, -1}},//geht    
    {"00 test Temp 2", 0x4c, {Temperature, None, None}, {3, -1, -1}},//geht    
// 1 byte lang
    {"Brennersperrzeit 1 byte 38", 0x38, {Signal1Byte, None, None}, {0, -1, -1}},//
    {"00 test 1 byte 1", 0x4b, {Signal1Byte, None, None}, {1, -1, -1}},//
    {"00 test 1 byte 2", 0x4c, {Signal1Byte, None, None}, {2, -1, -1}},//

//2 Byte lang
    {"00 test 2 byte 1", 0x4b, {Signal2Byte, Signal2Byte, None}, {0, -1, -1}},//
    {"Brennerstarts Heizen 29", 0x29, {Signal2Byte, Signal2Byte, None}, {1, -1, -1}},//
    {"Drehzahl Gebläse soll 24", 0x24, {Signal2Byte, Signal2Byte, None}, {2, -1, -1}},//
    {"00 test 2 byte 2", 0x4c, {Signal2Byte, Signal2Byte, None}, {3, -1, -1}},//

    {"Brenner", 0x0D, {Bool, None, None}, {0, -1, -1}},//geht
};
const byte vaillantCommandsSize = sizeof(vaillantCommands) / sizeof *(vaillantCommands);

class Vaillantx6 : public PollingComponent, public UARTDevice
{
  // Sensors as provided by custom_component lambda call
  Sensor *temperatureSensors[4];  // Array for temperature sensors
  Sensor *Signal1ByteSensor[3];  // Array for minute sensors
  Sensor *Signal2ByteSensor[4];  // Array for two byte sensors
  BinarySensor *binarySensors[1];  // Array for binary sensors
  
  // All commands start with the startBytes sequence
  const byte startBytes[4] = {0x07, 0x00, 0x00, 0x00};  // Start sequence for commands

public:
  Vaillantx6(UARTComponent *parent,
             Sensor *tSensor0, Sensor *tSensor1, Sensor *tSensor2, Sensor *tSensor3,
             Sensor *oneBSensor0, Sensor *oneBSensor1,Sensor *oneBSensor2,
             Sensor *twoBSensor0, Sensor *twoBSensor1, Sensor *twoBSensor2, Sensor *twoBSensor3, 
             BinarySensor *bSensor0
             )
      : PollingComponent(10000), UARTDevice(parent)
  {
    // Temperature Sensors
    temperatureSensors[0] = tSensor0; // Flow temperature is
    temperatureSensors[1] = tSensor1; // Flow temperature set
    temperatureSensors[2] = tSensor2; // Flow temperature target
    temperatureSensors[3] = tSensor3; // Flow temperature target

    // OneByte sensors
    Signal1ByteSensor[0] = oneBSensor0; // Remaining burner lockout time
    Signal1ByteSensor[1] = oneBSensor1; // Remaining burner lockout time
    Signal1ByteSensor[2] = oneBSensor2; // Remaining burner lockout time


    // TwoByte sensors
    Signal2ByteSensor[0] = twoBSensor0; // Time until maintenance
    Signal2ByteSensor[1] = twoBSensor1; // Time until maintenance
    Signal2ByteSensor[2] = twoBSensor2; // Time until maintenance
    Signal2ByteSensor[3] = twoBSensor3; // Time until maintenance

    // Binary sensors
    binarySensors[0] = bSensor0; // Burner
  }

  /**
   * Compute the checksum used for Vaillant commands (and responses)
   *
   * @param data Array of bytes to compute the checksum for
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
   * Create a command (or request) packet to be sent to the Vaillant device
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

  // Allocate a buffer big enough to fit the answer packet
  // byte *answerBuff = (byte *)malloc(sizeof(byte *) * answerLen >= 8);
  /**
   * Send a command packet (from buildPacket) to Vaillant and fetch the answer
   * @param answerBuff Pointer to an array of at least ANSWER_LENGTH bytes where the answer is stored
   * @param packet The command packet to be sent to Vaillant
   * @return Number of bytes read into answerBuff, -1 in case the answer was > 8 bytes long and -2 in case of checksum mismatch
   */
  int sendPacket(byte *answerBuff, byte *packet)
  {
    int answerLen = 0;
    int readRetry = 3;
    // Send the command packet to Vaillant
    write_array(packet, CMD_LENGTH);

    // Wait for the first byte to arrive to parse the length of the answer
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
      buildPacket(cmdPacket, vaillantCommands[i].Address); //Ein Befehls-Paket wird erstellt.
      logCmd(vaillantCommands[i].Name.c_str(), cmdPacket);//Der Befehl wird protokolliert.ESP_LOGD("Vaillantx6", "%s: 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x", tag, cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], cmd[6]);//debug
      
      answerLen = sendPacket(answerBuff, cmdPacket);//Paket wird gesendet, answer packet empfangen
 

      if (answerLen < 0)
      {
        ESP_LOGE("Vaillantx6", "sendPacket returned an error: %d", answerLen);
        continue;
      }
      else if (answerLen <= 3)
      {
        ESP_LOGW("Vaillantx6", "Answer is too short (%d bytes)", answerLen);
        continue;
      }
      else
      {
        logAnswer("Received Data", answerBuff, answerLen);
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
        case Signal1Byte:
          Signal1ByteSensor[sensorID]->publish_state(answerBuff[2]);
          goto exit_type_loop;

        case Signal2Byte:
          Signal2ByteSensor[sensorID]->publish_state(VaillantParse2byte(answerBuff, 2));
          goto exit_type_loop;
        }
      }
    exit_type_loop:;
    }

    free(cmdPacket);
    free(answerBuff);
  }
};