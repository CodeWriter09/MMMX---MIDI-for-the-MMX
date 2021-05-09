#include "ConfigHandler.h"
#include "Arduino.h"
#include <EEPROMex.h>

void ConfigHandler::begin(uint32_t baudrate)
{
  Serial.begin(baudrate);
  while(!Serial){}
  pulse_length_addr = EEPROM.getAddress(sizeof(pulse_length));
  dead_time_addr = EEPROM.getAddress(sizeof(dead_time));
  pulse_length = EEPROM.readInt(pulse_length_addr);
  dead_time = EEPROM.readInt(dead_time_addr);
}

void ConfigHandler::handleSerial()
{
  while(Serial.available())
  {
    String data = Serial.readString();
    parseSerialCmd(data);
  }
}


void ConfigHandler::printHelp()
{
  Serial.println("Available Commands:");
  Serial.println(" pulselength <value>      sets the pulse length in ms");
  Serial.println(" deadtime <value>      sets the dead time in ms");
}

void ConfigHandler::printUnknownCmd(String cmd)
{
  Serial.print("Unknown command: ");
  Serial.println(cmd);
}

void ConfigHandler::printInvalidValue(String value)
{
  Serial.print("Invalid value: ");
  Serial.println(value);
}

void ConfigHandler::parseSerialCmd(String cmd)
{
  // Separate command at spaces
  int num_spaces = 0;
  int space_indices[100];
  for(int i = 0; i < cmd.length(); i++)
  {
    if(cmd[i] == ' ')
    {
      space_indices[num_spaces] = i;
      num_spaces++;
    }
  }
  space_indices[num_spaces] = cmd.length();
  num_spaces++;

  String cmd_separated[num_spaces];
  int prev_index = 0;
  for(int i = 0; i < num_spaces; i++)
  {
    cmd_separated[i] = cmd.substring(prev_index, space_indices[i]);
    prev_index = space_indices[i] + 1;
  }

  executeCommand(cmd_separated);
}

void ConfigHandler::executeCommand(String cmd_separated[])
{
  // Execute command
  if(cmd_separated[0] == "help")
  {
    printHelp();
  }
  else if(cmd_separated[0] == "pulselength")
  {
    int16_t value = (int16_t)cmd_separated[1].toInt();
    if(value > 0)
    {
      pulse_length = value;
      EEPROM.writeInt(pulse_length_addr, value);
      Serial.print("Set pulse length to ");
      Serial.print(value);
      Serial.println("ms");
    }
    else
    {
      printInvalidValue(cmd_separated[1]);
    }
  }
  else if(cmd_separated[0] == "deadtime")
  {
    int16_t value = (int16_t)cmd_separated[1].toInt();
    if(value > 0)
    {
      dead_time = value;
      EEPROM.writeInt(dead_time_addr, value);
      Serial.print("Set dead time to ");
      Serial.print(value);
      Serial.println("ms");
    }
    else
    {
      printInvalidValue(cmd_separated[1]);
    }
  }
  else
  {
    printUnknownCmd(cmd_separated[0]);
  }
}
