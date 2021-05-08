#ifndef _ConfigHandler_H_
#define _ConfigHandler_H_

#include "Arduino.h"

class ConfigHandler
{
  public:
  uint16_t pulse_length;            // How long the gates stay open (in ms)
  uint16_t dead_time;               // How long the gates stay closed after being opened (in ms)
  void begin(uint32_t baudrate);
  void handleSerial();
  
  private:
  int pulse_length_addr;
  int dead_time_addr;
  void printHelp();
  void printUnknownCmd(String cmd);
  void printInvalidValue(String value);
  void parseSerialCmd(String cmd);
  void executeCommand(String cmd_separated[]);
};

#endif
