/*
 *  _____ ______    ___   ________   ___                   _____ ______    _____ ______       ___    ___ 
 * |\   _ \  _   \ |\  \ |\   ___ \ |\  \                 |\   _ \  _   \ |\   _ \  _   \    |\  \  /  /|
 * \ \  \\\__\ \  \\ \  \\ \  \_|\ \\ \  \   ____________ \ \  \\\__\ \  \\ \  \\\__\ \  \   \ \  \/  / /
 *  \ \  \\|__| \  \\ \  \\ \  \ \\ \\ \  \ |\____________\\ \  \\|__| \  \\ \  \\|__| \  \   \ \    / / 
 *   \ \  \    \ \  \\ \  \\ \  \_\\ \\ \  \\|____________| \ \  \    \ \  \\ \  \    \ \  \   /     \/  
 *    \ \__\    \ \__\\ \__\\ \_______\\ \__\                \ \__\    \ \__\\ \__\    \ \__\ /  /\   \  
 *     \|__|     \|__| \|__| \|_______| \|__|                 \|__|     \|__| \|__|     \|__|/__/ /\ __\ 
 *                                                                                           |__|/ \|__| 
 *                                                                                           
 * ============================================================
 *                     ┌----|#####|----┐
 *                    -│TX0         RAW│-
 *       MIDI Input >--│RX0         GND│-
 *                    -│GND         RST│-
 *                    -│GND        TVCC│-
 *     IO-      SDA <--│2            A3│-
 *  Expanders   SCL <--│3            A2│-
 *                    -│4            A1│-
 *                    -│5            A0│-
 *                    -│6            15│-
 *                    -│7            14│-
 * MIDI Passthrough <--│8            16│-
 *                    -│9  PRO MICRO 10│-
 *                     └---------------┘
 * ============================================================
 * Dependencies:
 * 
 * MIDI.h:                https://www.arduino.cc/reference/en/libraries/midi-library/
 * MIDIUSB.h              https://www.arduino.cc/en/Reference/MIDIUSB
 * Adafruit_MCP23017.h    https://www.arduino.cc/reference/en/libraries/adafruit-mcp23017-arduino-library/
 * SoftwareSerial.h       https://www.arduino.cc/en/Reference/SoftwareSerial
 */

#include <MIDI.h>
#include "MIDIUSB.h"
#include "Adafruit_MCP23017.h"
#include <SoftwareSerial.h>

#define LOWEST_NOTE 48      // This MIDI note will be mapped to channel 0
#define CHANNELS 32         // Number of MMX channels that are available
#define PULSE_LENGTH 200    // How long the gates stay open (in ms)

//#define DEBUG

SoftwareSerial softSerial(7, 8);
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, hw_midi_in);      // Hardware MIDI input
MIDI_CREATE_INSTANCE(SoftwareSerial, softSerial, hw_midi_out);  // MIDI Passthrough

Adafruit_MCP23017 mcp0;
Adafruit_MCP23017 mcp1;

unsigned long channel_activation_times[CHANNELS];
bool channel_states[CHANNELS];

void writeChannel(uint8_t channel, bool state)
{
  if(channel < 16)
  {
    mcp0.digitalWrite(channel, state);
  }
  else
  {
    mcp1.digitalWrite(channel - 16, state);
  }
}

void parsePacket(uint8_t cmd, uint8_t note)
{
  if(note >= LOWEST_NOTE && note < LOWEST_NOTE + CHANNELS)
  {
    if(cmd == 9)      // Note On
    {
      uint8_t channel = note - LOWEST_NOTE;
      if(!channel_states[channel])
      {
        channel_states[channel] = true;
        channel_activation_times[channel] = millis();
        writeChannel(channel, true);
      }
    }
  }
}

void setup()
{
  Serial.begin(112500);

  hw_midi_in.begin(MIDI_CHANNEL_OMNI);
  hw_midi_out.begin(MIDI_CHANNEL_OMNI);
  
  mcp0.begin(0, &Wire);   // IO-Expanders
  mcp1.begin(1, &Wire);

  for(int i = 0; i < 16; i++)
  {
    mcp0.pinMode(i, OUTPUT);
    mcp1.pinMode(i, OUTPUT);
  }
}

void loop()
{
  midiEventPacket_t rx;     // Read USB MIDI
  do{
    rx = MidiUSB.read();
    if (rx.header != 0)
    {
      uint8_t channel = (rx.byte1 & 0x0F)+1;
      uint8_t cmd = (rx.byte1 & 0xF0)>>4;
      uint8_t note = rx.byte2;
      uint8_t velocity = rx.byte3;
      
      hw_midi_out.send(rx.byte1, note, velocity, channel);    // MIDI Passthrough
      parsePacket(cmd, note);

      #ifdef DEBUG
        Serial.print("cmd: ");
        Serial.println(cmd);
        Serial.print("ch: ");
        Serial.println(channel);
        Serial.print("note: ");
        Serial.println(note);
        Serial.print("vel: ");
        Serial.println(velocity);
      #endif
    }
  } while(rx.header != 0);

  if (hw_midi_in.read())       // Read hardware MIDI
  {
    uint8_t channel = hw_midi_in.getChannel();
    uint8_t cmd = (hw_midi_in.getType() & 0xF0)>>4;
    uint8_t note = hw_midi_in.getData1();
    uint8_t velocity = hw_midi_in.getData2();
    
    hw_midi_out.send(hw_midi_in.getType(), note, velocity, channel);    // MIDI Passthrough
    parsePacket(cmd, note);

    #ifdef DEBUG
      Serial.print("cmd: ");
      Serial.println(cmd);
      Serial.print("ch: ");
      Serial.println(channel);
      Serial.print("note: ");
      Serial.println(note);
      Serial.print("vel: ");
      Serial.println(velocity);
    #endif
  }

  for(int channel = 0; channel < CHANNELS; channel++)     // Check if channels reached the pulse length
  {
    if(channel_states[channel] && millis() - channel_activation_times[channel] > PULSE_LENGTH)
    {
      channel_states[channel] = false;
      writeChannel(channel, false);
    }
  }
}
