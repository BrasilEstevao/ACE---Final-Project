#include <Arduino.h>
#include "commands.h"

int frame_data_t::command_is(const char* c)
{
  return !strncmp(command, c, COMMANDS_BUF_IN_SIZE);
}

commands_t::commands_t()
{
  count = 0;
  state = cs_wait_for_command;
  memset(buffer, 0, sizeof(buffer));
  process_command = NULL;
  
  frame.command = buffer;
  frame.text = buffer;
  frame.value = 0;
}

void commands_t::init(void (*process_command_function)(frame_data_t frame))
{
  process_command = process_command_function;
}

void commands_t::process_char(char b)
{
  if (state == cs_wait_for_command && isalpha(b)) {
    state = cs_reading_data;
    buffer[0] = b;
    count = 1;
  
  } else if (state == cs_reading_data && b == 0x08) {
    if (count > 0) {
      count--;
      buffer[count] = 0;  
    }    

  } else if (state == cs_reading_data && (b == 0x0A || b == 0x0D)) {
    if (count != 0) {
      buffer[count] = 0;
      frame.command = buffer;
      
      frame.text = buffer;
      while(*frame.text) {
        if (*frame.text == ' ' || *frame.text == ':') {
          *frame.text = 0;
          frame.text++;
          while(*frame.text == ' ') frame.text++;
          break;
        }
        frame.text++;
      }
      
      frame.value = atof((const char*)frame.text);

      if (process_command)
        (*process_command)(frame);

      count = 0;
      memset(buffer, 0, sizeof(buffer));
      frame.text = buffer;
    } 
    state = cs_wait_for_command;
  
  } else if (state == cs_reading_data && count < COMMANDS_BUF_IN_SIZE - 1) {
    buffer[count] = b;
    count++;
  }
}