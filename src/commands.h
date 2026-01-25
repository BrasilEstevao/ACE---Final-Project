#ifndef COMMANDS_H
#define COMMANDS_H

#include <Arduino.h>

#define COMMANDS_BUF_IN_SIZE 128

typedef struct { 
  char* command;
  float value;
  char* text;
  int command_is(const char* c);
} frame_data_t;

typedef enum { 
  cs_wait_for_command,
  cs_reading_data
} commands_state_t;

class commands_t
{
  public:
    void (*process_command)(frame_data_t frame);

    char buffer[COMMANDS_BUF_IN_SIZE];
    int count;
    commands_state_t state;
    frame_data_t frame;

    commands_t();
    void init(void (*process_command_function)(frame_data_t frame));
    void process_char(char b);
};

#endif // COMMANDS_H