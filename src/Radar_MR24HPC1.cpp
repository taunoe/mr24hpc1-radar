/*
Copyright 2023 Tauno Erik
*/

#include "Arduino.h"
#include "Radar_MR24HPC1.h"

Radar_MR24HPC1::Radar_MR24HPC1(Stream *s)
    : stream(s) {
  this->is_new_frame = false;
}

/*
  Receive radar frame and store it in frame array
*/ 
void Radar_MR24HPC1::read() {
  unsigned char buffer[FRAME_SIZE] = {0};
  uint8_t buff_len = 0;

  // save to buffer
  while (stream->available()) {
    // Frame start bytes
    if (stream->read() == HEAD1) {
      if (stream->read() == HEAD2) {
        // Read data
        // What if data is 0x43?? then its stops!!
        buff_len = stream->readBytesUntil(END2, buffer, FRAME_SIZE);
        // buff_len = stream->readBytes(buffer, DATA_SIZE);
        // Kontrollida kas viimane element on õige:
        // if (buffer[buff_len-1] != END1) {
        //  Serial.print(" !! False end !! ");
        //}
        if (buff_len > 0 && buff_len < FRAME_SIZE) {
          buffer[buff_len] = END2;  // add last end byte
        }
      }
    }
  }

  // Save to data_frame
  // Add headers
  frame[0] = HEAD1;
  frame[1] = HEAD2;

  for (int i = 0; i < buff_len+1; i++) {  // +1 end2 byte
    frame[i+2] = buffer[i];
  }

  // Checksum
  if (is_frame_good(frame)) {
    // Serial.print("Good: ");
    frame_len = buff_len + 3;
    is_new_frame = true;
  } else {
  // Serial.println(" Bad: ");
  }

}


/*
Print radar data on serial monitor
mode: HEX, DEC, CHAR
Adds headers
*/
void Radar_MR24HPC1::print(int mode) {
  if (is_new_frame) {
    switch (mode) {
      case DEC:
        Serial.print(' ');
        print_dec(frame, frame_len);
        break;
        break;
      default:  // HEX
        Serial.print(' ');
        print_hex(frame, frame_len);
        break;
    }

    is_new_frame = false;
    frame[frame_len] = {0};  // set all data to zero
  }
}


/*
  Serial Print data in HEX format
  buff - data char array
  len - data char lenght
*/
void Radar_MR24HPC1::print_hex(const unsigned char* buff, int len) {
  char charVal[4];

  for (int i=0; i < len; i++) {
    // sprintf(charVal, "%02X", buff[i]);
    snprintf(charVal, sizeof(charVal), "%02X", buff[i]);
    Serial.print(charVal);
    Serial.print(' ');
  }

  Serial.println();
}

/*
  Serial Print data in DEC format
  buff - data char array
  len - data char lenght
*/
void Radar_MR24HPC1::print_dec(const unsigned char* buff, int len) {
  char char_val[4];

  for (int i=0; i < len; i++) {
    // sprintf(char_val, "%d", buff[i]);
    snprintf(char_val, sizeof(char_val), "%02X", buff[i]);
    Serial.print(char_val);
    Serial.print(' ');
  }

  Serial.println();
}

/*

void Radar_MR24HPC1::analys(bool show_bodysign) {
  read();  // ?

  if (is_new_frame) {
    // Read control word
    int control_word = frame[I_CONTROL_WORD];

    // print();

    if (control_word == HUMAN_STATUS) {
      int report = frame[I_CMD_WORD];  // Read command word

      if (report == PRESENCE_REPORT) {
        // Read data byte
        int d = frame[I_DATA];  // OCCUPIED, UNOCCUPIED
        if (d == 0) {
          status_msg = NOONE;
        } else {  // == 1
          status_msg = SOMEONE;
        }
      } else if (report == MOTION_REPORT) {
        int d = frame[I_DATA];  // NONE, MOTIONLESS, ACTIVE
        switch (d) {
          case NONE:
            status_msg = NOTHING;
            break;
          case MOTIONLESS:
            status_msg = SOMEONE_STOP;
            break;
          case ACTIVE:
            status_msg = SOMEONE_MOVE;
            break;
        }
      } else if (report == MOVMENT_PARAM) {
        status_msg = HUMANPARA;
        bodysign_val = frame[I_DATA];
      } else if (report == PROX_REPORT) {
        int d = frame[I_DATA];  // NONE, NEAR, FAR
        switch (d) {
          case NONE:
            status_msg = NOTHING;
            break;
          case NEAR:
            status_msg = SOMEONE_CLOSE;
            break;
          case FAR:
            status_msg = SOMEONE_AWAY;
            break;
        }
      }
    } else if (control_word == DETAIL_STATUS) {
      int report = frame[I_CMD_WORD];  // Read command word

      if (report == SENSOR_REPORT) {
        // int d = data[4];
        status_msg  = DETAILMESSAGE;

        static_energy  = frame[I_DATA];
        static_dist = frame[I_DATA + 1];
        motion_energy = calculate_distance(frame[I_DATA + 2]);
        motion_dist  = calculate_distance(frame[I_DATA + 3]);
        motion_speed = calculate_speed(frame[I_DATA + 4]);
      } else if (report == DET_PROX_REPORT) {
        int d = frame[I_DATA];  // NONE, NEAR, FAR
        switch (d) {
          case NONE:
            status_msg = NOTHING;
            break;
          case NEAR:
            status_msg = SOMEONE_CLOSE;
            break;
          case FAR:
            status_msg = SOMEONE_AWAY;
            break;
        }
      } else if (report == DET_MOVMENT_PARA) {
        status_msg = HUMANPARA;
        bodysign_val = frame[I_DATA];
      }
    }
  }
  is_new_frame = false;
}
*/


// Send data frame
void Radar_MR24HPC1::write_cmd(const unsigned char* buff,
                              int len, bool cyclic) {
  if (cyclic || count < checkframe_len) {
    if (cyclic || count < 1) {
      stream->write(buff, len);
      stream->flush();
    }

    // read radar responce
    do {
      read();
      delay(20);
    } while (!(this->is_new_frame));

    if (cyclic || count < 1) {
      Serial.print("  Sent  ---> ");
      print_hex(buff, len);
    }

    if (count%2 == 1) {
      Serial.print("Receive <--- ");
      print();
    }

    is_new_frame = false;
  }
  count++;
}

/*
Return Radar ID
*/
int Radar_MR24HPC1::get_id() {
  const unsigned char id_cmd[CMD_LEN] = {
    HEAD1, HEAD2, 0x02, 0xA2, 0x00, 0x01, 0x0F, 0x60, END1, END2};

  stream->write(id_cmd, CMD_LEN);
  stream->flush();

  // read radar responce
  do {
    read();
    // delay(10);
  } while (!(is_new_frame));

  print(DEC);
  is_new_frame = false;
  // TODO: return id
  return 0;
}

int Radar_MR24HPC1::firm_ver_id() {
  const unsigned char firmware_ver_cmd[CMD_LEN] = {
    HEAD1, HEAD2, 0x02, 0xA4, 0x00, 0x01, 0x0F, 0x62, END1, END2};

  stream->write(firmware_ver_cmd, CMD_LEN);
  stream->flush();

  // read radar responce
  do {
    read();
    // delay(10);
  } while (!(is_new_frame));

  print();
  is_new_frame = false;
  // TODO: return id
  return 0;
}

/*
Reset radar
returns 1
*/
int Radar_MR24HPC1::reset() {
  const unsigned char reset_cmd[CMD_LEN] = {
    HEAD1, HEAD2, 0x01, 0x02, 0x00, 0x01, 0x0F, 0xBF, END1, END2};

  stream->write(reset_cmd, CMD_LEN);
  stream->flush();
  // Serial.println("Radar reset!");
  return 1;
}


float Radar_MR24HPC1::calculate_distance(int val) {
  return val*UNIT;
}

float Radar_MR24HPC1::calculate_speed(int val) {
  if (val == 0x0A) {
    return 0;
  } else if (val > 0x0A) {
    // Negative speed
    return -((val-10)*UNIT);
  } else if (val < 0x0A) {
    // Positive speed
    return (val)*UNIT;
  }
  return 0;
}


/*
 Converts the hexadecimal string to an integer
*/
int Radar_MR24HPC1::hex_to_int(const unsigned char *hexChar) {
  // Use strtol to convert the hexadecimal string to an integer
  // return strtol(hexChar, NULL, 16);
  int intValue = 0;

    if (*hexChar >= '0' && *hexChar <= '9') {
        intValue = *hexChar - '0';
    } else if (*hexChar >= 'A' && *hexChar <= 'F') {
        intValue = *hexChar - 'A' + 10;
    } else if (*hexChar >= 'a' && *hexChar <= 'f') {
        intValue = *hexChar - 'a' + 10;
    } else {
        // Handle invalid input if necessary
        printf("Invalid hexadecimal character: %c\n", *hexChar);
    }

    return intValue;
}

/*
 Converts the hexadecimal string to an char
*/
char Radar_MR24HPC1::hex_to_char(const unsigned char *hex) {
  int int_val = hex_to_int(hex);
  // Cast the integer value to a char
  char char_val = static_cast <char> (int_val);  // (char)int_val;

  return char_val;
}

/*
 Calcutaltes checksum
*/
uint8_t Radar_MR24HPC1::calculate_sum(const unsigned char f[], int size) {
  uint8_t sum = 0;

  for (int i = 0; i < size; i++) {
    sum += f[i];
  }

  return sum;
}

/*
 Compare data sum and calculated sum
*/
bool Radar_MR24HPC1::is_frame_good(const unsigned char f[]) {
  unsigned char data_lenght_byte = f[I_LENGHT_L];

  int count = I_DATA + data_lenght_byte;  // how many bytes total

  uint8_t my_sum = calculate_sum(f, count);

  uint8_t frame_sum = f[I_DATA + data_lenght_byte];

  if (frame_sum == my_sum) {
    return true;
  }

  return false;
}


/*
Set Radar Mode: 0 SIMPLE, 1 ADVANCED
*/
void Radar_MR24HPC1::set_mode(int mode) {
  const unsigned char on_cmd[CMD_LEN] = {
    HEAD1, HEAD2, 0x08, 0x00, 0x00, 0x01, 0x01, 0xB6, END1, END2 };

  const unsigned char off_cmd[CMD_LEN] = {
    HEAD1, HEAD2, 0x08, 0x00, 0x00, 0x01, 0x00, 0xB5, END1, END2 };

  if (mode == SIMPLE) {
    stream->write(off_cmd, CMD_LEN);
    stream->flush();
    is_advanced_mode = false;
  } else if (mode == ADVANCED) {
    stream->write(on_cmd, CMD_LEN);
    stream->flush();
    is_advanced_mode = true;
  }
}


/*
Returns Underlying Open function status:
1-ON
0-OFF
*/
int Radar_MR24HPC1::underlying_status() {
  const unsigned char status_cmd[CMD_LEN] = {
    HEAD1, HEAD2, 0x08, 0x80, 0x00, 0x01, 0x0F, 0x44, END1, END2};

  // Send command
  stream->write(status_cmd, CMD_LEN);
  stream->flush();
  // Read response
  do {
    read();
    // delay(10);
  } while (!(is_new_frame));

  Serial.print("Response: ");
  print();
  is_new_frame = false;
  return 0;
}

/*
Translate the responses into human-readable form
*/
void Radar_MR24HPC1::translate() {
  if (is_new_frame) {
    int control_word = frame[I_CONTROL_WORD];

    switch (control_word) {
      case 0x01:
        translate_01();
        break;
       case 0x02:
        translate_02();
        break;
      case 0x03:
        Serial.println("UART upgrade");
        break;
      case 0x05:
        translate_05();
        break;
      case 0x08:
        translate_08();
        break;
      case 0x80:
        translate_80();
        break;
      default:
        print();
        break;
    }
  }
}

/*
translate() helper func
Controll word 0x01
Hearbeat, reset
*/
void Radar_MR24HPC1::translate_01() {
  int cmd_word = frame[I_CMD_WORD];
  int data = frame[I_DATA];

  if (cmd_word == 0x01 && data == 0x0F) {
    Serial.println("Heartbeat");
  } else if (cmd_word == 0x02 && data == 0x0F) {
    Serial.println("Reset");
  }
}

/*
translate() helper func
Controll word 0x02
Product info
*/
void Radar_MR24HPC1::translate_02() {
  // int cmd_word = frame[I_CMD_WORD];
  int len = frame[I_LENGHT_L];
  // int data = frame[I_DATA];

  // Product Model
  if (cmd_word == 0xA1) {
    Serial.println("Product Model");
  } else if (cmd_word == 0xA2) {
    Serial.println("Product ID");
  } else if (cmd_word == 0xA3) {
    Serial.println("Hardware Model");
  } else if (cmd_word == 0xA3) {
    Serial.println("Firmware version");
  }
}

/*
translate() helper func
Controll word 0x05
Work status
*/
void Radar_MR24HPC1::translate_05() {
  int cmd_word = frame[I_CMD_WORD];
  // int len = frame[I_LENGHT_L];
  // int data = frame[I_DATA];

  if (cmd_word == 0x01) {
      Serial.println("Initialization completed");
  } else if (cmd_word == 0x07) {
      Serial.println("Scene settings");
  } else if (cmd_word == 0x08) {
      Serial.println("Sensitivity settings");
  } else if (cmd_word == 0x81) {
      Serial.println("Initializatio status inquiry");
  } else if (cmd_word == 0x87) {
      Serial.println("Scene settings inquiry");
  } else if (cmd_word == 0x88) {
      Serial.println("Sensitivity settings inquiry");
  }
  // Underlying
  else if (cmd_word == 0x09) {
      Serial.println("Custom mode setting");
  } else if (cmd_word == 0x0A) {
      Serial.println("Custom mode saved");
  } else if (cmd_word == 0x89) {
      Serial.println("Custom mode status");
  }
}


/*
translate() helper func
Controll word 0x0
Underlying open func
*/
void Radar_MR24HPC1::translate_08() {
  int cmd_word = frame[I_CMD_WORD];
  // int len = frame[I_LENGHT_L];
  // int data = frame[I_DATA];

  if (cmd_word == 0x00) {
      Serial.println("Output switch");
  } else if (cmd_word == 0x01) {
      Serial.println("Sensor report");
  } else if (cmd_word == 0x08) {
      Serial.println("Sensor report");
  } else if (cmd_word == 0x09) {
      Serial.println("Trigger threshold settings");
  } else if (cmd_word == 0x80) {
      Serial.println("Output switch inquiry");
  } else if (cmd_word == 0x81) {
      Serial.println("Static energy");
  } else if (cmd_word == 0x82) {
      Serial.println("Motion energy");
  } else if (cmd_word == 0x83) {
      Serial.println("Static distance inquiry");
  } else if (cmd_word == 0x84) {
      Serial.println("Motion distance inquiry");
  } else if (cmd_word == 0x85) {
      Serial.println("Motion speed");
  } else if (cmd_word == 0x88) {
      Serial.println("Static Threshold inquiry");
  } else if (cmd_word == 0x89) {
      Serial.println("Motion Threshold inquiry");
  } else if (cmd_word == 0x0A) {
      Serial.println("Static Detection range setting");
  } else if (cmd_word == 0x0B) {
      Serial.println("Motion Detection range setting");
  } else if (cmd_word == 0x0C) {
      Serial.println("Trigger time setting");
  } else if (cmd_word == 0x0D) {
      Serial.println("Still time setting");
  } else if (cmd_word == 0x0E) {
      Serial.println("no persson setting");
  } else if (cmd_word == 0x8A) {
      Serial.println("static boundary inquiry");
  } else if (cmd_word == 0x8B) {
      Serial.println("motion boundary inquiry");
  } else if (cmd_word == 0x8C) {
      Serial.println("motion trigger time inquiry");
  } else if (cmd_word == 0x8D) {
      Serial.println("motion to still time inquiry");
  } else if (cmd_word == 0x8E) {
      Serial.println("time no person state inquiry");
  }
}

/*
translate() helper func
Controll word 0x80
Human presence
*/
void Radar_MR24HPC1::translate_80() {
  int cmd_word = frame[I_CMD_WORD];
  // int len = frame[I_LENGHT_L];
  // int data = frame[I_DATA];

  if (cmd_word == 0x01) {
      Serial.println("Presence reporting");
  } else if (cmd_word == 0x02) {
      Serial.println("Motion reporting");
  } else if (cmd_word == 0x03) {
      Serial.println("Body parameter");
  } else if (cmd_word == 0x0A) {
      Serial.println("Time settings");
  } else if (cmd_word == 0x0B) {
      Serial.println("Proximity");
  } else if (cmd_word == 0x81) {
      Serial.println("Presence info");
  } else if (cmd_word == 0x82) {
      Serial.println("Motion info");
  } else if (cmd_word == 0x83) {
      Serial.println("Body movment");
  } else if (cmd_word == 0x8A) {
      Serial.println("Time inquiry");
  } else if (cmd_word == 0x8B) {
      Serial.println("Proximity movment");
  }
}