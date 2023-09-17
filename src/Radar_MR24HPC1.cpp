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
Send query to radar
frame - array of bytes
len - num of bytes
*/
void Radar_MR24HPC1::send_query(const unsigned char *frame, int len) {
  stream->write(frame, len);
  stream->flush();
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


/*
Send Reset frame
*/
void Radar_MR24HPC1::send_reset() {
  const int len = 10;
  uint8_t frame[len] = {
    HEAD1, HEAD2, 0x01, 0x02, 0x00, 0x01, 0x0F, 0xBF, END1, END2};
  frame[I_DATA+1] = get_frame_sum(frame, len);

  send_query(frame, len);
}

/*
Send heartbeat frame
*/
void Radar_MR24HPC1::send_heartbeat() {
  const int len = 10;
  uint8_t frame[len] = {
    HEAD1, HEAD2, 0x01, 0x01, 0x00, 0x01, 0x0F, 0x5F, END1, END2};
  frame[I_DATA+1] = get_frame_sum(frame, len);
  send_query(frame, len);
}

/*
Send product_model frame
*/
void Radar_MR24HPC1::ask_product_model() {
  const int len = 10;
  uint8_t frame[len] = {
    HEAD1, HEAD2, 0x02, 0xA1, 0x00, 0x01, 0x0F, 0xBE, END1, END2};
  frame[I_DATA+1] = get_frame_sum(frame, len);
  send_query(frame, len);
}

/*
Send product id frame
*/
void Radar_MR24HPC1::ask_product_id() {
  const int len = 10;
  uint8_t frame[len] = {
    HEAD1, HEAD2, 0x02, 0xA2, 0x00, 0x01, 0x0F, 0x00, END1, END2};
  frame[I_DATA+1] = get_frame_sum(frame, len);
  send_query(frame, len);
}

/*
Send harware model frame
*/
void Radar_MR24HPC1::ask_hardware_model() {
  const int len = 10;
  uint8_t frame[len] = {
    HEAD1, HEAD2, 0x02, 0xA3, 0x00, 0x01, 0x0F, 0x00, END1, END2};
  frame[I_DATA+1] = get_frame_sum(frame, len);
  send_query(frame, len);
}

/*
Send firmware_version frame
*/
void Radar_MR24HPC1::ask_firmware_version() {
  const int len = 10;
  uint8_t frame[len] = {
    HEAD1, HEAD2, 0x02, 0xA4, 0x00, 0x01, 0x0F, 0x00, END1, END2};
  frame[I_DATA+1] = get_frame_sum(frame, len);
  send_query(frame, len);
}

/*
Max Range to recognize human movements
Simple:
0x01 4.0-4.5 m detection radius
0x02 3.5-4.0 m
0x03 2.5-3.0 m
0x04 3.0-3.5 m
Advandced:
Motion trigger boundary settings
0x01 0.5m
0x02 1.0m
0x03 1.5m
0x04 2.0m
0x05 2.5m
0x06 3.0m
0x07 3.5m
0x08 4.0m
0x09 4.5m
0x0A 5.0m
*/
void Radar_MR24HPC1::set_movement_range(uint8_t range) {
  const int len = 10;

  if (advanced_mode) {
    if (range > 0x0A) {
      range = 0x0A;  // default
    }
    uint8_t frame[len] = {
      HEAD1, HEAD2, 0x08, 0x0B, 0x00, 0x01, range, 0x00, END1, END2};
  } else {   // Simple mode
    if (range < 1 || range > 4) {
      range = 0x01;
    }

    uint8_t frame[len] = {
      HEAD1, HEAD2, 0x05, 0x07, 0x00, 0x01, range, 0x00, END1, END2};
  }

  frame[I_DATA+1] = get_frame_sum(frame, len);
  send_query(frame, len);
}

/*
Max Range to recognize static human body
Simple mode:
0x01 2.5 m detection radius
0x02 3.0 m
0x03 4.0 m
Asvandced mode:
Existence perception boundary settings
0x01 0.5m
0x02 1.0m
0x03 1.5m
0x04 2.0m
0x05 2.5m
0x06 3.0m
0x07 3.5m
0x08 4.0m
0x09 4.5m
0x0A 5.0m
*/
void Radar_MR24HPC1::set_static_range(uint8_t range) {
  const int len = 10;

  if (advanced_mode) {
    if (range > 0x0A) {
      range = 0x0A;  // default
    }
    uint8_t frame[len] = {
      HEAD1, HEAD2, 0x08, 0x0A, 0x00, 0x01, range, 0x00, END1, END2};
  } else {  // Simple mode
    if (range < 1 || range > 3) {
      range = 0x03;  // default
    }
    uint8_t frame[len] = {
      HEAD1, HEAD2, 0x05, 0x08, 0x00, 0x01, range, 0x00, END1, END2};
  }

  frame[I_DATA+1] = get_frame_sum(frame, len);
  send_query(frame, len);
}


/*
Initialization status inquiry
*/
void Radar_MR24HPC1::ask_initialization_status() {
  const int len = 10;
  uint8_t frame[len] = {
    HEAD1, HEAD2, 0x05, 0x81, 0x00, 0x01, 0x0F, 0x00, END1, END2};
  frame[I_DATA+1] = get_frame_sum(frame, len);
  send_query(frame, len);
}

/*
scene settings inquiry
*/
void Radar_MR24HPC1::ask_movement_range() {
  const int len = 10;
  uint8_t frame[len] = {
    HEAD1, HEAD2, 0x05, 0x87, 0x00, 0x01, 0x0F, 0x00, END1, END2};
  frame[I_DATA+1] = get_frame_sum(frame, len);
  send_query(frame, len);
}

/*
Sensitivity settings inquiry
*/
void Radar_MR24HPC1::ask_sensitivity_level() {
  const int len = 10;
  uint8_t frame[len] = {
    HEAD1, HEAD2, 0x05, 0x88, 0x00, 0x01, 0x0F, 0x00, END1, END2};
  frame[I_DATA+1] = get_frame_sum(frame, len);
  send_query(frame, len);
}

/*
Time for entering no person state setting
0x00 none
0x01 10s
0x02 30s
0x03 1 min
0x04 2 min
0x05 5 min
0x06 10 min
0x07 30 min
0x08 60 min
*/
void Radar_MR24HPC1::set_absence_trigger_time(uint8_t time) {
  if (time > 0x08) {
    time = 0x02;
  }
  const int len = 10;
  uint8_t frame[len] = {
    HEAD1, HEAD2, 0x80, 0x0A, 0x00, 0x01, time, 0x00, END1, END2};
  frame[I_DATA+1] = get_frame_sum(frame, len);
  send_query(frame, len);
}

/*
Presence information inquiry
*/
void Radar_MR24HPC1::ask_presence() {
  const int len = 10;
  uint8_t frame[len] = {
    HEAD1, HEAD2, 0x80, 0x81, 0x00, 0x01, 0x0F, 0x00, END1, END2};
  frame[I_DATA+1] = get_frame_sum(frame, len);
  send_query(frame, len);
}


/*
Motion information inquiry
*/
void Radar_MR24HPC1::ask_motion_info() {
  const int len = 10;
  uint8_t frame[len] = {
    HEAD1, HEAD2, 0x80, 0x82, 0x00, 0x01, 0x0F, 0x00, END1, END2};
  frame[I_DATA+1] = get_frame_sum(frame, len);
  send_query(frame, len);
}

/*
Body movment parameter inquiry
*/
void Radar_MR24HPC1::ask_body_parameter() {
  const int len = 10;
  uint8_t frame[len] = {
    HEAD1, HEAD2, 0x80, 0x83, 0x00, 0x01, 0x0F, 0x00, END1, END2};
  frame[I_DATA+1] = get_frame_sum(frame, len);
  send_query(frame, len);
}

/*
Time for entering no person state inquiry
*/
void Radar_MR24HPC1::ask_absence_trigger_time() {
  const int len = 10;
  uint8_t frame[len] = {
    HEAD1, HEAD2, 0x80, 0x8A, 0x00, 0x01, 0x0F, 0x00, END1, END2};
  frame[I_DATA+1] = get_frame_sum(frame, len);
  send_query(frame, len);
}

/*
Proximity inquiry
*/
void Radar_MR24HPC1::ask_proximity() {
  const int len = 10;
  uint8_t frame[len] = {
    HEAD1, HEAD2, 0x80, 0x8B, 0x00, 0x01, 0x0F, 0x00, END1, END2};
  frame[I_DATA+1] = get_frame_sum(frame, len);
  send_query(frame, len);
}

/*
Underlying open function information
output switch inquiry
*/
void Radar_MR24HPC1::ask_mode() {
  const int len = 10;
  uint8_t frame[len] = {
    HEAD1, HEAD2, 0x08, 0x80, 0x00, 0x01, 0x0F, 0x00, END1, END2};
  frame[I_DATA+1] = get_frame_sum(frame, len);
  send_query(frame, len);
}

/*
Existence energy value inquiry
*/
void Radar_MR24HPC1::ask_static_energy() {
  const int len = 10;
  uint8_t frame[len] = {
    HEAD1, HEAD2, 0x08, 0x81, 0x00, 0x01, 0x0F, 0x00, END1, END2};
  frame[I_DATA+1] = get_frame_sum(frame, len);
  send_query(frame, len);
}

/*
Motion energy value inquiry
*/
void Radar_MR24HPC1::ask_motion_energy() {
  const int len = 10;
  uint8_t frame[len] = {
    HEAD1, HEAD2, 0x08, 0x82, 0x00, 0x01, 0x0F, 0x00, END1, END2};
  frame[I_DATA+1] = get_frame_sum(frame, len);
  send_query(frame, len);
}

/*
Static distance inquiry
*/
void Radar_MR24HPC1::ask_static_distance() {
  const int len = 10;
  uint8_t frame[len] = {
    HEAD1, HEAD2, 0x08, 0x83, 0x00, 0x01, 0x0F, 0x00, END1, END2};
  frame[I_DATA+1] = get_frame_sum(frame, len);
  send_query(frame, len);
}

/*
Motion distance inquiry
*/
void Radar_MR24HPC1::ask_motion_distance() {
  const int len = 10;
  uint8_t frame[len] = {
    HEAD1, HEAD2, 0x08, 0x84, 0x00, 0x01, 0x0F, 0x00, END1, END2};
  frame[I_DATA+1] = get_frame_sum(frame, len);
  send_query(frame, len);
}

/*
Motion speed inquiry
*/
void Radar_MR24HPC1::ask_motion_speed() {
  const int len = 10;
  uint8_t frame[len] = {
    HEAD1, HEAD2, 0x05, 0x85, 0x00, 0x01, 0x0F, 0x00, END1, END2};
  frame[I_DATA+1] = get_frame_sum(frame, len);
  send_query(frame, len);
}

/*
Advandsed Custom mode inquiry
0x00 Custom mode is not enabled
0x01 Custom mode 1
0x02 Custom mode 2
0x03 Custom mode 3
0x04 Custom mode 4
*/
void Radar_MR24HPC1::ask_custom_mode() {
  const int len = 10;
  uint8_t frame[len] = {
    HEAD1, HEAD2, 0x05, 0x89, 0x00, 0x01, 0x0F, 0x00, END1, END2};
  frame[I_DATA+1] = get_frame_sum(frame, len);
  send_query(frame, len);
}

/*
Advandsed Custom mode setting
0x01 Custom mode 1
0x02 Custom mode 2
0x03 Custom mode 3
0x04 Custom mode 4
*/
void Radar_MR24HPC1::start_custom_mode_settings(uint8_t mode) {
  if (mode > 0x04) {
    mode = 0x04;
  }
  const int len = 10;
  uint8_t frame[len] = {
    HEAD1, HEAD2, 0x05, 0x09, 0x00, 0x01, mode, 0x00, END1, END2};
  frame[I_DATA+1] = get_frame_sum(frame, len);
  send_query(frame, len);
}

/*
End Custom mode settings
*/
void Radar_MR24HPC1::end_custom_mode_settings() {
  const int len = 10;
  uint8_t frame[len] = {
    HEAD1, HEAD2, 0x05, 0x0A, 0x00, 0x01, 0x0F, 0x00, END1, END2};
  frame[I_DATA+1] = get_frame_sum(frame, len);
  send_query(frame, len);
}

/*
Existence judgement threshold settings
Range 0-250
*/
void Radar_MR24HPC1::set_static_threshold(uint8_t range) {
  if (range > 250) {
    range = 250;
  }
  const int len = 10;
  uint8_t frame[len] = {
    HEAD1, HEAD2, 0x08, 0x08, 0x00, 0x01, range, 0x00, END1, END2};
  frame[I_DATA+1] = get_frame_sum(frame, len);
  send_query(frame, len);
}


/*
Motion trigger threshold settings
Range 0-250
*/
void Radar_MR24HPC1::set_motion_threshold(uint8_t range) {
  if (range > 250) {
    range = 250;
  }
  const int len = 10;
  uint8_t frame[len] = {
    HEAD1, HEAD2, 0x08, 0x09, 0x00, 0x01, range, 0x00, END1, END2};
  frame[I_DATA+1] = get_frame_sum(frame, len);
  send_query(frame, len);
}

/*
Motion trigger boundry settings
0x01 0.5m
0x02 1.0m
0x03 1.5m
0x04 2.0m
0x05 2.5m
0x06 3.0m
0x07 3.5m
0x08 4.0m
0x09 4.5m
0x0A 5.0m
Returns 1 when advandsed mode
*/
int Radar_MR24HPC1::set_motion_range(uint8_t range) {
  if (advanced_mode) {
    if (range > 0x0A) {
      range = 0x0A;
    }
    const int len = 10;
    uint8_t frame[len] = {
      HEAD1, HEAD2, 0x08, 0x09, 0x00, 0x01, range, 0x00, END1, END2};
    frame[I_DATA+1] = get_frame_sum(frame, len);
    send_query(frame, len);
  } else {
    return 0;
  }
  return 1;
}

/*
Existence judgment threshold inquiry
Returns 1 when advandsed mode
*/
int Radar_MR24HPC1::ask_static_threshold() {
  if (advanced_mode) {
    const int len = 10;
    uint8_t frame[len] = {
    HEAD1, HEAD2, 0x08, 0x88, 0x00, 0x01, 0x0F, 0x00, END1, END2};
      frame[I_DATA+1] = get_frame_sum(frame, len);
    send_query(frame, len);
  } else {
    return 0;
  }
  return 1;
}

/*
Existence judgment threshold inquiry
Returns 1 when advandsed mode
*/
int Radar_MR24HPC1::ask_motion_threshold() {
  if (advanced_mode) {
    const int len = 10;
    uint8_t frame[len] = {
    HEAD1, HEAD2, 0x08, 0x89, 0x00, 0x01, 0x0F, 0x00, END1, END2};
      frame[I_DATA+1] = get_frame_sum(frame, len);
    send_query(frame, len);
  } else {
    return 0;
  }
  return 1;
}

/*
Existence perception boundry inquiry
Returns 1 when advandsed mode
*/
int Radar_MR24HPC1::ask_static_range() {
  if (advanced_mode) {
    const int len = 10;
    uint8_t frame[len] = {
    HEAD1, HEAD2, 0x08, 0x8A, 0x00, 0x01, 0x0F, 0x00, END1, END2};
      frame[I_DATA+1] = get_frame_sum(frame, len);
    send_query(frame, len);
  } else {
    return 0;
  }
  return 1;
}

/*
Motion trigger boundry inquiry
Returns 1 when advandsed mode
*/
int Radar_MR24HPC1::ask_motion_range() {
  if (advanced_mode) {
    const int len = 10;
    uint8_t frame[len] = {
    HEAD1, HEAD2, 0x08, 0x8B, 0x00, 0x01, 0x0F, 0x00, END1, END2};
      frame[I_DATA+1] = get_frame_sum(frame, len);
    send_query(frame, len);
  } else {
    return 0;
  }
  return 1;
}

/*
Motion trigger time inquiry
Returns 1 when advandsed mode
*/
int Radar_MR24HPC1::ask_motion_trigger_time() {
  if (advanced_mode) {
    const int len = 10;
    uint8_t frame[len] = {
    HEAD1, HEAD2, 0x08, 0x8C, 0x00, 0x01, 0x0F, 0x00, END1, END2};
      frame[I_DATA+1] = get_frame_sum(frame, len);
    send_query(frame, len);
  } else {
    return 0;
  }
  return 1;
}

/*
Motion to still time inquiry
Returns 1 when advandsed mode
*/
int Radar_MR24HPC1::ask_motion_to_still_time() {
  if (advanced_mode) {
    const int len = 10;
    uint8_t frame[len] = {
    HEAD1, HEAD2, 0x08, 0x8D, 0x00, 0x01, 0x0F, 0x00, END1, END2};
      frame[I_DATA+1] = get_frame_sum(frame, len);
    send_query(frame, len);
  } else {
    return 0;
  }
  return 1;
}

/*
Time for entering no person state inquiry
Returns 1 when advandsed mode
*/
int Radar_MR24HPC1::ask_no_person_time() {
  if (advanced_mode) {
    const int len = 10;
    uint8_t frame[len] = {
    HEAD1, HEAD2, 0x08, 0x8E, 0x00, 0x01, 0x0F, 0x00, END1, END2};
      frame[I_DATA+1] = get_frame_sum(frame, len);
    send_query(frame, len);
  } else {
    return 0;
  }
  return 1;
}



float Radar_MR24HPC1::calculate_distance(int val) {
  const double UNIT = 0.5;
  return val*UNIT;
}

float Radar_MR24HPC1::calculate_speed(int val) {
  const double UNIT = 0.5;

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
Takes time data in HEX array and retruns time in ms
size - 4 bytes
*/
int Radar_MR24HPC1::calculate_time(const unsigned char hex[], int size) {
  int decimal_value = 0;

  for (int i = 0; i < 4; i++) {
    decimal_value = (decimal_value << 8) | hex[i];
  }

  return decimal_value;  // ms
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
Full frames in
*/
uint8_t Radar_MR24HPC1::get_frame_sum(uint8_t *frame, int len) {
  unsigned int sum = 0;
  for (int i = 0; i < len - 3; i++) {
    sum += frame[i];
  }
  return sum & 0xff;
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
    advanced_mode = false;
  } else if (mode == ADVANCED) {
    stream->write(on_cmd, CMD_LEN);
    stream->flush();
    advanced_mode = true;
  }
}

/*
Returns radar mode:
1 - Advandced
0 - Simple
*/
int Radar_MR24HPC1::get_mode() {
  if (advanced_mode) {
    return ADVANCED;
  } else {
    return SIMPLE;
  }
}


/*
Runs on the loop 
*/
void Radar_MR24HPC1::run(bool mode) {
  read();  // Read new frames

  if (is_new_frame) {
    int control_word = frame[I_CONTROL_WORD];

    switch (control_word) {
      case 0x01:
        // run_01();
        break;
      case 0x02:
        // run_02();
        break;
      case 0x03:
        Serial.println("Radar: UART upgrade");
        break;
      case 0x05:
        // run_05();
        break;
      case 0x08:
        run_08(mode);
        break;
      case 0x80:
        // run_80(mode);
        break;
      default:
        //print();
        break;
    }
  }
  print();
}




/*
Translate the responses into human-readable form
TODO: Rename Verbal
*/
void Radar_MR24HPC1::translate() {
  read();
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
  int cmd_word = frame[I_CMD_WORD];
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
  } else if (cmd_word == 0x09) {  // Underlying
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

/////////////////////////
/*
run() helper func
Controll word 0x08
*/
void Radar_MR24HPC1::run_08(bool mode) {
  int cmd_word = frame[I_CMD_WORD];
  // int data_len = frame[I_LENGHT_L];
  int data = frame[I_DATA];
  const unsigned char time_data[4] = {frame[6], frame[7], frame[8], frame[9]};

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
    if (mode == VERBAL) {
      Serial.println("static boundary inquiry");
    }
  } else if (cmd_word == 0x8B) {
    switch (data) {
      case RANGE_50_CM:
        motion_trigger_range = 50;  // 50cm
        break;
      case RANGE_100_CM:
        motion_trigger_range = 100;
        break;
      case TIME_60_S:
        motion_trigger_range = 60;
        break;
      case TIME_2_MIN:
        motion_trigger_range = 120;
        break;
      case TIME_5_MIN:
        motion_trigger_range = 300;
        break;
      case TIME_10_MIN:
        motion_trigger_range = 600;
        break;
      case TIME_30_MIN:
        motion_trigger_range = 1800;
        break;
      case TIME_60_MIN:
        motion_trigger_range = 3600;
        break;
      default:
        break;
    }
    if (mode == VERBAL) {
      Serial.print("Motion trigger range: ");
      Serial.print(motion_trigger_range);
      Serial.println("s");
    }
  } else if (cmd_word == 0x8C) {
    motion_triger_time = calculate_time(time_data, 4);
    if (mode == VERBAL) {
      Serial.print("Motion trigger time: ");
      Serial.print(motion_triger_time);  // 0-1000ms
      Serial.println("ms");
    }
  } else if (cmd_word == 0x8D) {
    motion_to_still_time = calculate_time(time_data, 4);
    if (mode == VERBAL) {
      Serial.print("Motion to still time: ");
      Serial.print(motion_to_still_time);  // 1-60s
      Serial.println("ms");
    }
  } else if (cmd_word == 0x8E) {
    time_for_entering_no_person_state = calculate_time(time_data, 4);
    if (mode == VERBAL) {
      Serial.print("Time for entering no person state: ");
      Serial.print(time_for_entering_no_person_state);     // 0s to 3600s
      Serial.println("ms");
    }
  }
}
