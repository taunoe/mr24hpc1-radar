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
Send Reset frame
*/
void Radar_MR24HPC1::ask_reset() {
  const int len = 10;
  uint8_t frame[len] = {
    HEAD1, HEAD2, 0x01, 0x02, 0x00, 0x01, 0x0F, 0xBF, END1, END2};
  frame[I_DATA+1] = get_frame_sum(frame, len);

  send_query(frame, len);
}

/*
Send heartbeat frame
*/
void Radar_MR24HPC1::ask_heartbeat() {
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

  if (mode == ADVANCED) {
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

  if (mode == ADVANCED) {
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
Advandsed mode ==
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
  if (mode == ADVANCED) {
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
  if (mode == ADVANCED) {
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
  if (mode == ADVANCED) {
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
  if (mode == ADVANCED) {
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
  if (mode == ADVANCED) {
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
  if (mode == ADVANCED) {
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
  if (mode == ADVANCED) {
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
  if (mode == ADVANCED) {
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

/*
hex to cm
*/
int Radar_MR24HPC1::calculate_distance_cm(uint8_t data) {
  int UNIT = 50;     // cm
  return data*UNIT;  // cm
  /*
  switch (data) {
    case RANGE_50_CM:
      return 50;  // 50cm
      break;
    case RANGE_100_CM:
      return 100;
      break;
    case RANGE_150_CM:
      return 150;
      break;
    case RANGE_200_CM:
      return 200;
      break;
    case RANGE_250_CM:
      return 250;
      break;
    case RANGE_300_CM:
      return 300;
      break;
    case RANGE_350_CM:
      return 350;
      break;
    case RANGE_400_CM:
      return 400;
      break;
    case RANGE_450_CM:
      return 450;
      break;
    case RANGE_500_CM:
      return 500;
      break;
    default:
      return 0;
      break;
  }*/
}


/*
Hex to meters
*/
float Radar_MR24HPC1::calculate_distance_m(int val) {
  const double UNIT = 0.5;
  return val*UNIT;  // m
}

/*
Hex to speed
*/
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
void Radar_MR24HPC1::set_mode(int newmode) {
  const unsigned char on_cmd[CMD_LEN] = {
    HEAD1, HEAD2, 0x08, 0x00, 0x00, 0x01, 0x01, 0xB6, END1, END2 };

  const unsigned char off_cmd[CMD_LEN] = {
    HEAD1, HEAD2, 0x08, 0x00, 0x00, 0x01, 0x00, 0xB5, END1, END2 };

  if (newmode == SIMPLE) {
    stream->write(off_cmd, CMD_LEN);
    stream->flush();
    mode = SIMPLE;
  } else if (newmode == ADVANCED) {
    stream->write(on_cmd, CMD_LEN);
    stream->flush();
    mode = ADVANCED;
  }
}

/*
Returns radar mode:
1 - Advandced
0 - Simple
*/
int Radar_MR24HPC1::get_mode() {
  return mode;
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
        run_01(mode);
        break;
      case 0x02:
        run_02(mode);
        break;
      case 0x03:
        Serial.println("Radar: UART upgrade");
        break;
      case 0x05:
        run_05(mode);
        break;
      case 0x08:
        run_08(mode);
        break;
      case 0x80:
        run_80(mode);
        break;
      default:
        print();
        break;
    }
  }
}


/*
Controll word 0x01
Hearbeat, reset
*/
void Radar_MR24HPC1::run_01(bool mode) {
  int cmd_word = frame[I_CMD_WORD];
  int data = frame[I_DATA];

  switch (cmd_word) {
    case 0x01:  // heartbeat
      heartbeat++;
      break;
    case 0x02:  // reset
      Serial.println("Radar Reset!");
      break;
    default:
      break;
  }
}

/*
Controll word 0x02
Product info
*/
void Radar_MR24HPC1::run_02(bool mode) {
  int cmd_word = frame[I_CMD_WORD];
  int len = frame[I_LENGHT_L];
  // int data = frame[I_DATA];

  switch (cmd_word) {
    case 0xA1:
      Serial.print("Product Model ");
      break;
    case 0xA2:
      Serial.print("Product ID ");
      break;
    case 0xA3:
      Serial.print("Hardware Model ");
      break;
    case 0xA4:
      Serial.print("Firmware version ");
      break;
    default:
      print();
      break;
  }
}

/*
Controll word 0x05
Work status
*/
void Radar_MR24HPC1::run_05(bool mode) {
  int cmd_word = frame[I_CMD_WORD];

  switch (cmd_word) {
    case 0x01:
      // Work status: init completed
      run_05_cmd_0x01(mode);
      break;
    case 0x07:
      // Motion range response
      run_05_cmd_0x07(mode);
      break;
    case 0x08:
      // Static range response
      run_05_cmd_0x08(mode);
      break;
    case 0x09:
      // Custom mode setting
      run_05_cmd_0x09(mode);
      break;
    case 0x81:
      // Initializatio status inquiry response
      run_05_cmd_0x81(mode);
      break;
    case 0x84:
      // Motion distance inquiry response
      run_05_cmd_0x84(mode);
      break;
    case 0x85:
      // Motion speed inquiry response
      run_05_cmd_0x85(mode);
      break;
    case 0x87:
      // Scene settings inquiry response
      run_05_cmd_0x87(mode);
      break;
    case 0x88:
      // Static range settings response
      run_05_cmd_0x88(mode);
      break;
    case 0x89:
      // Custom mode inquiry
      run_05_cmd_0x89(mode);
      break;
    case 0x0A:
      // End of ustom mode setting response
      run_05_cmd_0x0A(mode);
      break;
    default:
      print();
      break;
  }
}

/*
Initialization completed
*/
void Radar_MR24HPC1::run_05_cmd_0x01(bool mode) {
  initialization_status = 0x01;  // Completed

  if (mode == VERBAL) {
    Serial.println("Initialization completed.");
  }
}

/*
Scene settings range response
Motion trigger range response
*/
void Radar_MR24HPC1::run_05_cmd_0x07(bool mode) {
  if (frame[I_DATA] == 0x01) {
    // Living room 4-4.5m
    motion_trigger_range = 450;  // cm
  } else if (frame[I_DATA] == 0x02) {
    // Bedroom 4m
    motion_trigger_range = 400;  // cm
  } else if (frame[I_DATA] == 0x03) {
    // Bathroom 3m
    motion_trigger_range = 300;  // cm
  } else if (frame[I_DATA] == 0x04) {
    // area detection 3.5m
    motion_trigger_range = 350;  // cm
  } else if (frame[I_DATA] == 0x00) {
    motion_trigger_range = 0;
  }

  if (mode == VERBAL) {
    Serial.print("Motion trigger range: ");
    Serial.print(motion_trigger_range);
    Serial.println(" cm");
  }
}

/*
Sensitivity settings range response
Static trigger range response
*/
void Radar_MR24HPC1::run_05_cmd_0x08(bool mode) {
  if (frame[I_DATA] == 0x01) {
    // Level 1
    static_trigger_range = 250;  // cm
  } else if (frame[I_DATA] == 0x02) {
    // Level 2
    static_trigger_range = 300;  // cm
  } else if (frame[I_DATA] == 0x03) {
    // Level 3
    static_trigger_range = 400;  // cm
  } else if (frame[I_DATA] == 0x00) {
    // Level 0
    static_trigger_range = 0;  // cm
  }

  if (mode == VERBAL) {
    Serial.print("Static trigger range: ");
    Serial.print(static_trigger_range);
    Serial.println(" cm");
  }
}

/*
Custom mode setting response
0x01 to 0x04
*/
void Radar_MR24HPC1::run_05_cmd_0x09(bool mode) {
  custom_mode = frame[I_DATA];

  if (mode == VERBAL) {
    Serial.print("Sellected custom mode: ");
    Serial.println(custom_mode);
  }
}

/*
Initialization status response
0x01 Completed
0x02 Incompleted
*/
void Radar_MR24HPC1::run_05_cmd_0x81(bool mode) {
  initialization_status = frame[I_DATA];

  if (mode == VERBAL) {
    if (initialization_status == 0x01) {
      Serial.println("Initialization completed.");
    } else {
      Serial.println("Initialization incompleted.");
    }
  }
}



/*
Motion distance inquiry response
*/
void Radar_MR24HPC1::run_05_cmd_0x84(bool mode) {
  uint8_t data = frame[I_DATA];
  motion_distance = calculate_distance_cm(data);

  if (mode == VERBAL) {
    Serial.print("Motion distance: ");
    Serial.print(motion_distance);  // 0-4m
    Serial.println("cm");
  }
}

/*
Motion speed inquiry response
*/
void Radar_MR24HPC1::run_05_cmd_0x85(bool mode) {
  uint8_t data = frame[I_DATA];
  motion_speed = calculate_speed(data);

  if (mode == VERBAL) {
    Serial.print("Motion speed: ");
    Serial.print(motion_speed);
    Serial.println("m/cm");
  }
}

/*
Scene settings response
0x00 Scene mode not set
0x01 Living room
0x02 Bedroom
0x03 Bathroom
0x04 Area detection
*/
void Radar_MR24HPC1::run_05_cmd_0x87(bool mode) {
  if (frame[I_DATA] == 0x01) {
    // Living room 4-4.5m
    motion_trigger_range = 450;  // cm
  } else if (frame[I_DATA] == 0x02) {
    // Bedroom 4m
    motion_trigger_range = 400;  // cm
  } else if (frame[I_DATA] == 0x03) {
    // Bathroom 3m
    motion_trigger_range = 300;  // cm
  } else if (frame[I_DATA] == 0x04) {
    // area detection 3.5m
    motion_trigger_range = 350;  // cm
  } else if (frame[I_DATA] == 0x00) {
    motion_trigger_range = 0;
  }

  if (mode == VERBAL) {
    Serial.print("Motion trigger range: ");
    Serial.print(motion_trigger_range);
    Serial.println(" cm");
  }
}

/*
Sensitivity settings response
Static range settings response
0x00 not set
0x01 level 1
0x02 level 2
0x03 level 3
*/
void Radar_MR24HPC1::run_05_cmd_0x88(bool mode) {
  if (frame[I_DATA] == 0x01) {
    // Level 1
    static_trigger_range = 250;  // cm
  } else if (frame[I_DATA] == 0x02) {
    // Level 2
    static_trigger_range = 300;  // cm
  } else if (frame[I_DATA] == 0x03) {
    // Level 3
    static_trigger_range = 400;  // cm
  } else if (frame[I_DATA] == 0x00) {
    // Level 0
    static_trigger_range = 0;  // cm
  }

  if (mode == VERBAL) {
    Serial.print("Static trigger range: ");
    Serial.print(static_trigger_range);
    Serial.println(" cm");
  }
}

/*
Custom mode inquiry response
0x01 to 0x04
*/
void Radar_MR24HPC1::run_05_cmd_0x89(bool mode) {
  custom_mode = frame[I_DATA];

  if (mode == VERBAL) {
    Serial.print("Custom mode: ");
    Serial.println(custom_mode);
  }
}

/*
End of ustom mode setting response
*/
void Radar_MR24HPC1::run_05_cmd_0x0A(bool mode) {
  if (mode == VERBAL) {
    Serial.println("Custom mode settings saved!");
  }
}


/*
run() helper func
Controll word 0x08
*/
void Radar_MR24HPC1::run_08(bool mode) {
  int cmd_word = frame[I_CMD_WORD];

  switch (cmd_word) {
    case 0x00:
      run_08_cmd_0x00(mode);  // Output switch
      break;
    case 0x01:
      run_08_cmd_0x01(mode);  // Sensor report
      break;
    case 0x08:
      run_08_cmd_0x08(mode);  // Static energy threshold
      break;
    case 0x09:
      run_08_cmd_0x09(mode);  // Motion energy threshold settings
      break;
    case 0x80:
      run_08_cmd_0x80(mode);  // Output switch inquiry
      break;
    case 0x81:
      run_08_cmd_0x81(mode);  // Static energy
      break;
    case 0x82:
      run_08_cmd_0x82(mode);  // Motion energy
      break;
    case 0x83:
      run_08_cmd_0x83(mode);  // Static distance inquiry
      break;
    case 0x88:
      run_08_cmd_0x88(mode);  // Static energy threshold
      break;
    case 0x89:
      run_08_cmd_0x89(mode);  // Motion energy threshold
      break;
    case 0x0A:
      run_08_cmd_0x0A(mode);  // Static trigger range settings
      break;
    case 0x8A:
      run_08_cmd_0x8A(mode);  // Static trigger range
      break;
    case 0x0B:
      run_08_cmd_0x0B(mode);  // Motion trigger range settings
      break;
     case 0x8B:
      run_08_cmd_0x8B(mode);  // Motion trigger range
      break;
    case 0x0C:
      run_08_cmd_0x0C(mode);  // Trigger time setting
      break;
    case 0x8C:
      run_08_cmd_0x8C(mode);  // Motion trigger time
      break;
    case 0x0D:
      run_08_cmd_0x0D(mode);  // Still time setting
      break;
    case 0x8D:
      run_08_cmd_0x8D(mode);  // Motion to still timeg
      break;
    case 0x0E:
      run_08_cmd_0x0E(mode);  // Time for entering no person state
      break;
    case 0x8E:
      run_08_cmd_0x8E(mode);  // Time for entering no person state
      break;
    default:
      // print();
      break;
  }
}

/*
Advandced mode: ON/OFF
*/
void Radar_MR24HPC1::run_08_cmd_0x00(bool mode) {
  if (frame[I_DATA] == 0x01) {
    mode = ADVANCED;
    if (mode == VERBAL) {
      Serial.println("Advandced mode: ON");
    }
  } else {
    mode = SIMPLE;
    if (mode == VERBAL) {
      Serial.println("Advandced mode: OFF");
    }
  }
}

/*
Reporting of sensor information
*/
void Radar_MR24HPC1::run_08_cmd_0x01(bool mode) {
  static_energy   = frame[I_DATA];
  static_distance = frame[I_DATA+1];
  motion_energy   = frame[I_DATA+2];
  motion_distance = frame[I_DATA+3];
  motion_speed    = frame[I_DATA+4];

  if (mode == VERBAL) {
    Serial.print("Static energy: ");
    Serial.println(static_energy);  // 0-250

    Serial.print("Static distance: ");
    Serial.print(static_distance);  // 0-3m
    Serial.println("cm");

    Serial.print("Motion energy: ");
    Serial.println(motion_energy);  // 0-250

    Serial.print("Motion distance: ");
    Serial.print(motion_distance);  // 0-4m
    Serial.println("cm");

    Serial.print("Motion speed: ");
    Serial.print(motion_speed);
    Serial.println("m/cm");
  }
}

/*
Advandced mode: ON/OFF
*/
void Radar_MR24HPC1::run_08_cmd_0x80(bool mode) {
  if (frame[I_DATA] == 0x01) {
    mode = ADVANCED;
    if (mode == VERBAL) {
      Serial.println("Advandced mode: ON");
    }
  } else {
    mode = SIMPLE;
    if (mode == VERBAL) {
      Serial.println("Advandced mode: OFF");
    }
  }
}

/*
Static energy value inquiry
*/
void Radar_MR24HPC1::run_08_cmd_0x81(bool mode) {
  static_energy = frame[I_DATA];

  if (mode == VERBAL) {
    Serial.print("Static energy: ");
    Serial.println(static_energy);  // 0-250
  }
}

/*
Motion energy value inquiry
*/
void Radar_MR24HPC1::run_08_cmd_0x82(bool mode) {
  motion_energy = frame[I_DATA];

  if (mode == VERBAL) {
    Serial.print("Motion energy: ");
    Serial.println(motion_energy);  // 0-250
  }
}

/*
Static distance inquiry
*/
void Radar_MR24HPC1::run_08_cmd_0x83(bool mode) {
  uint8_t data = frame[I_DATA];
  static_distance = calculate_distance_cm(data);

  if (mode == VERBAL) {
    Serial.print("Static distance: ");
    Serial.print(static_distance);  // 0-3m
    Serial.println("cm");
  }
}



/*
Static energy threshold
*/
void Radar_MR24HPC1::run_08_cmd_0x88(bool mode) {
  static_energy_threshold = frame[I_DATA];

  if (mode == VERBAL) {
    Serial.print("Static energy threshold: ");
    Serial.println(static_energy_threshold);  // 0-250
  }
}

void Radar_MR24HPC1::run_08_cmd_0x08(bool mode) {
  run_08_cmd_0x88(mode);
}

/*
Motion energy threshold
*/
void Radar_MR24HPC1::run_08_cmd_0x89(bool mode) {
  motion_energy_threshold = frame[I_DATA];

  if (mode == VERBAL) {
    Serial.print("Motion energy threshold: ");
    Serial.println(motion_energy_threshold);  // 0-250
  }
}

void Radar_MR24HPC1::run_08_cmd_0x09(bool mode) {
  run_08_cmd_0x89(mode);
}

/*
Static trigger range
*/
void Radar_MR24HPC1::run_08_cmd_0x8A(bool mode) {
  uint8_t data = frame[I_DATA];
  static_trigger_range = calculate_distance_cm(data);

  if (mode == VERBAL) {
    Serial.print("Static trigger range: ");
    Serial.print(static_trigger_range);  // 0-1000ms
    Serial.println("cm");
  }
}

void Radar_MR24HPC1::run_08_cmd_0x0A(bool mode) {
  run_08_cmd_0x8A(mode);
}

/*
Motion trigger range
*/
void Radar_MR24HPC1::run_08_cmd_0x8B(bool mode) {
  uint8_t data = frame[I_DATA];
  motion_trigger_range = calculate_distance_cm(data);

  if (mode == VERBAL) {
    Serial.print("Motion trigger range: ");
    Serial.print(motion_trigger_range);
    Serial.println("cm");
  }
}

void Radar_MR24HPC1::run_08_cmd_0x0B(bool mode) {
  run_08_cmd_0x8B(mode);
}

/*
Motion trigger time
*/
void Radar_MR24HPC1::run_08_cmd_0x0C(bool mode) {
  const unsigned char time_data[4] = {frame[6], frame[7], frame[8], frame[9]};
  motion_triger_time = calculate_time(time_data, 4);

  if (mode == VERBAL) {
    Serial.print("Motion trigger time: ");
    Serial.print(motion_triger_time);  // 0-1000ms
    Serial.println("ms");
  }
}

/*
Motion trigger time
*/
void Radar_MR24HPC1::run_08_cmd_0x8C(bool mode) {
  const unsigned char time_data[4] = {frame[6], frame[7], frame[8], frame[9]};
  motion_triger_time = calculate_time(time_data, 4);

  if (mode == VERBAL) {
    Serial.print("Motion trigger time: ");
    Serial.print(motion_triger_time);  // 0-1000ms
    Serial.println("ms");
  }
}

/*
Motion to still time setting
*/
void Radar_MR24HPC1::run_08_cmd_0x0D(bool mode) {
  const unsigned char time_data[4] = {frame[6], frame[7], frame[8], frame[9]};
  motion_to_still_time = calculate_time(time_data, 4);

  if (mode == VERBAL) {
    Serial.print("Motion to still time: ");
    Serial.print(motion_to_still_time);  // 1-60s
    Serial.println("ms");
  }
}

/*
Motion to still time
*/
void Radar_MR24HPC1::run_08_cmd_0x8D(bool mode) {
  const unsigned char time_data[4] = {frame[6], frame[7], frame[8], frame[9]};
  motion_to_still_time = calculate_time(time_data, 4);

  if (mode == VERBAL) {
    Serial.print("Motion to still time: ");
    Serial.print(motion_to_still_time);  // 1-60s
    Serial.println("ms");
  }
}

/*
Time for entering no person state
*/
void Radar_MR24HPC1::run_08_cmd_0x8E(bool mode) {
  const unsigned char time_data[4] = {frame[6], frame[7], frame[8], frame[9]};
  time_for_entering_no_person_state = calculate_time(time_data, 4);

  if (mode == VERBAL) {
    Serial.print("Time for entering no person state: ");
    Serial.print(time_for_entering_no_person_state);     // 0s to 3600s
    Serial.println("ms");
  }
}

void Radar_MR24HPC1::run_08_cmd_0x0E(bool mode) {
  run_08_cmd_0x8E(mode);
}

/*
Controll word 0x80
*/
void Radar_MR24HPC1::run_80(bool mode) {
  int cmd_word = frame[I_CMD_WORD];
  int data = frame[I_DATA];

  switch (cmd_word) {
    case 0x01:  // presence
      run_80_cmd_0x01(mode);
      break;
    case 0x02:  // motion
      run_80_cmd_0x02(mode);
      break;
    case 0x03:  // body parameter
      run_80_cmd_0x03(mode);
      break;
    case 0x81:  // presence
    run_80_cmd_0x81(mode);
      break;
    case 0x82:  // motion
      run_80_cmd_0x82(mode);
      break;
    case 0x83:  // body parameter
      run_80_cmd_0x83(mode);
      break;
    case 0x0A:  // setting time for no person state
      run_80_cmd_0x0A(mode);
      break;
    case 0x0B:  // proximity setting
      run_80_cmd_0x0B(mode);
      break;
    case 0x8A:  // setting time for no person state inquiry
      run_80_cmd_0x8A(mode);
      break;
    case 0x8B:  // proximity inquiry
      run_80_cmd_0x8B(mode);
      break;
    default:
      break;
  }
}

/*
Active reporting of presence information report
0x00 Unoccupied
0x01 Occupied
*/
void Radar_MR24HPC1::run_80_cmd_0x01(bool mode = NONVERBAL) {
  occupation = frame[I_DATA];

  if (mode == VERBAL) {
    if (occupation == OCCUPIED) {
      Serial.println("Occupied!");
    }
  }
}

/*
Active reporting of motion information report
0x00 none
0x01 Static
0x02 Active
*/
void Radar_MR24HPC1::run_80_cmd_0x02(bool mode = NONVERBAL) {
  motion = frame[I_DATA];

  if (mode == VERBAL) {
    switch (motion) {
      case STATIC:
        Serial.println("Static");
        break;
      case ACTIVE:
        Serial.println("Active");
        break;
      default:
        Serial.println("None");
        break;
    }
  }
}

/*
Active reporting of body movement parameter report
0-100
*/
void Radar_MR24HPC1::run_80_cmd_0x03(bool mode = NONVERBAL) {
  activity = frame[I_DATA];

  if (mode == VERBAL) {
    Serial.print("Activity: ");
    Serial.println(activity);
  }
}
