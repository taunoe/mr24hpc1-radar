#include "Arduino.h"
#include "Radar_MR24HPC1.h"

Radar_MR24HPC1::Radar_MR24HPC1(Stream *s)
    : stream(s){
  this->is_new_data = false;
}

/*
  Receive radar data
*/ 
void Radar_MR24HPC1::read() {
  while (stream->available()) {

    if(stream->read() == HEAD1) {

      if(stream->read() == HEAD2) {
        // Read data
        data_len = stream->readBytesUntil(END2, data, 20);

        if (data_len > 0 && data_len < 20){
          data[data_len] = END2; // last byte
          is_new_data = true;
        }

      }

    }

  }
}


/*
Print radar data on serial monitor
mode: HEX, DEC, CHAR
Adds headers
*/
void Radar_MR24HPC1::print(int mode) {
  if(is_new_data) {

    // Print headers
    Serial.print(HEAD1, mode);
    Serial.print(' ');
    Serial.print(HEAD2, mode);

    switch (mode) {
      case DEC:
        Serial.print(' ');
        print_dec(data, data_len);
        break;
        break;
      default: // HEX
        Serial.print(' ');
        print_hex(data, data_len);
        break;
    }

    is_new_data = false;
    data[data_len] = {0};  // set data to zero
  }
}


/*
  Serial Print data in HEX format
  buff - data char array
  len - data char lenght
*/
void Radar_MR24HPC1::print_hex(const unsigned char* buff, int len) {
  char charVal[4];

  for(int i=0; i<=len; i++){
    sprintf(charVal, "%02X", buff[i]);
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

  for(int i=0; i<len; i++){
    sprintf(char_val, "%d", buff[i]);
    Serial.print(char_val);
    Serial.print(' ');
  }
  
  Serial.println();
}

/*

*/
void Radar_MR24HPC1::analys(bool show_bodysign) {
  read();

  if (is_new_data) {
    Serial.println("Uus:");
    
    // Read control word
    int control_word = data[I_CONTROL_WORD];

    print();

    if (control_word == HUMAN_STATUS) {
      int report = data[I_CMD_WORD];  // Read command word

      if (report == PRESENCE_REPORT) {
        // Read data byte
        int d = data[4];  // OCCUPIED, UNOCCUPIED
        if (d == 0) {
          status_msg = NOONE;
        }
        else { // == 1
          status_msg = SOMEONE;
        }
      }
      else if (report == MOTION_REPORT) {
        int d = data[4];  // NONE, MOTIONLESS, ACTIVE
        switch (d) {
          case NONE:
            status_msg = NOTHING; break;
          case MOTIONLESS:
            status_msg = SOMEONE_STOP; break;
          case ACTIVE:
            status_msg = SOMEONE_MOVE; break;
        }
      }
      else if (report == MOVMENT_PARAM) {
        status_msg = HUMANPARA;
        bodysign_val = data[4];
      }
      else if (report == PROX_REPORT) {
        int d = data[4]; // NONE, NEAR, FAR
        switch (d) {
          case NONE:
            status_msg = NOTHING; break;
          case NEAR:
            status_msg = SOMEONE_CLOSE; break;
          case FAR:
            status_msg = SOMEONE_AWAY; break;
        }
      }
    }
    else if (control_word == DETAIL_STATUS) {
      int report = data[I_CMD_WORD];  // Read command word

      if (report == SENSOR_REPORT) {
        //int d = data[4];
        status_msg = DETAILMESSAGE;
        static_val = data[4];
        dynamic_val = data[5];
        dis_static = calgulate_distance(data[6]);
        dis_move = calgulate_distance(data[7]);
        speed = calgulate_speed(data[8]);
      }
      else if (report == DET_PROX_REPORT) {
        int d = data[4]; // NONE, NEAR, FAR
        switch(d) {
          case NONE:
            status_msg = NOTHING; break;
          case NEAR:
            status_msg = SOMEONE_CLOSE; break;
          case FAR:
            status_msg = SOMEONE_AWAY; break;
        }
      }
      else if (report == DET_MOVMENT_PARA) {
        status_msg = HUMANPARA;
        bodysign_val = data[4];
      }
    }

  }
  is_new_data = false;
}



//Send data frame
void Radar_MR24HPC1::write_cmd(const unsigned char* buff, int len, bool cyclic /*=false*/) {

  if (cyclic || count < checkdata_len) {

    if (cyclic || count < 1) {
      stream->write(buff, len);
      stream->flush();
    }

    // read radar responce
    do {
      read();
      delay(20);
    } while (!(this->is_new_data));

    if (cyclic || count < 1) {
      Serial.print("  Sent  ---> ");
      print_hex(buff, len);
    }

    if (count%2 == 1) {
      Serial.print("Receive <--- ");
      print();
    }

    is_new_data = false;
  }
  count++;
}

/*
Return Radar ID
*/
int Radar_MR24HPC1::get_id() {
  stream->write(id_cmd, CMD_LEN);
  stream->flush();

  // read radar responce
  do {
    read();
    //delay(10);
  } while (!(is_new_data));

  print(DEC);
  is_new_data = false;
  // TODO: return id
  return 0;
}

int Radar_MR24HPC1::firm_ver_id() {
  stream->write(firmware_ver_cmd, CMD_LEN);
  stream->flush();

  // read radar responce
  do {
    read();
    //delay(10);
  } while (!(is_new_data));

  print();
  is_new_data = false;
  // TODO: return id
  return 0;
}

/*
Reset radar
returns 1
*/
int Radar_MR24HPC1::reset() {
  stream->write(reset_cmd, CMD_LEN);
  stream->flush();
  //Serial.println("Radar reset!");
  return 1;
}


float Radar_MR24HPC1::calgulate_distance(int val) {
  return val*UNIT;
}

float Radar_MR24HPC1::calgulate_speed(int val) {
  if (val == 0x0A) {
    return 0;
  }
  else if (val > 0x0A) {
    // Negative speed
    return -((val-10)*UNIT);
  }
  else if (val < 0x0A) {
    // Positive speed
    return (val)*UNIT;
  }
  return 0;
}


/*
Converts the hexadecimal string to an integer
*/
int Radar_MR24HPC1::hex_to_int(const char *hex) {
  // Use strtol to convert the hexadecimal string to an integer
  return strtol(hex, NULL, 16);
}

/*
Converts the hexadecimal string to an char
*/
char Radar_MR24HPC1::hex_to_char(const char *hex) {
  int int_val = hex_to_int(hex);
  // Cast the integer value to a char
  char char_val = (char)int_val;

  return char_val;
}