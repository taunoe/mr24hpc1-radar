#include "Arduino.h"
#include "Radar_MR24HPC1.h"

Radar_MR24HPC1::Radar_MR24HPC1(Stream *s)
    : stream(s){
  this->is_new_data = false;
}

// Receive data and process
void Radar_MR24HPC1::read() {
  while (stream->available()) {
    if(stream->read() == HEAD1) {
      if(stream->read() == HEAD2) {
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
    data[data_len] = {0}; // set to all zero
  }
}


/*
  Serial Print data in HEX format
  buff - data char array
  len - data char lenght
*/
void Radar_MR24HPC1::print_hex(const unsigned char* buff, int len) {
  char charVal[4];

  for(int i=0; i<len; i++){
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


//Parsing data frames
void Radar_MR24HPC1::HumanStatic_func(bool bodysign /*=false*/){
  read();

  status = 0;
  bodysign_val = 0x00;
  static_val = 0x00;
  dynamic_val = 0x00;
  dis_static = 0x00;
  dis_move = 0x00;
  speed = 0x00;

  if(is_new_data){
    switch (data[0]) {
      case HUMANSTATUS:
        switch (data[1])
        {
          case HUMANEXIST:
            switch (data[4])
            {
              case SOMEBODY:
                print();
                status = SOMEONE;
                break;
              case NOBODY:
                print();
                status = NOONE;
                break;
            }
            break;
          case HUMANMOVE:
            switch (data[4])
            {
              case NONE:
                print();
                status = NOTHING;
                break;
              case SOMEBODY_STOP:
                print();
                status = SOMEONE_STOP;
                break;
              case SOMEBODY_MOVE:
                print();
                status = SOMEONE_MOVE;
                break;
            }
            break;
          case HUMANSIGN:
            if(bodysign){
              print();
              status = HUMANPARA;
              bodysign_val = data[4];
            }
            break;
          case HUMANDIRECT:
            switch (data[4])
            {
              case NONE:
                print();
                status = NOTHING;
                break;
              case CA_CLOSE:
                print();
                status = SOMEONE_CLOSE;
                break;
              case CA_AWAY:
                print();
                status = SOMEONE_AWAY;
                break;
            }
            break;
        }
        break;
      case DETAILSTATUS:
        switch(data[1]){
          case DETAILINFO:
            print();
            status = DETAILMESSAGE;
            static_val = data[4];
            dynamic_val = data[5];
            dis_static = decodeVal_func(data[6]);
            dis_move = decodeVal_func(data[7]);
            speed = decodeVal_func(data[8],true);
            break;
          case DETAILDIRECT:
            switch(data[4]){
              case NONE:
                print();
                status = NOTHING;
                break;
              case CA_CLOSE:
                print();
                status = SOMEONE_CLOSE;
                break;
              case CA_AWAY:
                print();
                status = SOMEONE_AWAY;
                break;
            }
            break;
          case DETAILSIGN:
            if(bodysign){
              print();
              status = HUMANPARA;
              bodysign_val = data[4];
            }
            break;
        }
        break;
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



float Radar_MR24HPC1::decodeVal_func(int val, bool decode){
  if(!decode) {
    return val*unit;   //Calculate distance
  }
  else {                          //Calculate speed
    if(val == 0x0A) {
      return 0;
    }
    else if(val > 0x0A) {
      return -((val-10)*unit);   //Away speed is negative
    }
    else if(val < 0x0A) {
      return (val)*unit;         //Approach speed is positive
    }
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