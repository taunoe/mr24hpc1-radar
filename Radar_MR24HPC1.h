#ifndef _Radar_MR24HPC1_H__
#define _Radar_MR24HPC1_H__

#define HEAD1         0x53  // Frame header 1
#define HEAD2         0x59  // Frame header 2
#define END1          0x54  // End of data frame 1
#define END2          0x43  // End of data frame 2

#define HUMANSTATUS   0x80       //Human Presence Information
#define HUMANEXIST    0x01       //Presence of the human body
#define HUMANMOVE     0x02       //Human movement information
#define HUMANSIGN     0x03       //Body Signs Parameters
#define HUMANDIRECT   0x0B       //Human movement trends

#define SOMEBODY      0x01       //Somebody move
#define NOBODY        0x00       //No one here

#define NONE          0x00
#define SOMEBODY_STOP 0x01       //Somebody stop
#define SOMEBODY_MOVE 0x02       //Somebody move

#define CA_CLOSE      0x01       //Someone approaches
#define CA_AWAY       0x02       //Some people stay away


#define DETAILSTATUS  0x08       //Underlying parameters of the human state
#define DETAILINFO    0x01       //Detailed data on the state of human movement
#define DETAILDIRECT  0x06       //Human movement trends
#define DETAILSIGN    0x07       //Body Signs Parameters

//Return status, Use in arduino
#define SOMEONE       0x01       //There are people
#define NOONE         0x02       //No one
#define NOTHING       0x03       //No message
#define SOMEONE_STOP  0x04       //Somebody stop
#define SOMEONE_MOVE  0x05       //Somebody move
#define HUMANPARA     0x06       //Body Signs Parameters
#define SOMEONE_CLOSE 0x07       //Someone approaches
#define SOMEONE_AWAY  0x08       //Some people stay away
#define DETAILMESSAGE 0x09       //Underlying parameters of the human state

#define unit          0.5        //Calculate unit steps

// Reset cmd
#define CMD_LEN 10       // Reset data frame length
const unsigned char reset_cmd[CMD_LEN] = {
    HEAD1, HEAD2, 0x01, 0x02, 0x00, 0x01, 0x0F, 0xBF, END1, END2 };

// Get id cmd
const unsigned char id_cmd[CMD_LEN] = {
    HEAD1, HEAD2, 0x02, 0xA2, 0x00, 0x01, 0x0F, 0x60, END1, END2 };

const unsigned char firmware_ver_cmd[CMD_LEN] = {
    HEAD1, HEAD2, 0x02, 0xA4, 0x00, 0x01, 0x0F, 0x62, END1, END2 };

class Radar_MR24HPC1 {

    private:
        Stream *stream;               // e.g. SoftwareSerial or Serial1
        boolean is_new_data;
        byte data_len;

        unsigned char data[20] = {0};  // data returned by the radar up to a maximum length of 20
        int count = 0;
        int checkdata_len = 2;        // Without cyclic sending, number of frames sent
        void print_hex(const unsigned char* buff, int len);
        void print_dec(const unsigned char* buff, int len);
        float decodeVal_func(int val, bool decode = false);

        int hex_to_int(const char *hex);
        char hex_to_char(const char *hex);

    public:
        int status = 0;
        int bodysign_val = 0;
        int static_val = 0;
        int dynamic_val = 0;
        float dis_static = 0.0;
        float dis_move = 0.0;
        float speed = 0.0;

        Radar_MR24HPC1(Stream *s);

        void read();
        void print(int mode = HEX);
        void HumanStatic_func(bool bodysign = false);
        void write_cmd(const unsigned char* buff, int len, bool cyclic = false);  // checkSetMode_func
        int reset();
        int get_id();
        int firm_ver_id();
};

#endif
