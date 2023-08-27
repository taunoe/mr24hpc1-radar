#ifndef _Radar_MR24HPC1_H__
#define _Radar_MR24HPC1_H__

// Frame Headers
#define HEAD1            0x53  // Frame header 1
#define HEAD2            0x59  // Frame header 2
// End of frame
#define END1             0x54  // End of data frame 1
#define END2             0x43  // End of data frame 2

// Controll words
#define HEARTBEAT_STATUS 0x01
#define PRODUCT_STATUS   0x02
#define UART_UPGRADE     0x03
#define OPERATION_STATUS 0x05
#define DETAIL_STATUS    0x08  // Underlying parameters of the human state
#define HUMAN_STATUS     0x80  // Human Presence Reporting

// Human status (presence) command words
#define PRESENCE_REPORT  0x01  // Presence report //HUMAN_EXIST
#define MOTION_REPORT    0x02  // Motion report  //HUMAN_MOVE
#define MOVMENT_PARAM    0x03  // Body movment/Signs Parameters  //HUMAN_SIGN
#define NO_HUMAN_SET     0x0A  // Time for entering no person state setting
#define PROX_REPORT      0x0B  // Proximity report //HUMAN_DIRECT 

// Presense reporting data
#define UNOCCUPIED       0x00
#define OCCUPIED         0x01

// Motion reporting data
#define NONE             0x00
#define MOTIONLESS       0x01
#define ACTIVE           0x02

// Proximity reporting data
#define NEAR             0x01       //Someone approaches //CA_CLOSE
#define FAR              0x02       //Some people stay away //CA_AWAY

// Underlying Open function Command Word
#define SENSOR_REPORT    0x01       //DETAILINFO
#define DET_PROX_REPORT  0x06       //Human movement trends //DETAILDIRECT
#define DET_MOVMENT_PARA 0x07       //Body Signs Parameters //DETAILSIGN


//Return statues, Use in arduino
#define SOMEONE       0x01       //There are people
#define NOONE         0x02       //No one
#define NOTHING       0x03       //No message
#define SOMEONE_STOP  0x04       //Somebody stop
#define SOMEONE_MOVE  0x05       //Somebody move
#define HUMANPARA     0x06       //Body Signs Parameters
#define SOMEONE_CLOSE 0x07       //Someone approaches
#define SOMEONE_AWAY  0x08       //Some people stay away
#define DETAILMESSAGE 0x09       //Underlying parameters of the human state

#define UNIT          0.5        //Calculate unit steps

// Data bytes indexes
#define I_HEAD1        0  // Frame header 1
#define I_HEAD2        1
#define I_CONTROL_WORD 2  // Controll word index in data
#define I_CMD_WORD     3  // Command word index in data
#define I_LENGHT_H     4
#define I_LENGHT_L     5  // How many bytes of data
#define I_DATA         6  // Beginning of data bytes
//
#define DATA_SIZE     14  // Max data frame size in bytes

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
        bool is_new_data;
        uint8_t data_len;

        unsigned char data[DATA_SIZE] = {0};  // data returned by the radar up to a maximum length
        int count = 0;
        int checkdata_len = 2;        // Without cyclic sending, number of frames sent

        float calculate_distance(int val);
        float calculate_speed(int val);
        int hex_to_int(const unsigned char *hexChar);
        char hex_to_char(const unsigned char *hex);

    public:
        int status_msg = 0;     // Status message
        int bodysign_val = 0;

        int static_energy = 0;     // Spatial static values
        float static_dist = 0.0;   // Distance to stationary object m
        int motion_energy = 0;
        float motion_dist = 0.0;   // Distance from the moving object m
        float motion_speed = 0.0;  // Speed of moving object m/s

        Radar_MR24HPC1(Stream *s);

        void read();
        void print(int mode = HEX);
        void analys(bool show_bodysign);
        void write_cmd(const unsigned char* buff, int len, bool cyclic = false);  // checkSetMode_func
        int reset();
        int get_id();
        int firm_ver_id();
        void print_hex(const unsigned char* buff, int len);
        void print_dec(const unsigned char* buff, int len);
        // Calculate checksum
        uint8_t calculate_sum(const unsigned char data[], int size);
        bool is_data_good(const unsigned char data[]);
};

#endif
