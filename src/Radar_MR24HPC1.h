/*
Copyright 2023 Tauno Erik
*/

#ifndef LIB_RADAR_MR24HPC1_SRC_RADAR_MR24HPC1_H_
#define LIB_RADAR_MR24HPC1_SRC_RADAR_MR24HPC1_H_

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
#define NEAR             0x01       // Someone approaches //CA_CLOSE
#define FAR              0x02       // Some people stay away //CA_AWAY

// Underlying Open function Command Word
#define SENSOR_REPORT    0x01       // DETAILINFO
#define DET_PROX_REPORT  0x06       // Human movement trends //DETAILDIRECT
#define DET_MOVMENT_PARA 0x07       // Body Signs Parameters //DETAILSIGN

#define RANGE_50_CM      0x01
#define RANGE_100_CM     0x02
#define RANGE_150_CM     0x03
#define RANGE_200_CM     0x04
#define RANGE_250_CM     0x05
#define RANGE_300_CM     0x06
#define RANGE_350_CM     0x07
#define RANGE_400_CM     0x08
#define RANGE_450_CM     0x09
#define RANGE_500_CM     0x0A

#define TIME_10_S       0x01
#define TIME_30_S       0x02
#define TIME_60_S       0x03
#define TIME_2_MIN      0x04
#define TIME_5_MIN      0x05
#define TIME_10_MIN     0x06
#define TIME_30_MIN     0x07
#define TIME_60_MIN     0x08

// run()
#define VERBAL          true
#define NONVERBAL       false

// Data bytes indexes
#define I_HEAD1        0  // Frame header 1
#define I_HEAD2        1
#define I_CONTROL_WORD 2  // Controll word index in data
#define I_CMD_WORD     3  // Command word index in data
#define I_LENGHT_H     4
#define I_LENGHT_L     5  // How many bytes of data
#define I_DATA         6  // Beginning of data bytes
//
#define FRAME_SIZE    32  // Max data frame size in bytes. Is it 128??

// Reset cmd
#define CMD_LEN 10       // Reset data frame length

// Mode
#define SIMPLE        0
#define ADVANCED      1

class Radar_MR24HPC1 {
 private:
    Stream *stream;     // SoftwareSerial or Serial1
    unsigned char frame[FRAME_SIZE] = {0};
    bool is_new_frame;  // New frame is ready
    uint8_t frame_len;  // Data frame size

    float calculate_distance_m(int val);
    int calculate_distance_cm(uint8_t data);
    float calculate_speed(int val);
    int calculate_time(const unsigned char hex[], int size);

    int hex_to_int(const unsigned char *hexChar);
    char hex_to_char(const unsigned char *hex);

    void run_08(bool mode = NONVERBAL);
    void run_08_cmd_0x00(bool mode = NONVERBAL);  // advandced on/off
    void run_08_cmd_0x01(bool mode = NONVERBAL);  // Sensor report
    void run_08_cmd_0x08(bool mode = NONVERBAL);  // Static energy threshold
    void run_08_cmd_0x09(bool mode = NONVERBAL);  // Motion energy threshold
    void run_08_cmd_0x80(bool mode = NONVERBAL);  // advandced on/off
    void run_08_cmd_0x81(bool mode = NONVERBAL);  // Static energy
    void run_08_cmd_0x82(bool mode = NONVERBAL);  // Motion energy
    void run_08_cmd_0x83(bool mode = NONVERBAL);  // Static distance
    void run_08_cmd_0x84(bool mode = NONVERBAL);  // Motion distance
    void run_08_cmd_0x85(bool mode = NONVERBAL);  // Motion speed
    void run_08_cmd_0x88(bool mode = NONVERBAL);  // Static energy threshold
    void run_08_cmd_0x89(bool mode = NONVERBAL);  // Motion energy threshold
    void run_08_cmd_0x0A(bool mode = NONVERBAL);  // Static trigger range
    void run_08_cmd_0x8A(bool mode = NONVERBAL);  // Static trigger range
    void run_08_cmd_0x0B(bool mode = NONVERBAL);  // Motion trigger range
    void run_08_cmd_0x8B(bool mode = NONVERBAL);  // Motion trigger range
    void run_08_cmd_0x0C(bool mode = NONVERBAL);  // Motion trigger time setting
    void run_08_cmd_0x8C(bool mode = NONVERBAL);  // Motion trigger time
    void run_08_cmd_0x0D(bool mode = NONVERBAL);  // Motion to still time setting
    void run_08_cmd_0x8D(bool mode = NONVERBAL);  // Motion to still time
    void run_08_cmd_0x0E(bool mode = NONVERBAL);  // entering no person state
    void run_08_cmd_0x8E(bool mode = NONVERBAL);  // entering no person state

    bool advanced_mode = false;

    void send_query(const unsigned char *frame, int len);  // Send to radar
    // Calculate checksum
    uint8_t calculate_sum(const unsigned char f[], int size);
    uint8_t get_frame_sum(uint8_t *frame, int len);
    bool is_frame_good(const unsigned char f[]);  // Private

    void translate_01();
    void translate_02();
    void translate_05();
    void translate_08();
    void translate_80();

    // Radar dada
    int motion_to_still_time = 0;               // advanced
    int time_for_entering_no_person_state = 0;  // advanced
    int motion_triger_time = 0;                 // advanced
    int motion_trigger_range = 0;               // advanced
    int static_trigger_range = 0;               // advanced
    int motion_energy_threshold = 0;            // advanced
    int static_energy_threshold = 0;
    int static_distance = 0;
    int motion_distance = 0;
    int motion_energy = 0;
    float motion_speed = 0;  // m/s
    int static_energy = 0;

 public:
 /*   int status_msg = 0;     // Status message
    int bodysign_val = 0;

    int static_energy = 0;     // Spatial static values
    float static_dist = 0.0;   // Distance to stationary object m
    int motion_energy = 0;
    float motion_dist = 0.0;   // Distance from the moving object m
    float motion_speed = 0.0;  // Speed of moving object m/s
*/
    Radar_MR24HPC1(Stream *s);


    void read();
    void print(int mode = HEX);

    void set_mode(int mode);
    int get_mode();

    void run(bool mode = NONVERBAL);

    void send_reset();
    void send_heartbeat();
    void ask_product_model();
    void ask_product_id();
    void ask_hardware_model();
    void ask_firmware_version();

    void set_movement_range(uint8_t range);    // simple + advanced
    void set_static_range(uint8_t range);      // simple + advanced
    void set_static_threshold(uint8_t range);  //
    void set_motion_threshold(uint8_t range);  // simple
    int set_motion_range(uint8_t range);      // advanced

    void set_absence_trigger_time(uint8_t time);  // simple

    void ask_initialization_status();
    void ask_movement_range();
    void ask_sensitivity_level();  // simple
    void ask_presence();
    void ask_motion_info();
    void ask_body_parameter();
    void ask_absence_trigger_time();
    void ask_proximity();
    // Advandced
    void ask_mode();
    void ask_static_energy();
    void ask_motion_energy();
    void ask_static_distance();
    void ask_motion_distance();
    void ask_motion_speed();
    void ask_custom_mode();
    int ask_static_threshold();
    int ask_motion_threshold();
    int ask_static_range();
    int ask_motion_range();
    int ask_motion_trigger_time();
    int ask_motion_to_still_time();
    int ask_no_person_time();

    void start_custom_mode_settings(uint8_t mode);
    void end_custom_mode_settings();



    void write_cmd(const unsigned char *buff, int len, bool cyclic = false);

    void print_hex(const unsigned char *buff, int len);
    void print_dec(const unsigned char *buff, int len);


    void translate();
};

#endif  // LIB_RADAR_MR24HPC1_SRC_RADAR_MR24HPC1_H_
