/*
Copyright 2023 Tauno Erik
*/

#ifndef LIB_RADAR_MR24HPC1_SRC_RADAR_MR24HPC1_H_
#define LIB_RADAR_MR24HPC1_SRC_RADAR_MR24HPC1_H_

// Frame Headers
#define HEAD1          0x53  // Frame header 1
#define HEAD2          0x59  // Frame header 2
// End of frame
#define END1           0x54  // End of data frame 1
#define END2           0x43  // End of data frame 2

// Presense reporting data
#define UNOCCUPIED     0x00
#define OCCUPIED       0x01

// Motion reporting data
#define NONE           0x00
#define STATIC         0x01
#define ACTIVE         0x02
#define MOTION         0x02

// Proximity reporting data
#define APPROACHING    0x01  // near
#define RECEDING       0x02  // far

#define RANGE_50_CM    0x01
#define RANGE_100_CM   0x02
#define RANGE_150_CM   0x03
#define RANGE_200_CM   0x04
#define RANGE_250_CM   0x05
#define RANGE_300_CM   0x06
#define RANGE_350_CM   0x07
#define RANGE_400_CM   0x08
#define RANGE_450_CM   0x09
#define RANGE_500_CM   0x0A

#define TIME_10_S      0x01
#define TIME_30_S      0x02
#define TIME_60_S      0x03
#define TIME_2_MIN     0x04
#define TIME_5_MIN     0x05
#define TIME_10_MIN    0x06
#define TIME_30_MIN    0x07
#define TIME_60_MIN    0x08

// run()
#define VERBAL         true
#define NONVERBAL      false

// Data bytes indexes
#define I_HEAD1        0  // Frame header 1
#define I_HEAD2        1
#define I_CONTROL_WORD 2  // Controll word index in data
#define I_CMD_WORD     3  // Command word index in data
#define I_LENGHT_H     4
#define I_LENGHT_L     5  // How many bytes of data
#define I_DATA         6  // Beginning of data bytes
// Mode
#define SIMPLE         0
#define ADVANCED       1
//
#define FRAME_SIZE    32  // Max data frame size in bytes. Is it 128??

class Radar_MR24HPC1 {
 private:
    Stream *stream;     // SoftwareSerial or Serial1
    unsigned char frame[FRAME_SIZE] = {0};
    bool is_new_frame;  // New frame is ready
    uint8_t frame_len;  // Data frame size

    float calculate_distance_m(int val);
    int   calculate_distance_cm(int data);
    float calculate_speed(int val);
    int   calculate_time(const unsigned char hex[], int size);

    void send_query(const unsigned char *frame, int len);  // Send to radar
    // Calculate checksum
    uint8_t calculate_sum(const unsigned char f[], int size);
    uint8_t get_frame_sum(uint8_t *frame, int len);
    bool is_frame_good(const unsigned char f[]);

    int hex_to_int(const unsigned char *hexChar);
    char hex_to_char(const unsigned char *hex);
    void print_hex(const unsigned char *buff, int len);
    void print_dec(const unsigned char *buff, int len);

    // Responses
    void run_01(bool mode = NONVERBAL);  // Heartbeat
    void run_02(bool mode = NONVERBAL);
    void run_05(bool mode = NONVERBAL);
    void run_05_cmd_0x01(bool mode = NONVERBAL);  // Init completed
    void run_05_cmd_0x07(bool mode = NONVERBAL);  // Motion limit respomse
    void run_05_cmd_0x08(bool mode = NONVERBAL);  // Static limit response
    void run_05_cmd_0x09(bool mode = NONVERBAL);  // Custom mode setting
    void run_05_cmd_0x81(bool mode = NONVERBAL);  // Initialization status inquiry response
    // void run_05_cmd_0x84(bool mode = NONVERBAL);  // TODO: tegelikult 08_0x84 Motion distance response
    void run_05_cmd_0x85(bool mode = NONVERBAL);  // Motion speed response
    void run_05_cmd_0x87(bool mode = NONVERBAL);  // Scene settings
    void run_05_cmd_0x88(bool mode = NONVERBAL);  // Sensitivity settings
    void run_05_cmd_0x89(bool mode = NONVERBAL);  // Custom mode inquiry response
    void run_05_cmd_0x0A(bool mode = NONVERBAL);  // End of custom mode setting
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
    void run_08_cmd_0x88(bool mode = NONVERBAL);  // Static energy threshold
    void run_08_cmd_0x89(bool mode = NONVERBAL);  // Motion energy threshold
    void run_08_cmd_0x0A(bool mode = NONVERBAL);  // Static trigger limit
    void run_08_cmd_0x8A(bool mode = NONVERBAL);  // Static trigger limit
    void run_08_cmd_0x0B(bool mode = NONVERBAL);  // Motion trigger limit
    void run_08_cmd_0x8B(bool mode = NONVERBAL);  // Motion trigger limit
    void run_08_cmd_0x0C(bool mode = NONVERBAL);  // Motion trigger time setting
    void run_08_cmd_0x8C(bool mode = NONVERBAL);  // Motion trigger time
    void run_08_cmd_0x0D(bool mode = NONVERBAL);  // Motion to still time setting
    void run_08_cmd_0x8D(bool mode = NONVERBAL);  // Motion to still time
    void run_08_cmd_0x0E(bool mode = NONVERBAL);  // entering no person state
    void run_08_cmd_0x8E(bool mode = NONVERBAL);  // entering no person state
    void run_80(bool mode = NONVERBAL);
    void run_80_cmd_0x01(bool mode = NONVERBAL);  // Presence report
    void run_80_cmd_0x02(bool mode = NONVERBAL);  // Motion report
    void run_80_cmd_0x03(bool mode = NONVERBAL);  // Body parameter report
    void run_80_cmd_0x81(bool mode = NONVERBAL);  // Presence response
    void run_80_cmd_0x82(bool mode = NONVERBAL);  // Motion response
    void run_80_cmd_0x83(bool mode = NONVERBAL);  // Body parameter
    void run_80_cmd_0x0A(bool mode = NONVERBAL);  // setting time for no person state
    void run_80_cmd_0x0B(bool mode = NONVERBAL);  // proximity setting
    void run_80_cmd_0x8A(bool mode = NONVERBAL);  // setting time for no person state inquiry
    void run_80_cmd_0x8B(bool mode = NONVERBAL);  // proximity inquiry

    // Radar dada
    int mode = ADVANCED;  // 0 simple, 1 advanced
    int heartbeat = NONE;

    int motion_to_static_time = 0;               // advanced
    int time_for_entering_no_person_state = 0;  // advanced absence triger time
    int motion_trigger_time = 0;                 // advanced

    int motion_trigger_limit = 0;               // advanced
    int static_trigger_limit = 0;               // advanced

    int motion_energy_threshold = 0;            // advanced
    int static_energy_threshold = 0;

    int static_distance = 0;
    int motion_distance = 0;

    int motion_energy = 0;
    int static_energy = 0;

    float motion_speed = 0;         // m/s

    int custom_mode = 0;            // 0x01 to 0x04
    int initialization_status = 0;  // 0x01 or 0x02

    // Simple mode
    int presence = UNOCCUPIED;        // 0x00 or 0x01
    int motion = NONE;                // none, static, active
    int activity = NONE;              // body_parameter 0-100%
    int direction = NONE;             // APPROACHING, RECEDING

 public:
    Radar_MR24HPC1(Stream *s);

    void set_mode(int mode);          // Simple or Advanced
    void ask_mode();

    void read();                      // Read dada frame
    void print(int mode = HEX);       // Print frame

    void run(bool mode = NONVERBAL);  // process frames

    void reset();                     // x
    void ask_heartbeat();             // x
    void ask_product_model();
    void ask_product_id();
    void ask_hardware_model();
    void ask_firmware_version();

    void ask_initialization_status();  // x
    void ask_custom_mode();

    void ask_presence();               // x Occupation

    void ask_activity();               // x
    void ask_absence_trigger_time();   // x
    void ask_direction();              // x APPROACHING RECEDING

    void ask_motion_energy();          // x
    void ask_motion_body_distance();   // x
    void ask_motion_limit();           // x advanced and simple
    void ask_motion_speed();           // x

    void ask_motion();                 // x none, static, active
    void ask_motion_trigger_time();    // x
    void ask_motion_to_static_time();  // x
    void ask_no_person_time();         // x

    void ask_static_energy();          // x
    void ask_static_body_distance();   // x
    void ask_static_limit();           // x

    void set_motion_limit(uint8_t limit);      // x  simple + advanced
    void set_static_limit(uint8_t limit);      // x simple + advanced
    void set_static_threshold(uint8_t limit);  //
    void set_motion_threshold(uint8_t limit);  // 0-250

    void set_absence_trigger_time(int time_ms);  // simple

    void start_custom_mode_settings(uint8_t mode);
    void end_custom_mode_settings();

// ----------------------//
    int get_mode();                  // return radar mode
    int get_heartbeat();             // returns heartbeat counter value

    // Works only in SIMPLE mode:
    int get_motion();
    int get_activity();  // body parameter
    int get_direction();
    int get_presence();

    int   get_motion_energy();
    float get_motion_speed();
    int   get_motion_distance();
    int   get_static_energy();
    int   get_static_distance();
    int   get_initialization_status();
    int   get_time_for_entering_no_person_state();
    int   get_motion_trigger_time();
    int   get_motion_to_static_time();
    int   get_static_trigger_limit();

};

#endif  // LIB_RADAR_MR24HPC1_SRC_RADAR_MR24HPC1_H_
