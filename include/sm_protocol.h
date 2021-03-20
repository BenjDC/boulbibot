#include <unistd.h>
#include "port_handler_linux.h"
#include "port_handler.h"
#include "sm_registers.h"


// binary tool macro
#define LOW_BYTE(x) ((unsigned char)((x)&0xFF))
#define HIGH_BYTE(x) ((unsigned char)(((x) >> 8) & 0xFF))
#define MAKE_SHORT(l, h) (((uint16_t)((h)&0xFF) << 8) | (uint16_t)((l)&0xFF))
#define SIGN(x) (((x)&0x8000) ? ((x)-0x10000) : (x))



class sm_protocol
{

public:
  sm_protocol(uint32_t baud) : verbose(0),
                               rx_packet_position(0),
                               rx_packet_payload_length(0),
                               tx_packet_length(0),
                               rx_packet_id(0),
                               rx_packet_instruction(0),
                               rx_packet_error(0),
                               rx_packet_crc(0),
                               rx_timeout_us(10000),
                               MySerial(smarter_motor::PortHandler::getPortHandler("/dev/ttyUSB0"))
  {
    // MEGA 16MHz:
    //  baud = 115200 OK 1.4ms reply
    //  baud = 250000 OK 0.7ms reply
    //  baud = 500000 OK 440us reply
    //  baud = 1000000 OK 430us reply
    // M5Stack:
    //  baud = 500000 OK
    //  baud = 1000000 OK
    MySerial->setBaudRate(baud);
  } // 10ms

  int verbose;

  struct sync_byte_data
  {
    uint8_t id;
    uint8_t length;
    uint8_t data[16];
    // helpers
    sync_byte_data() : id(0), length(0) {}
    sync_byte_data(uint8_t const &i) : id(i), length(0) {}
    void add_param_u8(uint8_t const &param) { data[length++] = param; }
    void add_param_f(float const &param) { data[length++] = (uint8_t)(param); }
  };

  struct sync_word_data
  {
    uint8_t id;
    uint8_t length;
    uint16_t wdata[8];
    // helpers
    sync_word_data() : id(0), length(0) {}
    sync_word_data(uint8_t const &i) : id(i), length(0) {}
    void add_param_u16(uint16_t const &param) { wdata[length++] = param; }
    void add_param_f(float const &param) { wdata[length++] = (uint16_t)(param); }
  };

  // Low lew commands
  int pingCommand(uint8_t id, uint16_t *model_number, uint8_t *firmware_version);
  int readByteCommand(uint8_t id, uint16_t address, uint16_t length, uint8_t *data);
  int readWordCommand(uint8_t id, uint16_t address, uint16_t length, uint16_t *data);
  int writeByteCommand(uint8_t id, uint16_t address, uint16_t length, uint8_t const *data);
  int writeWordCommand(uint8_t id, uint16_t address, uint16_t length, uint16_t const *data);
  void syncWriteByteCommand(uint16_t address, uint8_t number_of_ids, sync_byte_data const *data);
  int syncWriteWordCommand(uint16_t address, uint8_t number_of_ids, sync_word_data const *data);
  int factoryResetCommand(uint8_t id);
  int rebootCommand(uint8_t id);

  // Utility SMS commands
  int enableTorque(uint8_t id);
  int disableTorque(uint8_t id);
  int enableLed(uint8_t id);  
  int disableLed(uint8_t id);
  int setMode(uint8_t id, uint8_t mode);
  int setPosition(uint8_t id, float angle_deg);
  int setVelocityLimit(uint8_t id, float velocity_dps);
  int setPositionVelocity(uint8_t id, float angle_deg, float velocity_dps);
  int setCurrentLimit(uint8_t id, uint16_t current_mA);
  int setPWMLimit(uint8_t id, uint8_t pwm_pc);
  int setPositionVelocityCurrentPWM(uint8_t id, float angle_deg, float velocity_dps, uint16_t current_mA, uint8_t pwm_pc);
  float getPosition(uint8_t id, int * error);
  float getVelocity(uint8_t id, int * error);
  float getCurrent(uint8_t id, int * error);

private:
  uint8_t tx_packet_buffer[TX_BUFFER_SIZE];
  uint8_t rx_packet_buffer[RX_BUFFER_SIZE];

  uint32_t rx_packet_position;
  uint16_t rx_packet_payload_length;
  uint32_t tx_packet_length;

  uint8_t rx_packet_id;
  uint8_t rx_packet_instruction;
  uint8_t rx_packet_error;
  uint16_t rx_packet_crc;
  unsigned long rx_timeout_us;
  smarter_motor::PortHandler *MySerial;

  uint16_t updateCRC(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size);
  void serialFlush();
  void completePacket(uint8_t id, uint16_t payload_length); // length from instruction byte to last parameter byte
  int receiveStatusPacket(uint8_t id, unsigned long timeout_us);
  bool decodePacket(int b);
  unsigned long micros();
};
