#include <stdint.h>
#include <Wire.h>
#include <Arduino.h>


#define PERIPHERAL_I2C_ADDRESS 0x10
#define PERIPHERAL_I2C_INSTANCE PICO_DEFAULT_I2C_INSTANCE()
#define PERIPHERAL_I2C_SDA_PIN PICO_DEFAULT_I2C_SDA_PIN
#define PERIPHERAL_I2C_SCL_PIN PICO_DEFAULT_I2C_SCL_PIN
#define PERIPHERAL_ENABLE true


typedef union FourByteArray {
    uint32_t integer;
    uint8_t bytes[sizeof(uint32_t)];
} FourByteArray;


#define TEMP_DEGREE_SCALE_FACTOR 1000.0

class TempDegree {
    public:
        TempDegree(float deg) {
            value=(uint32_t)(deg*TEMP_DEGREE_SCALE_FACTOR);
        }
        TempDegree(uint32_t raw) {
            value=raw;
        }
        FourByteArray get_trans_value() {
            return FourByteArray{value};
        }
        float get_value() {
            return (float)value/TEMP_DEGREE_SCALE_FACTOR;
        }
    
    private:
        uint32_t value;
};

#define ROT_DEGREE_SCALE_FACTOR 100.0

class RotDegree {
    public:
        RotDegree(float deg) {
            value=(uint32_t)(deg*ROT_DEGREE_SCALE_FACTOR);
        }
        RotDegree(uint32_t raw) {
            value=raw;
        }
        FourByteArray get_trans_value() {
            return FourByteArray{value};
        }
        float get_value() {
            return (float)value/ROT_DEGREE_SCALE_FACTOR;
        }
    
    private:
        uint32_t value;
};

class Peripheral {
    public:
        Peripheral(TwoWire* wire);
        void begin(bool wait);
        bool is_ready();


        void set_rot(RotDegree deg);
        RotDegree get_desired_rot();
        RotDegree get_cur_rot();

        void set_temp(TempDegree temp);
        TempDegree get_desired_temp();
        TempDegree get_cur_temp();

        void set_vac(bool status);
        bool get_vac();
        
        bool home();
        void estop();
    
    private:
        void write_to_register(uint8_t reg, size_t size, uint8_t* buffer);
        void read_from_register(uint8_t reg, size_t size, uint8_t* buffer);

        void execute(bool rot, bool temp, bool vac);

        TwoWire* wire;
};

#define STATUS_REG 0x00
#define ESTOP_REG 0xFD
#define HOME_REG 0xFE
#define EXEC_REG 0xFF

#define ROT_TARGET_00_REG 0x10
#define ROT_TARGET_01_REG 0x11
#define ROT_TARGET_02_REG 0x12
#define ROT_TARGET_03_REG 0x13
#define ROT_POS_00_REG 0x14
#define ROT_POS_01_REG 0x15
#define ROT_POS_02_REG 0x16
#define ROT_POS_03_REG 0x17

#define TEMP_TARGET_00_REG 0x20
#define TEMP_TARGET_01_REG 0x21
#define TEMP_TARGET_02_REG 0x22
#define TEMP_TARGET_03_REG 0x23
#define TEMP_POS_00_REG 0x24
#define TEMP_POS_01_REG 0x25
#define TEMP_POS_02_REG 0x26
#define TEMP_POS_03_REG 0x27

#define VAC_REG 0x30


#include <hw_config.h>


Peripheral::Peripheral(TwoWire* wire_ptr) {
  // initialize the two wire system
  wire = wire_ptr;
}

void Peripheral::begin(bool wait) {
  wire->begin();

  // if wait is enable then wait for the status register to return any non 0 value
  while (wait){
    if(is_ready()){
      break;
    }
    delay(100);
  }
}

bool Peripheral::is_ready() {
  uint8_t buffer = 0;
  read_from_register(STATUS_REG, 1, &buffer);
  return buffer!=0;
}


void Peripheral::set_rot(RotDegree deg) {
  FourByteArray trans_val = deg.get_trans_value();
  write_to_register(ROT_TARGET_00_REG, 1, &trans_val.bytes[0]);
  write_to_register(ROT_TARGET_01_REG, 1, &trans_val.bytes[1]);
  write_to_register(ROT_TARGET_02_REG, 1, &trans_val.bytes[2]);
  write_to_register(ROT_TARGET_03_REG, 1, &trans_val.bytes[3]);
  execute(true, false, false);
}

RotDegree Peripheral::get_desired_rot() {
  FourByteArray trans_val = FourByteArray{0};
  read_from_register(ROT_TARGET_00_REG, 1, &trans_val.bytes[0]);
  read_from_register(ROT_TARGET_01_REG, 1, &trans_val.bytes[1]);
  read_from_register(ROT_TARGET_02_REG, 1, &trans_val.bytes[2]);
  read_from_register(ROT_TARGET_03_REG, 1, &trans_val.bytes[3]);

  return RotDegree(trans_val.integer);
}

RotDegree Peripheral::get_cur_rot() {
  FourByteArray trans_val = FourByteArray{0};
  read_from_register(ROT_POS_00_REG, 1, &trans_val.bytes[0]);
  read_from_register(ROT_POS_01_REG, 1, &trans_val.bytes[1]);
  read_from_register(ROT_POS_02_REG, 1, &trans_val.bytes[2]);
  read_from_register(ROT_POS_03_REG, 1, &trans_val.bytes[3]);

  return RotDegree(trans_val.integer);
}

void Peripheral::set_temp(TempDegree deg) {
  FourByteArray trans_val = deg.get_trans_value();
  write_to_register(TEMP_TARGET_00_REG, 1, &trans_val.bytes[0]);
  write_to_register(TEMP_TARGET_01_REG, 1, &trans_val.bytes[1]);
  write_to_register(TEMP_TARGET_02_REG, 1, &trans_val.bytes[2]);
  write_to_register(TEMP_TARGET_03_REG, 1, &trans_val.bytes[3]);
  execute(false, true, false);
}

TempDegree Peripheral::get_desired_temp() {
  FourByteArray trans_val = FourByteArray{0};
  read_from_register(TEMP_TARGET_00_REG, 1, &trans_val.bytes[0]);
  read_from_register(TEMP_TARGET_01_REG, 1, &trans_val.bytes[1]);
  read_from_register(TEMP_TARGET_02_REG, 1, &trans_val.bytes[2]);
  read_from_register(TEMP_TARGET_03_REG, 1, &trans_val.bytes[3]);

  return TempDegree(trans_val.integer);
}

TempDegree Peripheral::get_cur_temp() {
  FourByteArray trans_val = FourByteArray{0};
  read_from_register(TEMP_POS_00_REG, 1, &trans_val.bytes[0]);
  read_from_register(TEMP_POS_01_REG, 1, &trans_val.bytes[1]);
  read_from_register(TEMP_POS_02_REG, 1, &trans_val.bytes[2]);
  read_from_register(TEMP_POS_03_REG, 1, &trans_val.bytes[3]);

  return TempDegree(trans_val.integer);
}


void Peripheral::set_vac(bool status) {
  write_to_register(VAC_REG, 1, (uint8_t*)&status);
  execute(false, false, true);
}

bool Peripheral::get_vac() {
  bool status;
  read_from_register(VAC_REG, 1, (uint8_t*)&status);
  return status;
}

bool Peripheral::home() {
  write_to_register(HOME_REG, 0, NULL);
  return true;
}
void Peripheral::estop() {
  write_to_register(ESTOP_REG, 0, NULL);
}

void Peripheral::execute(bool rot, bool temp, bool vac) {
  uint8_t base = 0x00;
  if(rot){
    base+=1;
  }
  if(temp){
    base+=2;
  }
  if(vac){
    base+=4;
  }
  write_to_register(EXEC_REG, 1, &base);
}


// TwoWire Helpers

void Peripheral::write_to_register(uint8_t reg, size_t size, uint8_t* buffer){
    wire->beginTransmission(PERIPHERAL_I2C_ADDRESS);
    wire->write(reg);
    for(size_t i=0;i<size;i++){
      wire->endTransmission(false);
      wire->write(buffer[i]);
    }
    wire->endTransmission(true);
}

void Peripheral::read_from_register(uint8_t reg, size_t size, uint8_t* buffer){
    wire->beginTransmission(PERIPHERAL_I2C_ADDRESS);
    wire->write(reg);
    wire->endTransmission(false);
    wire->requestFrom(PERIPHERAL_I2C_ADDRESS, (uint8_t)size);
    for(size_t i=0;i<size;i++){
      buffer[i] = wire->read();
    }
    wire->endTransmission(true);
}



Peripheral* peripheral = NULL;

void setup() {
   TwoWire* two_wire = new TwoWire(PERIPHERAL_I2C_INSTANCE, PERIPHERAL_I2C_SDA_PIN, PERIPHERAL_I2C_SCL_PIN);
  peripheral = new Peripheral(two_wire);

  // ensure peripheral is connected
  peripheral->begin(PERIPHERAL_ENABLE);

  Serial.begin(9600);
  while(!Serial);
}

void loop() {
  Serial.println("Beginning loop");
  for (int i =0;i<5;i++){
    float desired_rot = (float)90.0*i;
    Serial.print("Attempting to rotate to ");
    Serial.println(desired_rot);
    peripheral->set_rot(RotDegree(desired_rot));
    while(true) {
        RotDegree cur_pos = peripheral->get_cur_rot();
        Serial.print("Current pos ");
        Serial.println(cur_pos.get_value());
        delay(1000);
    }
  }

}
