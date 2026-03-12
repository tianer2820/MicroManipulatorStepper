#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "i2c_parser.h"




/**
 * I2C Communication
 */
TwoWire i2c_inst(I2C_SDA, I2C_SCL);

uint8_t i2c_cursor = 0;

uint8_t i2c_virtual_reg[256] = {0};
uint8_t i2c_virtual_reg_cursor = 0;

enum I2CStatus {
    MODULE_UNINITIALIZED = 0,
    MODULE_HOMING = 1,
    MODULE_READY = 2,
    MODULE_EMERGENCY_STOPPED = 3,
    MODULE_ERROR = 4,
};


static struct i2c_parser_callbacks parser_callbacks;



/**
 * I2C protocol:
 * 
 * The first byte written is the cursor address.
 * Any bytes after will write the register in increasing order.
 * Example: 01 0A 0B 0C
 * will write 0A to address 1, 0B to 2, 0C to 3.
 * The cursor now stays at address 4.
 * 
 * Write one byte to place the cursor before read,
 * same increasing order applies for reading.
 */
static void on_i2c_receive(int num_bytes){
    int i = 0;
    while (i2c_inst.available()) {
        char c = i2c_inst.read();
        if (i == 0) {
            // first byte is cursor address
            i2c_virtual_reg_cursor = c;
        } else {
            // write to virtual register
            i2c_virtual_reg[i2c_virtual_reg_cursor] = c;
            i2c_virtual_reg_cursor += 1;
        }

        // add special checks here
        // such as trigger registers



        i += 1;
    }
}

static void on_i2c_request(){
    i2c_inst.write(i2c_virtual_reg + i2c_virtual_reg_cursor, 256 - i2c_virtual_reg_cursor);
}


void i2c_parser_init(i2c_parser_callbacks callbacks){
    i2c_inst.begin(10);
    parser_callbacks = callbacks;
    i2c_inst.onReceive(on_i2c_receive);
    i2c_inst.onRequest(on_i2c_request);
}


void i2c_parser_poll(void){
    
}