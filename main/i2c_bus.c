#include "i2c_bus.h"

#define I2C_SDA_GPIO GPIO_NUM_33
#define I2C_SCL_GPIO GPIO_NUM_32
#define I2C_GLITCH_IGNORE_CYCLES 7

esp_err_t i2c_bus_init(i2c_master_bus_handle_t *bus)
{
    const i2c_master_bus_config_t cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = I2C_GLITCH_IGNORE_CYCLES,
        .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&cfg, bus);
}
