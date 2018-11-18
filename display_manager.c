#include "display_manager.h"
#include "esp_log.h"

bool u8g2_setup = false;
static u8g2_t u8g2; // a structure which will contain all the data for one display

const char* TAG  = "kolban_ex";

void set_display(void)
{
    //setup the hal state for the display callback(this is kolban specific)
    u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
    u8g2_esp32_hal.sda   = PIN_SDA;
    u8g2_esp32_hal.scl  = PIN_SCL;
    u8g2_esp32_hal_init(u8g2_esp32_hal);

    u8g2_Setup_ssd1306_i2c_128x64_noname_f(
            &u8g2,
            U8G2_R0,
            u8g2_esp32_i2c_byte_cb,
            u8g2_esp32_gpio_and_delay_cb);  // init u8g2 structure

//either    “0111100” or “0111101”, 
    u8x8_SetI2CAddress(&(u8g2.u8x8),0x3C << 1);
    if(i2c_error()) goto i2c_err;
    u8x8_SetI2CAddress(&(u8g2.u8x8),0x3C << 1);
    if(i2c_error()) goto i2c_err;
    u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,
    if(i2c_error()) goto i2c_err;
    u8g2_SetPowerSave(&u8g2, 0); // wake up display
    if(i2c_error()) goto i2c_err;
    ESP_LOGI(TAG, "u8g2_ClearBuffer");
    u8g2_ClearBuffer(&u8g2);
    if(i2c_error()) goto i2c_err;
    u8g2_setup = true;
    return;
i2c_err:
    //explicitly don't allow setup if we have an i2c error
    
    ESP_LOGE(TAG, "I2C SETUP FAILED - no display");
    u8g2_setup = false;
}



void hal_print_screen(char *output)
{
    if(!u8g2_setup)
    {
        return;
    }

    u8g2_ClearBuffer(&u8g2);
    ESP_LOGI("display", "printing output: %s", output); 
    u8g2_SetFont(&u8g2, u8g2_font_ncenB14_tr);
    u8g2_DrawStr(&u8g2, 16,16, output);
    u8g2_SendBuffer(&u8g2);
}