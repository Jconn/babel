#ifndef DISPLAY_MANAGER_H_
#define DISPLAY_MANAGER_H_ 



#define I2C_EXAMPLE_MASTER_SCL_IO          19               /*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO          18               /*!< gpio number for I2C master data  */
#define EEPROM_PORT             I2C_NUM_1        /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000           /*!< I2C master clock frequency */


#define PIN_SDA I2C_EXAMPLE_MASTER_SDA_IO
#define PIN_SCL I2C_EXAMPLE_MASTER_SCL_IO

void hal_print_screen(char *output);

void set_display(void);

#endif //DISPLAY_MANAGER_H_
