/**********************************************************************
* - Description:		esp32-mpu6050
* - File:				i2c_driver.c
* - Compiler:			xtensa-esp32
* - Debugger:			USB2USART
* - Author:				Mohamed El-Sabagh
* - Target:				ESP32
* - Created:			2017-12-11
* - Last changed:		2017-12-11
*
**********************************************************************/

#include "i2c_driver.h"

#define I2C_MASTER_NUM 					I2C_NUM_1   /*!< I2C port number for master dev */
#define I2C_MASTER_SCL_IO    			19    		/*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO    			18    		/*!< gpio number for I2C master data  */
#define I2C_MASTER_FREQ_HZ    			100000     	/*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   	0   		/*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   	0   		/*!< I2C master do not need buffer */
#define WRITE_BIT  						I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT   						I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN   					0x1     /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS  					0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL    						0x0         /*!< I2C ack value */
#define NACK_VAL   						0x1         /*!< I2C nack value */

/**
  * @brief  Initializes peripherals used by the I2C driver.
  * @param  None
  * @retval None
  */
void vI2CInit()
{
	int i2c_master_port = I2C_MASTER_NUM;
	    i2c_config_t conf;
	    conf.mode = I2C_MODE_MASTER;
	    conf.sda_io_num = I2C_MASTER_SDA_IO;
	    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	    conf.scl_io_num = I2C_MASTER_SCL_IO;
	    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
	    i2c_param_config(i2c_master_port, &conf);
	    i2c_driver_install(i2c_master_port, conf.mode,
	                       I2C_MASTER_RX_BUF_DISABLE,
	                       I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief test code to read esp-i2c-slave
 *        We need to fill the buffer of esp slave device, then master can read them out.
 *
 * _______________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------------|--------------------|------|
 *
 */
esp_err_t sI2cMasterReadSlave(uint8_t* data_rd, size_t size, uint8_t slave_address)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( slave_address << 1 ) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Test code to write esp-i2c-slave
 *        Master device write data to slave(both esp32),
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 *
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------------|------|
 *
 */
esp_err_t sI2cMasterWriteSlave(uint8_t* data_wr, size_t size, uint8_t slave_address)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( slave_address << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
