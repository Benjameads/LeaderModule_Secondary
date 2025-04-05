#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "imu_setup.h"
#include "imu_spi.h"

// Function to set up the IMU with specific configurations
//This function will set up accelerometr and gyroscope settings
void setup_imu(spi_device_handle_t spi) {
    uint8_t who_am_i = read_register(spi, 0x00);    // Read WHO_AM_I register
    ESP_LOGI(TAG, "IMU WHO_AM_I: 0x%X", who_am_i);

    if (who_am_i == 0xEA) { // Expected WHO_AM_I value
        ESP_LOGI(TAG, "IMU Detected");
        
        // Reset Device
        set_register_bits(spi, 0x06, 0x80); // Set DEVICE_RESET bit in PWR_MGMT_1 register
        while(read_register(spi, 0x06) & 0x80){ // Wait for DEVICE_RESET bit to clear
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        //Wake Device
        while(!(read_register(spi, 0x06) & 0x40)){ // Wait for SLEEP bit to turn on
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        clear_register_bits(spi, 0x06, 0x40); // Clear SLEEP bit (bit 6)
        set_register_bits(spi, 0x06, 0x01);   // Set clock to auto-select value 1-5

        // Enable Gyroscope and Accelerometer
        clear_register_bits(spi, 0x07, 0x3F); // Clear DISABLE_ACCEL and DISABLE_GYRO

        // Switch to USER_BANK_2 for gyro/accel settings
        select_user_bank(spi, 2);

        // Set Gyro sample rate to 100Hz
        clear_register_bits(spi, 0x00, 0xFF); // Clear GYRO_SMPLRT_DIV bits
        set_register_bits(spi, 0x00, 0x0A); // GYRO_SMPLRT_DIV = 10 (100Hz)

        // Set Gyroscope Scale to ±2000 dps
        set_register_bits(spi, 0x01, 0x07);  // Set GYRO_FS_SEL to 0b11 (±2000 dps) and fchoice to 1

        // Set Accel sample rate to 100Hz
        clear_register_bits(spi, 0x09, 0x07); // Clear ACCEL_SMPLRT_DIV upper bits
        clear_register_bits(spi, 0x10, 0xFF); // Clear ACCEL_SMPLRT_DIV lower bits
        set_register_bits(spi, 0x10, 0x00); // ACCEL_SMPLRT_DIV = 10 (100Hz)

        // Set Accelerometer Scale to ±8g
        clear_register_bits(spi, 0x14, 0x06);   // Clear both FS_SEL (bits 2:1)
        set_register_bits(spi, 0x14, 0x05);     // Set FS_SEL = 0b10 and enable DLPF (bit 0)

        // Switch back to USER_BANK_0
        select_user_bank(spi, 0);

        ESP_LOGI(TAG, "Accelerometer and Gyroscope setup complete");

        // Call setup_fifo and setup_magnetometer functions Optionally
        //setup_fifo(spi); // Set up FIFO buffer NOT WORKING***
        setup_magnetometer(spi); // Set up magnetometer settings
    } 
    else {
        ESP_LOGE(TAG, "IMU Not Detected");
    }
}

// Function to set up the Magnetometer settings
void setup_magnetometer(spi_device_handle_t spi) {
    // Enable I2C Master Mode
    set_register_bits(spi, 0x03, 0x30); //Enable I2C_MST_EN and I2C_IF_DIS

    // Switch to USER_BANK_3
    select_user_bank(spi, 3);

    //If we weren't using the Accelerometer and Gyroscope, we would need to set the I2C_MST_ODR_CONFIG register for the magnetometer to 100Hz, 
    //but we are using the gyro and accel so the I2C master uses the same sample rate as the gyro and accel (100Hz)

    // Configure I2C Master Settings
    clear_register_bits(spi, 0x01, 0x8F);   // Clear I2C_MST_CLK (bit 3:0) and disable multi-master mode (bit 7)
    set_register_bits(spi, 0x01, 0x17);     // Set I2C Master Clock (400 kHz) and set I2C_MST_P_NSR (bit 4)

    // Configure I2C Slave 0 for Magnetometer
    //set up I2C Master to write to magnetometer and have it go to continuous measurement mode *********************************************************/
        // Set Slave 0 I2C Address in Write Mode
        clear_register_bits(spi, 0x03, 0xFF);  // Clear I2C_SLV0_ADDR and I2C_SLV0_RNW (set to write)
        set_register_bits(spi, 0x03, 0x0C);  // Set Slave 0 Address to 0x0C (Magnetometer)

        // Slave 0 I2C Register to write (CNTL2)
        clear_register_bits(spi, 0x04, 0xFF);  // Clear I2C_SLV0_REG
        set_register_bits(spi, 0x04, 0x31);  // Set I2C slave Register adress to read/write to CNTL2 (0x31)

        // Slave 0 I2C Data to write to Control Register (CNTL2) (continuous measurement mode 0x08)
        write_register(spi, 0x06, 0x08);  // Set Magnetometer Continuous Measurement Mode (using write register because this register is used for internal I2C communication)

        // Set I2C_SLV0_CTRL register to enable read/write from Slave 0 (bit 7) and set number of bytes to read (1 byte for CNTL2 register)
        clear_register_bits(spi, 0x05, 0x0F);  // Clear I2C_SLV0_CTRL register
        set_register_bits(spi, 0x05, 0x81);  // Enable Read from Slave 0 (bit 7) and bits 3:0 set number of bytes to write (1 byte for CNTL2 register)
        vTaskDelay(pdMS_TO_TICKS(15)); // Wait for 5ms for the magnetometer to set up

        //Disable I2C_SLV0_EN (bit 7) to stop reading from Slave 0
        clear_register_bits(spi, 0x05, 0x80);  // Disable Read from Slave 0 (bit 7)
        vTaskDelay(pdMS_TO_TICKS(15)); // Wait for 5ms for the magnetometer to set up
    //**************************************************************************************************************************************************/

    //Set up Automatic read from magnetometer to EXT_SENS_DATA registers (0x3B-0x40) using I2C Slave 0 *************************************************/
        clear_register_bits(spi, 0x01, 0x20);  // Clear I2C_MST_P_NSR (bit 4) to enable burst read from Slave 0
        // this may be causing the accel dsta issue (when off) ............................................................................................
        clear_register_bits(spi, 0x03, 0xFF);  // Set I2C_SLV0_RNW (set to read)
        set_register_bits(spi, 0x03, 0x8C);  // Set Slave 0 Address to 0x0C (Magnetometer) and MSB of I2C_SLV0_RNW (bit 7) to read for Slave 0

        clear_register_bits(spi, 0x04, 0xFF);  // Clear I2C_SLV0_REG
        set_register_bits(spi, 0x04, 0x10);  // Set which I2C slave Register to read/write to Magnetometer Data register 0x10

        clear_register_bits(spi, 0x05, 0x0F);  // Clear I2C_SLV0_CTRL register
        set_register_bits(spi, 0x05, 0x89);  // Enable Read from Slave 0 (bit 7) and bits 3:0 set number of bytes to read (8 bytes for magnetometer axis data)
        // SLV0 Read will start at 0x10 (Magnetometer Data Register) and read 8 bytes (0x10-0x17) and put them in EXT_SENS_DATA registers (0x3B-0x42)
    //***************************************************************************************************************************************************/

    //Magnetometer Data Register Description:
    // 0x10: ST1 (Status Register 1)
    // 0x11: Axis X LSB
    // 0x12: Axis X MSB
    // 0x13: Axis Y LSB
    // 0x14: Axis Y MSB
    // 0x15: Axis Z LSB
    // 0x16: Axis Z MSB
    // 0x17: Dummy
    // 0x18: ST2 (Status Register 2)
    //These registers will be accessed internally by the I2C Master
    //The data will be read from the magnetometer automatically when the I2C_SLV0_EN bit is set in the I2C_SLV0_CTRL register (user bank 3 register 0x05)
    //The I2C master puts this data in the EXT_SENS_DATA registers in the order above starting at EXT_SENS_DATA_00 (user bank 0 register 0x3B)

    // Switch back to USER_BANK_0
    select_user_bank(spi, 0);
    ESP_LOGI(TAG, "Magnetometer setup complete");
}

// Function to set up the FIFO buffer 
//*************This function is set up incorrectly, it will not work properly **************//
void setup_fifo(spi_device_handle_t spi) {
    // Reset FIFO
    write_register(spi, 0x68, 0x1F);  // Assert reset
    write_register(spi, 0x68, 0x00);  // Deassert reset

    // Enable FIFO Mode (0x69)
    write_register(spi, 0x69, 0x00);  // Stream Mode (old data overwritten when full)

    // Enable FIFO for Gyro, Accel, and Temp
    write_register(spi, 0x67, 0b00011110);

    // Enable FIFO for Magnetometer
    write_register(spi, 0x66, 0b00000001);
}

