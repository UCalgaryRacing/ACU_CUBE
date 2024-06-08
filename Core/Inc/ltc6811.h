#ifndef LTC6811_H
#define LTC6811_H

#include <stdint.h>
#include <stdbool.h>

// Struct containing LTC6811 configuration settings.
// This is used to persistently store configuration settings for when the configuration register has to be written to toggle discharging.
typedef struct ltc6811_config {
    uint8_t gpio_pulldowns;
    bool refon;
    bool adcopt;
    uint16_t vuv;
    uint16_t vov;
    uint8_t dcto;
    uint16_t dcc;
}ltc6811_config;


// Struct containing information unique to each LTC6811 on the bus.
typedef struct ltc6811 {
    uint8_t address;
    uint8_t cell_count;
    uint8_t cva_reg[6];
    uint8_t cvb_reg[6];
    uint8_t cvc_reg[6];
    uint8_t cvd_reg[6];
    uint8_t auxa_reg[6];
    uint8_t auxb_reg[6];
    uint16_t dcc;
}ltc6811;


// If defined, LTC6811 functions will always perform a standby wake before communication.
// If LTC6811 core has gone to sleep, user must still call wake_sleep() manually.
#define ALWAYS_STANDBY_WAKE


/**
 * @brief Initialize memory structures for LTC6811 functions.
 *
 * This function must be called in the same or higher context before any LTC6811 communication functions are called.
 */
void init_LTC6811(void);


/**
 * @brief Wake LTC6811 isoSPI port while LTC6811 core is in SLEEP state.
 *
 * Typically the LTC6811 core will be in SLEEP state after there has been no communication for 2 seconds.
 *
 * See LTC6811 datasheet Figures 1 and 26.
 *
 * @param ltc6820 Struct containing spi_dt_spec and gpio_dt_spec representing SPI and CS connections to LTC6820.
 */
void wake_sleep();


/**
 * @brief Wake LTC6811 isoSPI port while LTC6811 core is in STANDBY state.
 *
 * The LTC6811 isoSPI port must be woken if there has been no communication for 5.5 milliseconds.
 *
 * See LTC6811 datasheet Figures 1 and 26.
 *
 * @note If there has been no communication for 2 seconds, wake_sleep() should be called instead.
 *
 * @note If ALWAYS_STANDBY_WAKE is defined, all LTC6811 communication functions will automatically call this function.
 *
 * @param ltc6820 Struct containing spi_dt_spec and gpio_dt_spec representing SPI and CS connections to LTC6820.
 */
void wake_standby();


/**
 * @brief Broadcast a command to every LTC6811 on the bus.
 *
 * See LTC6811 datasheet Table 31 (Broadcast/Address Poll Command).
 *
 * @note This function will NOT poll for command completion.
 *
 * @param ltc6820 Struct containing spi_dt_spec and gpio_dt_spec representing SPI and CS connections to LTC6820.
 * @param command_code 11-bit code representing the command to be issued.
 */
void broadcast_command(uint16_t command_code);


/**
 * @brief Write to a register on every LTC6811 on the bus.
 *
 * See LTC6811 datasheet Table 32 (Broadcast Write Command).
 *
 * @note This function will not send daisy-chain shift bytes after the command.
 *
 * @param ltc6820 Struct containing spi_dt_spec and gpio_dt_spec representing SPI and CS connections to LTC6820.
 * @param command_code 11-bit code representing the command to be issued.
 * @param tx_reg 6-byte array containing the data to be written to the register.
 */
void broadcast_write(uint16_t command_code, uint8_t *tx_reg);


/**
 * @brief Write to a register on a specific LTC6811.
 *
 * See LTC6811 datasheet Table 33 (Address Write Command).
 *
 * @param ltc6820 Struct containing spi_dt_spec and gpio_dt_spec representing SPI and CS connections to LTC6820.
 * @param address 4-bit address of the LTC6811.
 * @param command_code 11-bit code representing the command to be issued.
 * @param tx_reg 6-byte array containing the data to be written to the register.
 */
void address_write(uint8_t address, uint16_t command_code, uint8_t *tx_reg);


/**
 * @brief Read from a register on a specific LTC6811.
 *
 * See LTC6811 datasheet Table 35 (Address Read Command).
 *
 * @param ltc6820 Struct containing spi_dt_spec and gpio_dt_spec representing SPI and CS connections to LTC6820.
 * @param address 4-bit address of the LTC6811.
 * @param command_code 11-bit code representing the command to be issued.
 * @param rx_reg 6-byte array to be filled with the data read from the register.
 */
void address_read(uint8_t address, uint16_t command_code, uint8_t *rx_reg);


/**
 * @brief Updates configuration registers of every LTC6811 on the bus.
 *
 * Used to update configuration settings from persistent configuration struct.
 *
 * @note All discharging will be disabled since DCC bits will be wiped.
 *
 * @param ltc6820 Struct containing spi_dt_spec and gpio_dt_spec representing SPI and CS connections to LTC6820.
 * @param config Struct containing persistent LTC6811 configuration settings (see notes on struct ltc6811_config).
 */
void update_config(ltc6811_config *config);


/**
 * @brief Extract floating-point cell voltages from cell voltage register data.
 *
 * See LTC6811 datasheet Table 52 (Memory Bit Descriptions) -> CxV.
 *
 * @note If cell voltage registers are filled with 0xFF, this function will attempt to fill the output array with corresponding NaN.
 *
 * @param voltage_reg 6-byte array containing data from the cell voltage register.
 * @param voltages 3-index array to be filled with floating-point cell voltages.
 */
void extract_voltage_reg(uint8_t *voltage_reg, float *voltages);


/**
 * @brief Extract floating-point cell voltages from all LTC6811s in the provided array of LTC6811 structs.
 *
 * @note cell_voltage array size must be the sum of all cell_count variable within each LTC6811 struct.
 *
 * @param ltc6811 array of LTC6811 structs.
 * @param cell_voltage floating point array to hold every cell voltage.
 * @param slave_num number of slaves in LTC6811 struct array
 */
void extract_all_voltages(ltc6811 *ltc6811_arr, float *cell_voltage, int slave_num);


/**
 * @brief Read voltages from all LTC6811s in the provided array of LTC6811 structs.
 *
 * @note The LTC6811 structs contain both the addresses of physical LTC6811s to be read from and also the register buffers to be filled.
 *
 * @param ltc6820 Struct containing spi_dt_spec and gpio_dt_spec representing SPI and CS connections to LTC6820.
 * @param ltc6811 array of LTC6811 structs.
 * @param slave_num number of slaves in LTC6811 struct array
 */
void read_all_voltages(ltc6811 *ltc6811_arr, int slave_num);

void generate_i2c(uint8_t * comm_reg, uint8_t *comm_data, uint8_t len);

void send_comm(uint8_t * i2c_message, uint8_t len, int mux_num);

void broadcast_command_stcomm(uint16_t command_code);

double calc_temp(double adc_value);

int read_all_temps(ltc6811 *ltc6811_arr, float *thermistor_temps, uint8_t mux_channels, int slave_num);




// Command Codes
// (see Table 38 in LTC6811 datasheet)
#define WRCFGA      (0b00000000001) // Write Configuration Register Group A
#define RDCFGA      (0b00000000010) // Read Configuration Register Group A
#define RDCVA       (0b00000000100) // Read Cell Voltage Register Group A
#define RDCVB       (0b00000000110) // Read Cell Voltage Register Group B
#define RDCVC       (0b00000001000) // Read Cell Voltage Register Group C
#define RDCVD       (0b00000001010) // Read Cell Voltage Register Group D
#define RDAUXA      (0b00000001100) // Read Auxiliary Register Group A
#define RDAUXB      (0b00000001110) // Read Auxiliary Register Group B
#define RDSTATA     (0b00000010000) // Read Status Register Group A
#define RDSTATB     (0b00000010010) // Read Status Register Group B
#define WRSCTRL     (0b00000010100) // Write S Control Register Group
#define RDSCTRL     (0b00000010110) // Read S Control Register Group
#define WRPWM       (0b00000100000) // Write PWM Register Group
#define RDPWM       (0b00000100010) // Read PWM Register Group
#define STSCTRL     (0b00000011001) // Start S Control Pulsing and Poll Status
#define CLRSCTRL    (0b00000011000) // Clear S Control Register Group
// Start Cell Voltage ADC Conversion and Poll Status
#define ADCV(MD, DCP, CH)       (0b01001100000 | ((MD) << 7) | ((DCP) << 4) | (CH))
// Start Open Wire ADC Conversion and Poll Status
#define ADOW(MD, PUP, DCP, CH)  (0b01000101000 | ((MD) << 7) | ((PUP) << 6) | ((DCP) << 4) | (CH))
// Start Self Test Cell Voltage Conversion and Poll Status
#define CVST(MD, ST)            (0b01000000111 | ((MD) << 7) | ((ST << 5)))
// Start Overlap Measurement of Cell 7 Voltage
#define ADOL(MD, DCP)           (0b01000000001 | ((MD) << 7) | ((DCP) << 4))
// Start GPIOs ADC Conversion and Poll Status
#define ADAX(MD, CHG)           (0b10001100000 | ((MD) << 7) | (CHG))
// Start GPIOs ADC Conversion With Digital Redundancy and Poll Status
#define ADAXD(MD, CHG)          (0b10000000000 | ((MD) << 7) | (CHG))
// Start Self Test GPIOs Conversion and Poll Status
#define AXST(MD, ST)            (0b10000000111 | ((MD) << 7) | ((ST << 5)))
// Start Status Group ADC Conversion and Poll Status
#define ADSTAT(MD, CHST)        (0b10001101000 | ((MD) << 7) | (CHST))
// Start Status Group ADC Conversion With Digital Redundancy and Poll Status
#define ADSTATD(MD, CHST)       (0b10000001000 | ((MD) << 7) | (CHST))
// Start Self Test Status Group Conversion and Poll Status
#define STATST(MD, ST)          (0b10000001111 | ((MD) << 7) | ((ST << 5)))
// Start Combined Cell Voltage and GPIO1, GPIO2 Conversion and Poll Status
#define ADCVAX(MD, DCP)         (0b10001101111 | ((MD) << 7) | ((DCP) << 4))
// Start Combined Cell Voltage and Sum of All Cells Conversion and Poll Status
#define ADCVSC(MD, DCP)         (0b10001100111 | ((MD) << 7) | ((DCP) << 4))
#define CLRCELL     (0b11100010001) // Clear Cell Voltage Register Groups
#define CLRAUX      (0b11100010010) // Clear Auxiliary Register Groups
#define CLRSTAT     (0b11100010011) // Clear Status Register Groups
#define PLADC       (0b11100010100) // Poll ADC Conversion Status
#define DIAGN       (0b11100010101) // Diagnose MUX and Poll Status
#define WRCOMM      (0b11100100001) // Write COMM Register Group
#define RDCOMM      (0b11100100010) // Read COMM Register Group
#define STCOMM      (0b11100100011) // Start I2C/SPI Communication


// Command Bits (used with Command Codes above that take arguments)
// (see Table 39 in LTC6811 datasheet)
#define MD_422_1k           (0b00)  // 422Hz if ADCOPT = 0, 1kHz if ADCOPT = 1
#define MD_27k_14k          (0b01)  // 27kHz if ADCOPT = 0, 14kHz if ADCOPT = 1
#define MD_7k_3k            (0b10)  // 7kHz if ADCOPT = 0, 3kHz if ADCOPT = 1
#define MD_26_2k            (0b11)  // 26Hz if ADCOPT = 0, 2kHz if ADCOPT = 1
#define DCP_NOT_PERMITTED   (0b0)   // Discharge not permitted during cell measurement
#define DCP_PERMITTED       (0b1)   // Discharge permitted during cell measurement
#define CH_ALL              (0b000) // Measure all cells
#define CH_CELLS_1_7        (0b001) // Measure Cells 1 and 7
#define CH_CELLS_2_8        (0b010) // Measure cells 2 and 8
#define CH_CELLS_3_9        (0b011) // Measure cells 3 and 9
#define CH_CELLS_4_10       (0b100) // Measure cells 4 and 10
#define CH_CELLS_5_11       (0b101) // Measure cells 5 and 11
#define CH_CELLS_6_12       (0b110) // Measure cells 6 and 12
#define PUP_DOWN            (0b0)   // Pull down for open wire check
#define PUP_UP              (0b1)   // Pull up for open wire check
#define ST_1                (0b01)  // Self Test 1 (see datasheet for values)
#define ST_2                (0b10)  // Self Test 2 (see datasheet for values)
#define CHG_ALL             (0b000) // Measure all GPIOs and 2nd reference
#define CHG_GPIO_1          (0b001) // Measure GPIO 1
#define CHG_GPIO_2          (0b010) // Measure GPIO 2
#define CHG_GPIO_3          (0b011) // Measure GPIO 3
#define CHG_GPIO_4          (0b100) // Measure GPIO 4
#define CHG_GPIO_5          (0b101) // Measure GPIO 5
#define CHG_2ND_REF         (0b110) // Measure 2nd reference
#define CHST_ALL            (0b000) // Measure SC, ITMP, VA, VD
#define CHST_SC             (0b001) // Measure SC (Sum of All Cells)
#define CHST_ITMP           (0b010) // Measure ITMP (Internal Die Temperature)
#define CHST_VA             (0b011) // Measure VA (Analog Power Supply)
#define CHST_VD             (0b100) // Measure VD (Digital Power Supply)


// Configuration Bits (used with ltc6811_config struct)
#define GPIO1_PULLDOWN      (0b0 << 0)  // Enable GPIO1 pull-down
#define GPIO1_NO_PULLDOWN   (0b1 << 0)  // Disable GPIO1 pull-down
#define GPIO2_PULLDOWN      (0b0 << 1)  // Enable GPIO2 pull-down
#define GPIO2_NO_PULLDOWN   (0b1 << 1)  // Disable GPIO2 pull-down
#define GPIO3_PULLDOWN      (0b0 << 2)  // Enable GPIO3 pull-down
#define GPIO3_NO_PULLDOWN   (0b1 << 2)  // Disable GPIO3 pull-down
#define GPIO4_PULLDOWN      (0b0 << 3)  // Enable GPIO4 pull-down
#define GPIO4_NO_PULLDOWN   (0b1 << 3)  // Disable GPIO4 pull-down
#define GPIO5_PULLDOWN      (0b0 << 4)  // Enable GPIO5 pull-down
#define GPIO5_NO_PULLDOWN   (0b1 << 4)  // Disable GPIO5 pull-down
#define REFON_STAY_POWERED  (0b1)       // References remain powered up until watchdog timeout
#define REFON_SHUT_DOWN     (0b0)       // References shut down after conversions
#define ADCOPT_MODE_0       (0b0)       // For 27kHz, 7kHz, 422Hz, 26Hz ADC Conversion
#define ADCOPT_MODE_1       (0b1)       // For 14kHz, 3kHz, 1kHz, 2kHz ADC Conversion

#endif
