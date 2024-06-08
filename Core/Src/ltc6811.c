#include "main.h"
#include "ltc6811.h"
#include "global_definitions.h"
#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>
#include <math.h>


int16_t pec15Table[256];
const int16_t CRC15_POLY = 0x4599;

void init_PEC15_Table(void)
{
    int16_t remainder;

    for (int i = 0; i < 256; i++)
    {
        remainder = i << 7;
        for (int bit = 8; bit > 0; --bit)
        {
            if (remainder & 0x4000)
            {
                remainder = ((remainder << 1));
                remainder = (remainder ^ CRC15_POLY);
            }
            else
            {
                remainder = ((remainder << 1));
            }
        }
        pec15Table[i] = remainder & 0xFFFF;
    }
}


uint16_t pec15(uint8_t *data, int len)
{
    int16_t remainder, address;

    remainder = 16; // PEC seed

    for (int i = 0; i < len; i++)
    {
        address = ((remainder >> 7) ^ data[i]) & 0xff; // calculate PEC table address
        remainder = (remainder << 8) ^ pec15Table[address];
    }
    return (remainder * 2); // The CRC15 has a 0 in the LSB so the final value must be multiplied by 2
}


void init_LTC6811(void)
{
    init_PEC15_Table();
}


void update_config(ltc6811_config *config)
{
    uint8_t cfgr[6] = {0};

    cfgr[0] = config->gpio_pulldowns << 3;
    cfgr[0] |= config->refon << 2;
    cfgr[0] |= config->adcopt;

    cfgr[1] = config->vuv;

    cfgr[2] = config->vuv >> 8;
    cfgr[2] |= config->vov << 4;

    cfgr[3] = config->vov >> 4;

    cfgr[4] = config -> dcc;

    cfgr[5] = config->dcto << 4;
    cfgr[5] |= config -> dcc >> 4;



    wake_sleep();

    broadcast_write(WRCFGA, cfgr);
}

////NEED TO CHANGE TO HAL
void wake_sleep()
{
	HAL_GPIO_WritePin(LTC6820_CS, GPIO_PIN_RESET);

	HAL_Delay(1);

    HAL_GPIO_WritePin(LTC6820_CS, GPIO_PIN_SET);

    HAL_Delay(1);
}


void wake_standby()
{
	HAL_GPIO_WritePin(LTC6820_CS, GPIO_PIN_RESET);

	HAL_Delay(1);

    HAL_GPIO_WritePin(LTC6820_CS, GPIO_PIN_SET);

    HAL_Delay(1);
}


void broadcast_command(uint16_t command_code)
{
    uint8_t CMD[2];

    // see Table 36 (Broadcast Command Format) in LTC6811 datasheet
    CMD[0] = 0;                  // CMD0 bits 3 thru 7  = 0
    CMD[0] |= command_code >> 8; // CMD0 bits 0 thru 2  = top 3 bits of command code
    CMD[1] = command_code;       // CMD1                = bottom 8 bits of command code

    uint16_t crc = pec15(CMD, 2); // CRC for CMD

    uint8_t tx_msg[4];

    // Send 2-byte CMD and 2-byte PEC15.

    tx_msg[0] = CMD[0];
    tx_msg[1] = CMD[1];
    tx_msg[2] = crc >> 8;
    tx_msg[3] = crc;




#ifdef ALWAYS_STANDBY_WAKE
    wake_standby();
#endif
    HAL_GPIO_WritePin(LTC6820_CS, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, tx_msg, 4, 1000);
    HAL_GPIO_WritePin(LTC6820_CS, GPIO_PIN_SET);
}


void broadcast_write(uint16_t command_code, uint8_t *tx_reg)
{
    uint8_t CMD[2];

    // see Table 36 (Broadcast Command Format) in LTC6811 datasheet
    CMD[0] = 0;                  // CMD0 bits 3 thru 7  = 0
    CMD[0] |= command_code >> 8; // CMD0 bits 0 thru 2  = top 3 bits of command code
    CMD[1] = command_code;       // CMD1                = bottom 8 bits of command code

    uint16_t crc = pec15(CMD, 2); // CRC for CMD

    uint8_t tx_msg[12];

    // Send 2-byte CMD and 2-byte PEC15, then 6-byte register and 2-byte PEC15. No shift bytes.

    tx_msg[0] = CMD[0];
    tx_msg[1] = CMD[1];
    tx_msg[2] = crc >> 8;
    tx_msg[3] = crc;

    crc = pec15(tx_reg, 6); // CRC for register value

    tx_msg[4] = tx_reg[0];
    tx_msg[5] = tx_reg[1];
    tx_msg[6] = tx_reg[2];
    tx_msg[7] = tx_reg[3];
    tx_msg[8] = tx_reg[4];
    tx_msg[9] = tx_reg[5];
    tx_msg[10] = crc >> 8;
    tx_msg[11] = crc;







#ifdef ALWAYS_STANDBY_WAKE
    wake_standby();
#endif
    HAL_GPIO_WritePin(LTC6820_CS, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, tx_msg, 12, 1000);
    HAL_GPIO_WritePin(LTC6820_CS, GPIO_PIN_SET);
}


void address_write(uint8_t address, uint16_t command_code, uint8_t *tx_reg)
{
    uint8_t CMD[2];

    // see Table 37 (Address Command Format) in LTC6811 datasheet
    CMD[0] = 0x80;               // CMD0 bit 7          = 1
    CMD[0] |= address << 3;      // CMD0 bits 3 thu 6   = address
    CMD[0] |= command_code >> 8; // CMD0 bits 0 thru 2  = top 3 bits of command code
    CMD[1] = command_code;       // CMD1                = bottom 8 bits of command code

    uint16_t crc = pec15(CMD, 2); // CRC for CMD

    uint8_t tx_msg[12];

    // See Table 33 (Address Write Command) in LTC6811 datasheet.
    // Send 2-byte CMD and 2-byte PEC15, then 6-byte register and 2-byte PEC15

    tx_msg[0] = CMD[0];
    tx_msg[1] = CMD[1];
    tx_msg[2] = crc >> 8;
    tx_msg[3] = crc;

    crc = pec15(tx_reg, 6); // CRC for register value

    tx_msg[4] = tx_reg[0];
    tx_msg[5] = tx_reg[1];
    tx_msg[6] = tx_reg[2];
    tx_msg[7] = tx_reg[3];
    tx_msg[8] = tx_reg[4];
    tx_msg[9] = tx_reg[5];
    tx_msg[10] = crc >> 8;
    tx_msg[11] = crc;


#ifdef ALWAYS_STANDBY_WAKE
    wake_standby();
#endif
    HAL_GPIO_WritePin(LTC6820_CS, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, tx_msg, 12, 1000);
    HAL_GPIO_WritePin(LTC6820_CS, GPIO_PIN_SET);
}



void address_read(uint8_t address, uint16_t command_code, uint8_t *rx_reg)
{
    uint8_t CMD[2];

    // see Table 37 (Address Command Format) in LTC6811 datasheet
    CMD[0] = 0x80;               // CMD0 bit 7          = 1
    CMD[0] |= address << 3;      // CMD0 bits 3 thu 6   = address
    CMD[0] |= command_code >> 8; // CMD0 bits 0 thru 2  = top 3 bits of command code
    CMD[1] = command_code;       // CMD1                = bottom 8 bits of command code

    uint16_t crc = pec15(CMD, 2);

    uint8_t tx_msg[12];

    // Send 2-byte CMD and PEC15, then read 6-byte register and 2-byte PEC15

    tx_msg[0] = CMD[0];
    tx_msg[1] = CMD[1];
    tx_msg[2] = crc >> 8;
    tx_msg[3] = crc;
    tx_msg[4] = 0;
    tx_msg[5] = 0;
    tx_msg[6] = 0;
    tx_msg[7] = 0;
    tx_msg[8] = 0;
    tx_msg[9] = 0;
    tx_msg[10] = 0;
    tx_msg[11] = 0;

    uint8_t rx_msg[8] = {0};

	#ifdef ALWAYS_STANDBY_WAKE
		wake_standby();
	#endif
	HAL_GPIO_WritePin(LTC6820_CS, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, tx_msg, 4, 1000);
    HAL_SPI_Receive(&hspi3, rx_msg, 8, 1000);
    HAL_GPIO_WritePin(LTC6820_CS, GPIO_PIN_SET);
        crc = pec15(rx_msg, 6); // calculate PEC15 for received message (first 6 bytes)

        uint16_t rx_crc = (rx_msg[6] << 8) + rx_msg[7]; // received PEC15

        if (crc == rx_crc)
        {
            for (int i = 0; i < 6; i++)
            {
                rx_reg[i] = rx_msg[i];
            }
        }
}



void extract_voltage_reg(uint8_t *voltage_reg, float *voltages)
{
    for (int i = 0; i < 3; i++)
    {
        if (voltage_reg[i * 2] == 0xFF && voltage_reg[(i * 2) + 1] == 0xFF)
        {
#ifdef NAN
            voltages[i] = NAN;
#endif
        }
        else
        {
            voltages[i] = (float)(voltage_reg[i * 2] + (voltage_reg[(i * 2) + 1] << 8)) * 0.0001f;
        }
    }
}


void extract_all_voltages(ltc6811 *ltc6811, float *cell_voltage, int slave_num)
{

    int cell = 0;

    for (int slave = 0; slave < slave_num; slave++)
    {
        int i_max = ltc6811[slave].cell_count;

        float reg_voltages[12];

        extract_voltage_reg(ltc6811[slave].cva_reg, &reg_voltages[0]);
        extract_voltage_reg(ltc6811[slave].cvb_reg, &reg_voltages[3]);
        extract_voltage_reg(ltc6811[slave].cvc_reg, &reg_voltages[6]);
        extract_voltage_reg(ltc6811[slave].cvd_reg, &reg_voltages[9]);

        for (int i = 0; i < i_max; i++)
        {
            cell_voltage[cell] = reg_voltages[i];
            cell++;
        }
    }
}


void read_all_voltages(ltc6811 *ltc6811, int slave_num)
{

    for (int slave = 0; slave < slave_num; slave++)
    {
        address_read(ltc6811[slave].address, RDCVA, ltc6811[slave].cva_reg);
        address_read(ltc6811[slave].address, RDCVB, ltc6811[slave].cvb_reg);
        address_read(ltc6811[slave].address, RDCVC, ltc6811[slave].cvc_reg);
        address_read(ltc6811[slave].address, RDCVD, ltc6811[slave].cvd_reg);
    }
}

void generate_i2c(uint8_t * comm_reg, uint8_t *comm_data, uint8_t len)
// comm_data is an array with maximum 3 bytes to be written to COMM register
// len is number of bytes to be written (how many are in comm_data)

{

	switch (len) {

	//maybe remove bitwise or because not needed when shifting (automatically populated 0s)

	case 2:
        comm_reg[0] = ((comm_data[0] >> 4) & 0b00001111) | 0b01100000; //mask with upper half data bit and start bits
	    comm_reg[1] = (comm_data[0] << 4) & 0b11110000; //mask with lower half data byte and master ack bits
	    comm_reg[2] = (comm_data[1] >> 4) & 0b00001111; //mask with upper half data bit and blank bits
	    comm_reg[3] = ((comm_data[1] << 4) & 0b11110000) | 0b00001001; //mask with lower half data byte and master ack bits
	    comm_reg[4] = 0x00;
	    comm_reg[5] = 0x00;
	    break;

	case 3:

	    comm_reg[0] = ((comm_data[0] >> 4) & 0b00001111) | 0b01100000; //mask with upper half data bit and start bits
	    comm_reg[1] = (comm_data[0] << 4) & 0b11110000; //mask with lower half data byte and master ack bits
	    comm_reg[2] = (comm_data[1] >> 4) & 0b00001111; //mask with upper half data bit and blank bits
	    comm_reg[3] = (comm_data[1] << 4) & 0b11110000; //mask with lower half data byte and master ack bits
	    comm_reg[4] = (comm_data[2] >> 4) & 0b00001111; //mask with upper half data bit and blank bits
	    comm_reg[5] = ((comm_data[2] << 4) & 0b1111) | 0b00001001; //mask with lower half data byte and master ack bits

		//how to send stop bits (does it automatically if using all 5 bytes?)
	    break;
    }
}


void send_comm(uint8_t *i2c_message, uint8_t len, int mux_num) {

    uint8_t comm_reg[6];

    generate_i2c(comm_reg, i2c_message, len);

    if (mux_num)
    {
    	comm_reg[1] |= 0b00100000;
    }


    wake_sleep();

    broadcast_write(WRCOMM, comm_reg);

    broadcast_command_stcomm(STCOMM);


}


void broadcast_command_stcomm(uint16_t command_code)
{
uint8_t CMD[2];

// see Table 36 (Broadcast Command Format) in LTC6811 datasheet
CMD[0] = 0;                  // CMD0 bits 3 thru 7  = 0
CMD[0] |= command_code >> 8; // CMD0 bits 0 thru 2  = top 3 bits of command code
CMD[1] = command_code;       // CMD1                = bottom 8 bits of command code

uint16_t crc = pec15(CMD, 2); // CRC for CMD


// Send 2-byte CMD and 2-byte PEC15.





uint8_t tx_msg[13] = {CMD[0], CMD[1], crc >> 8, crc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};



#ifdef ALWAYS_STANDBY_WAKE
wake_standby();
#endif

HAL_GPIO_WritePin(LTC6820_CS, GPIO_PIN_RESET);
HAL_SPI_Transmit(&hspi3, tx_msg, 13, 1000);
HAL_GPIO_WritePin(LTC6820_CS, GPIO_PIN_SET);
}


double calc_temp(double adc_voltage) {
	//stole this shit from arduino forum!!!
  double steinhart;
  double resistance = 10000 * adc_voltage / (3 - adc_voltage);
  steinhart = resistance / 10000;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= 3950;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (25 + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;

  return steinhart;
}


int read_all_temps(ltc6811 *ltc6811_arr, float *thermistor_temps, uint8_t mux_channels, int slave_num)
{
	double thermistor_voltage;
	int thermistor_num = 0;
	int overtemp_limit = 100;
	uint8_t mux_off[2] = {0b10010000, 0b00000000};


	//FOR EACH SLAVE
	  wake_sleep();
	  broadcast_command(ADAX(MD_27k_14k, CHG_GPIO_3)); //measure gpio 3 (non mux'd thermistor)

	  broadcast_command(ADAX(MD_27k_14k, CHG_GPIO_2)); //measure gpio 2 (non mux'd thermistor)

	  for (int slave = 0; slave < slave_num; slave++) //loop through and read every slave AUXA register to see temps
	  {

		  ltc6811 selected_slave = ltc6811_arr[slave]; //increment over all slaves


		  //READ GPIO 3
		  address_read(selected_slave.address, RDAUXA, selected_slave.auxa_reg);

		  thermistor_voltage = ((selected_slave.auxa_reg[5] << 8) | selected_slave.auxa_reg[4]) * 0.0001;

		  thermistor_temps[thermistor_num] = calc_temp(thermistor_voltage); //convert voltage to temperature in degrees celcius

		  if(thermistor_temps[thermistor_num] > overtemp_limit) //if overtemp, trigger shutdown
		  {
			  return 1; //ADD SDC
		  }

		  thermistor_num++;


		 for (int mux = 0; mux < 2; mux++) //loop through both muxes on a slave
			 {
			  	 uint8_t i2c_data[2] = {0b10010000, 0b00001000};	//bits 4 - 7 are address bits for the mux IC, bits 11 - 15 are the address bits for the mux channel, start with channel 0

				 for (int mux_channel = 0;  mux_channel < mux_channels; mux_channel++)
				 	 {

					 send_comm(i2c_data, 2, mux); //generate commands to access each mux channel

					 broadcast_command(ADAX(MD_27k_14k, CHG_GPIO_1)); //measure gpio 1 (mux output)


					 address_read(selected_slave.address, RDAUXA, selected_slave.auxa_reg); //read auxa_reg where adc value was stored


					 thermistor_voltage = ((selected_slave.auxa_reg[1] << 8) | selected_slave.auxa_reg[0]) * 0.0001;


					 thermistor_temps[thermistor_num] = calc_temp(thermistor_voltage); //convert voltage to temperature in degrees celcius


					 if(thermistor_temps[thermistor_num] > overtemp_limit) //if overtemp, trigger shutdown
					 	 {
						  return 1; //AMS_OK fault
					 	 }

					  thermistor_num++;

					  i2c_data[1]++;

				 	 }
				  send_comm(mux_off, 2, mux);

			 }


		  address_read(selected_slave.address, RDAUXA, selected_slave.auxa_reg);

		  thermistor_voltage = ((selected_slave.auxa_reg[3] << 8) | selected_slave.auxa_reg[2]) * 0.0001;

		  thermistor_temps[thermistor_num] = calc_temp(thermistor_voltage); //convert voltage to temperature in degrees celcius

		  if(thermistor_temps[thermistor_num] > overtemp_limit) //if overtemp, trigger shutdown
		  {
			  return 1; //ADD SDC
		  }

		  thermistor_num++;
	  }




	//ADD TO ARRAY
	//LOOP THROUGH MUX 0 AND MUX 1
	//ADD TO ARRAY
	//READ GPIO 2
	//ADD TO ARRAY









	  return 0;




}




//
//
//
//	double thermistor_temp;
//		double thermistor_voltage;
//		int thermistor_num = 0;
//		int overtemp_limit = 60;
//
//		wake_sleep();
//
//
//		  broadcast_command(ADAX(MD_27k_14k, CHG_GPIO_3)); //measure gpio 3 (non mux'd thermistor)
//
//
//		  for (int slave = 0; slave < slave_num; slave++) //loop through and read every slave AUXA register to see temps
//		  {
//			  ltc6811 selected_slave = ltc6811_arr[slave]; //increment over all slaves
//
//
//			  address_read(selected_slave.address, RDAUXA, selected_slave.auxa_reg);
//
//			  thermistor_voltage = ((selected_slave.auxa_reg[5] << 8) | selected_slave.auxa_reg[4]) * 0.0001;
//
//			  thermistor_temps[thermistor_num] = calc_temp(thermistor_voltage); //convert voltage to temperature in degrees celcius
//
//			  if(thermistor_temp > overtemp_limit) //if overtemp, trigger shutdown
//			  {
//				  return 1; //ADD SDC
//			  }
//
//			  thermistor_num++;
//		  }
//
//
//
//			for (int mux = 0; mux < 2; mux++){ //loop through both muxes on a slave
//				uint8_t i2c_data[2] = {0b10010000, 0b00001000};	//bits 4 - 7 are address bits for the mux IC, bits 11 - 15 are the address bits for the mux channel, start with channel 0
//			for (int mux_channel = 0;  mux_channel < mux_channels; mux_channel++)
//			{
//
//			  wake_sleep(); // wake LTC6811 from sleep
//			  send_comm(i2c_data, 2, mux); //generate commands to access each mux channel
//
//
//			  broadcast_command(ADAX(MD_27k_14k, CHG_GPIO_1)); //measure gpio 1 (mux output)
//
//			  for (int slave = 0; slave < slave_num; slave++) //loop through and read every slave AUXA register to see temps
//			  {
//				  ltc6811 selected_slave = ltc6811_arr[slave]; //increment over all slaves
//				  address_read(selected_slave.address, RDAUXA, selected_slave.auxa_reg); //read auxa_reg where adc value was stored
//
//
//				  thermistor_voltage = ((selected_slave.auxa_reg[1] << 8) | selected_slave.auxa_reg[0]) * 0.0001;
//
//				  thermistor_temps[thermistor_num] = calc_temp(thermistor_voltage); //convert voltage to temperature in degrees celcius
//
//
//				  if(thermistor_temps[thermistor_num] > overtemp_limit) //if overtemp, trigger shutdown
//				  {
//					  return 1; //AMS_OK fault
//				  }
//
//
//			  thermistor_num++;
//
//			  i2c_data[1]++;
//			  }
//
//			}
//			}
//
//
//
//
//
//
//
//
//
//
//
//			  broadcast_command(ADAX(MD_27k_14k, CHG_GPIO_2)); //measure gpio 2 (non mux'd thermistor)
//
//			  				  for (int slave = 0; slave < slave_num; slave++) //loop through and read every slave AUXA register to see temps
//			  				  {
//			  				  ltc6811 selected_slave = ltc6811_arr[slave]; //increment over all slaves
//			  				  address_read(selected_slave.address, RDAUXA, selected_slave.auxa_reg);
//
//			  				  thermistor_voltage = ((selected_slave.auxa_reg[3] << 8) | selected_slave.auxa_reg[2]) * 0.0001;
//
//			  				  thermistor_temps[thermistor_num] = calc_temp(thermistor_voltage); //convert voltage to temperature in degrees celcius
//
//			  				  if(thermistor_temp > overtemp_limit) //if overtemp, trigger shutdown
//			  				  {
//			  					  return 1; //ADD SDC
//			  				  }
//
//			  				  thermistor_num++;
//			  				  }
//
//
//
//
//
//
//



//
//		  return 0;












