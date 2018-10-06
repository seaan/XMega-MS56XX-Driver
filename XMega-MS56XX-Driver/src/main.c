#include <asf.h>
#include <adc.h>
#include <math.h>
#include "Drivers/uart.h"
#include "Drivers/SPI.h"
#include "Drivers/MS56XX.h"

#define PRESSURE_SELECT_PIN		IOPORT_CREATE_PIN(PORTC, 4)

#define HOTWIRE PIN0_bm
#define BUZZ PIN4_bm
#define LED PIN1_bm

float get_alt(uint32_t init_press, uint32_t press, uint32_t temp);
void TCC0_init(uint16_t period, float duty);

int main (void)
{	/* Insert system clock initialization code here (sysclk_init()). */

	sysclk_init();
	
	PORTE.DIR |= HOTWIRE | BUZZ; // hotwire and buzzer dir
	PORTE.OUT &= ~(HOTWIRE | BUZZ); // hotwire and buzzer low
	
	PORTC.DIR |= LED; // LED dir
	PORTC.OUT |= LED; // LED high
	
	MS56XX_t pressure_sensor = define_new_MS56XX_default_OSR(MS5607, &SPIC, PRESSURE_SELECT_PIN);
	
	initializespi(&SPIC, &PORTC);
	enable_select_pin(pressure_sensor.select_pin);
	
	//Pressure sensor initialization routine, also reads calibration data from sensor
	calibratePressureSensor(&pressure_sensor);
	
	readMS56XX(&pressure_sensor);
	
	uint32_t init_press = pressure_sensor.data.pressure;

	/* Insert application code here, after the board has been initialized. */

	uart_terminal_init();
	
	float alt = 0;
	uint8_t flight_state = 0;
	
	TCC0_init(62499,.5);
	
	while(1)
	{
		switch(flight_state)
		{
			case 0:
				readMS56XX(&pressure_sensor);
				alt = get_alt(init_press, pressure_sensor.data.pressure, pressure_sensor.data.temperature);
				
				printf("FS0, %lu, %lu, %lu\n", alt, pressure_sensor.data.pressure, pressure_sensor.data.temperature);
				
				if(alt > 30)
				{
					TCC0_init(31249,.1);
					flight_state = 1;
				}
			break;

			case 1:
				readMS56XX(&pressure_sensor);
				alt = get_alt(init_press, pressure_sensor.data.pressure, pressure_sensor.data.temperature);
				
				printf("FS0, %lu, %lu, %lu\n", alt, pressure_sensor.data.pressure, pressure_sensor.data.temperature);
				
				if(alt > 700)
				{
					TCC0_init(6294,.10);
					
					PORTE.OUT |= HOTWIRE; // cutdown
					delay_ms(4000);
					PORTE.OUT &= ~HOTWIRE;
					
					flight_state = 2;
				}
			break;
			
			case 2:
				readMS56XX(&pressure_sensor);
				alt = get_alt(init_press, pressure_sensor.data.pressure, pressure_sensor.data.temperature);
			
				printf("FS0, %lu, %lu, %lu\n", alt, pressure_sensor.data.pressure, pressure_sensor.data.temperature);
				
				if(alt < 50)
				{
					TCC0_init(62499,.1);
					
					PORTE.OUT |= BUZZ; // buzz
					
					flight_state = 3;
				}
			break;
			
			case 3:
				readMS56XX(&pressure_sensor);
				alt = get_alt(init_press, pressure_sensor.data.pressure, pressure_sensor.data.temperature);
				
				printf("FS0, %lu, %lu, %lu\n", alt, pressure_sensor.data.pressure, pressure_sensor.data.temperature);
			break;
		}
		delay_ms(500);
	}
}

float get_alt(uint32_t init_press, uint32_t press, uint32_t temp)
{
	float R = 287;
	float g = 9.80665;
	float pconst = 101325;
	return (((R * ((float)temp/100+273.15))/g)*log((float)init_press/(float)press)) * 3.28084; //return altitude in feet
}

void TCC0_init(uint16_t period, float duty)
{
	sysclk_enable_peripheral_clock(&TCC0);
	sysclk_enable_module(SYSCLK_PORT_C, SYSCLK_HIRES);
	TCC0.CTRLA = 0x07;
	TCC0.CTRLB = 0x23;
	TCC0.PER = period;
	TCC0.CCB = TCC0.PER*duty;
}
