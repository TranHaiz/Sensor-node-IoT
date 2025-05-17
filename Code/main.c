#include "sgp30.h"
#include "systick.h"
#include "dht11.h"
#include "adc.h"
#include "usart.h"
#include <math.h>
#include "filter.h"
#include "convt.h"
#include <string.h>


#define devId_msb 0x12
#define devId_lsb 0x80

#define devId 0x1280

#define SLEEP (SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk)

#define TIMEOUT 1000000
#define USARTx USART1
#define dht_size 1
#define dht_delay 1
#define co2_size 10
#define co2_delay 1
#define adc_size 100
#define adc_delay 10
#define size 50
#define USART_SIZE 17
#define dry_value 2750
#define wet_value 1270
//--------------------------------------

int check1=0;
int timeout=0;

uint32_t do_am_dat[adc_size];
uint32_t anh_sang[adc_size];
float temp[dht_size];
float hum[dht_size];
uint32_t co2[co2_size];

float lux[adc_size];
float vwc[adc_size];

float Temp=0;
float Hum=0;
float Lux=0;
float Vwc=0;
float Co2=0;

uint8_t work=0;
int count=0;


enum{
	MSB_ID,
	LSB_ID,
	CMD,
	CRC8,
	RECEIVE_SIZE
}data_receive_type;

uint8_t data_receive[RECEIVE_SIZE];
int i_receive=0;

enum {
    HUM_MSB,
    HUM_LSB,
    
    TEMP_MSB,
    TEMP_LSB,
    
    CO2_MSB,
    CO2_LSB,
    CO2_DEC,

    LUX_MSB,
    LUX_LSB,
    LUX_DEC,

    VWC_MSB,
    VWC_LSB,
	
		CHECKSUM,
	
//		ID_MSB,
//		ID_LSB,
		DATA_SIZE
} data_type_t;
uint8_t data[USART_SIZE] = {0};
//--------------------------------------

/*function*/
void VWC_READ(void){
	for(int i=0;i<size*2;i++)
		{
			do_am_dat[i]=ADCx_READ(ADC1,ADC_Channel_2);
			delay_ms(adc_delay);
		}
//		SGP30_START();
}
void LUX_READ(void){
	for(int i=0;i<size*2;i++)
		{
			delay_ms(1);
			anh_sang[i]=ADCx_READ(ADC1,ADC_Channel_7);
			anh_sang[0]=anh_sang[1];
		}
//		SGP30_START();
}
void DHT_READ(void){
	for(int i=0;i<dht_size;i++)
		{
//			hum[i]=dht_hum();
//			delay_ms(dht_delay);
//			temp[i]=dht_temp();
			dht_temp_hum(&temp[i],&hum[i]);
			delay_s(dht_delay);
		}
}
void CO2_READ(void){
//	SGP30_START();
//		delay_s(15);
		int i=0;
		while(i<co2_size)
		{
			int buff=SGP30_READ();
			if(buff != 000) 
			{
				co2[i]=SGP30_READ();
				i++;
			}
			delay_ms(co2_delay);
		}
}




float adc_to_lux(uint32_t adc){
    float adc_cpy = (float)adc;
    float res = ((10000*adc_cpy)/(4095-adc_cpy))/1000;;
    return ( pow(pow(10,1.905)/(res+0.031),2) );
}


void change_lux(void){
    for (int i= 0; i < adc_size; i++)
    {
        lux[i] = adc_to_lux(anh_sang[i]);
    }
}

float t=0;
//float adc_to_vwc(uint32_t adc){
////	float volt = ((float)adc/4095.0)*3.3;
////	return ((2.48/volt)-0.72);
//	uint32_t adc_10bit = (1023.0/4095.0)*adc;
//	t=adc_10bit;
//	return ( 100 - ((adc_10bit - wet_value)/( dry_value - wet_value)));
//}
//float adc_to_vwc(uint32_t adc){
//	uint32_t adc_10bit = (adc * 1023) / 4095;
//	float water = 100.0f - ((float)(adc_10bit - wet_value) / (dry_value - wet_value)) * 100.0f;
//	if (water > 100.0f) water = 100.0f;
//	if (water < 0.0f) water = 0.0f;
//	return water;
//}

float adc_to_vwc(uint32_t adc){
    float water = 100.0f - ((float)(adc - wet_value) / (dry_value - wet_value)) * 100.0f;
    if (water > 100.0f) water = 100.0f;
    if (water < 0.0f) water = 0.0f;
    return water;
}

void change_vwc(void){
	for(int i=0;i<adc_size;i++)
	{
		vwc[i] = adc_to_vwc(do_am_dat[i]);
	}
}
void READ_SENSOR(void){
	LUX_READ();
	VWC_READ();
	DHT_READ();
	CO2_READ();
}
void turnOff_usart(void){
	USARTx->CR1 &= ~(1<<13);
}
void turnOn_usart(void){
	USARTx->CR1 |= 1<<13;
}
uint8_t calculate_crc(uint8_t *data) {
    uint16_t sum = 0;
    for (int i = 0; i < CHECKSUM; i++) {
        sum += data[i];
    }
    return (uint8_t)(sum & 0xff);
}
//--------------------------------------
void filter(void){
	change_lux();
	change_vwc();
	Temp=avg_float_filter(temp,dht_size);
	Hum=avg_float_filter(hum,dht_size);
	Lux=avg_float_filter(lux,adc_size);
	Vwc=avg_float_filter(vwc,adc_size);
	Co2=avg_uint32_filter(co2,co2_size);
}
void convt(void) {
//		data[ID_MSB] = devId_msb;
//    data[ID_LSB] = devId_lsb;
		
    Float8i8d_FromFloat(Temp, &data[TEMP_MSB], &data[TEMP_LSB]);
    Float8i8d_FromFloat(Hum,  &data[HUM_MSB],  &data[HUM_LSB]);
    Float16i8d_FromFloat(Co2, &data[CO2_MSB],  &data[CO2_LSB], &data[CO2_DEC]);
    Float16i8d_FromFloat(Lux, &data[LUX_MSB],  &data[LUX_LSB], &data[LUX_DEC]);
    Float8i8d_FromFloat(Vwc,   &data[VWC_MSB],  &data[VWC_LSB]);
		data[CHECKSUM] = calculate_crc(data);
}
void send_data(uint8_t *data, uint8_t length) {
    char buffer[128] = "";
    char temp[8];

    for (int i = 0; i < length; i++) {
        snprintf(temp, sizeof(temp), "%d", data[i]);
        strcat(buffer, temp);
        if (i < length - 1) strcat(buffer, ";");
    }
    strcat(buffer, ".");
		
    USARTx_WRTITE(USARTx,buffer);
}
void parse_data(const char *str) {
    char temp[100];
    strncpy(temp, str, sizeof(temp) - 1);
    temp[sizeof(temp) - 1] = '\0';

    char *token = strtok(temp, ";");
    int i = 0;
    while (token && i < RECEIVE_SIZE) {
        data_receive[i++] = atoi(token);
        token = strtok(NULL, ";");
    }
}

void reset_data(void)
{
	for (int i = 0; i < DATA_SIZE; i++) {
        data[i] = 0;
    }
	return;
}
//--------------------------------------
int state=1;
int main()
{
	delay_init();
	check1=1;
	ADCx_init(ADC1,ADC_Channel_2); //do am dat
	ADCx_init(ADC1,ADC_Channel_7); //anh sang
	dht_init(GPIOB,10); //TEMP, HUM
	SGP30_INIT(I2C1,I2C1_Pin_PB6PB7); //CO2
	check1=2;
	USARTx_INIT(USARTx,USART1_PA9PA10,9600);
	SGP30_START();
	check1=3;
	USARTx_WRTITE(USARTx,"reset;");
	
	
	
	while(1)
	{
		if(work)
		{
			if(state){
				state=0;
//				USARTx_WRTITE(USARTx,"ON;");
			}
			
			turnOff_usart();
			delay_s(1);
			READ_SENSOR();
			filter();
			convt();
			turnOn_usart();
			send_data(data,DATA_SIZE);
			count++;
			work=0;
		}
		else
		{
//			if(state){
//				state=0;
//				USARTx_WRTITE(USARTx,"OFF;");
//			}
			
			turnOn_usart();
			delay_s(1);
			SLEEP;
		}
	};
}
int check=0;
char buff[14];
void USART1_IRQHandler(void) {
	volatile int i;
	for (i = 0; i < 10; i++) __NOP();

    uint8_t buffer = USARTx->DR;

    if (buffer != '.') {
			buff[i_receive++] = buffer;
        
    } else {
				parse_data(buff);
        
            if (((data_receive[MSB_ID] + data_receive[LSB_ID] + data_receive[CMD]) & 0xFF) == data_receive[CRC8]) {
                if ((data_receive[MSB_ID] == devId_msb) && (data_receive[LSB_ID] == devId_lsb)) {
                    if (data_receive[CMD] == 1) work = 1;
										else if (data_receive[CMD] == 2) {send_data(data,DATA_SIZE);check=2;}
										else if (data_receive[CMD] == 3) {reset_data();check=3;}
                    //new--cmd
                }
            }
        
        i_receive = 0; // 
    }
		state=1;
}
