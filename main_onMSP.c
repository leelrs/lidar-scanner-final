#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"





#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          																// 7) disable analog functionality on PB2,3

                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        						// 8) configure for 100 kbps clock
        
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}

void PortH_Init(void){
	//Use PortH pins (PH0-PH3) for output
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;				// activate clock for Port H
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};	// allow time for clock to stabilize
	GPIO_PORTH_DIR_R |= 0x0F;        								// configure Port H pins (PH0-PH3) as output
  GPIO_PORTH_AFSEL_R &= ~0x0F;     								// disable alt funct on Port M pins (PH0-PH3)
  GPIO_PORTH_DEN_R |= 0x0F;        								// enable digital I/O on Port M pins (PH0-PH3)
																									
  GPIO_PORTH_AMSEL_R &= ~0x0F;     								// disable analog functionality on Port H	pins (PH0-PH3)	
	return;
}


void PortJ_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;					// Activate clock for Port J
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};			// Allow time for clock to stabilize
  GPIO_PORTJ_DIR_R &= ~0x03;    									// Make PJ1 input 
  GPIO_PORTJ_DEN_R |= 0x03;     									// Enable digital I/O on PJ1
	
	GPIO_PORTJ_PCTL_R &= ~0x000000F0;	 						// Configure PJ1 as GPIO 
	GPIO_PORTJ_AMSEL_R &= ~0x03;								// Disable analog functionality on PJ1		
	GPIO_PORTJ_PUR_R |= 0x03;									// Enable weak pull up resistor on PJ1
	GPIO_PORTJ_DATA_R &= ~0x03;
}

void PortM_Init(void){
	//Use PortM pins (PM0-PM3) for input
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;				// activate clock for Port M
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};	// allow time for clock to stabilize
	GPIO_PORTM_DIR_R |= 0x01;        								// configure Port M pins (PM0-PM3) as output
  GPIO_PORTM_AFSEL_R &= ~0x01;     								// disable alt funct on Port M pins (PM0-PM3)
  GPIO_PORTM_DEN_R |= 0x01;        								// enable digital I/O on Port M pins (PM0-PM3)															
  GPIO_PORTM_AMSEL_R &= ~0x01;     								// disable analog functionality on Port M	pins (PM0-PM3)	
	return;
}




//xshut; on tof. active low shutdown input, msp pulls up to vdd to enable the sensor, but cranking it back down puts it in standby. not lvl shfited
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        //output 4 pg0
    GPIO_PORTG_DATA_R &= 0b11111110;                                 
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            
    
}


//*****IMPORTANT FUNCTIONS!!!!!! */
volatile uint16_t spinStat; 
volatile uint16_t direction_status;
volatile uint16_t pauseStat;
volatile int currAngle;
volatile int angle_step;


void spinforward() //hold on how does a stepper work again
{																		
	uint32_t delay = 1;															
	
																			
	GPIO_PORTH_DATA_R = 0b00000011; //oh yeah you have to turn on 2/4 coils at a time depending on where you want the motor
	SysTick_Wait10us(delay);											
	GPIO_PORTH_DATA_R = 0b00000110;													
	SysTick_Wait10us(delay);
	GPIO_PORTH_DATA_R = 0b00001100;													
	SysTick_Wait10us(delay);
	GPIO_PORTH_DATA_R = 0b00001001;													
	SysTick_Wait10us(delay);
	

}

void spinbackward()
{
		uint32_t delay = 1;															
	
		GPIO_PORTH_DATA_R = 0b00001001;
		SysTick_Wait10us(delay);											
		GPIO_PORTH_DATA_R = 0b00001100;													
		SysTick_Wait10us(delay);
		GPIO_PORTH_DATA_R = 0b00000110;													
		SysTick_Wait10us(delay);
		GPIO_PORTH_DATA_R = 0b00000011;													
		SysTick_Wait10us(delay);	
}

void spin(uint32_t direction, int steps)
{
		if(direction == 0)
		{
				for(int i = 0; i<steps; i++)
				{
					spinforward();
					SysTick_Wait10us(1);
				}
				
				currAngle += angle_step;
		}
		
		else if(direction == 1)
		{
			for(int i = 0; i<steps; i++){
				spinbackward();
				SysTick_Wait10us(1);
			}
			
			currAngle -= angle_step;
		}	
}


void startEm(void)
{
	uint16_t data = GPIO_PORTJ_DATA_R & 0x1;

	if(data==0x0)
	{
		while(!(GPIO_PORTJ_DATA_R & 0x1)){};
		SysTick_Wait10ms(5);
	
		spinStat^=1;
		
	}
}

void holdOn(void)
{
	uint16_t data = GPIO_PORTJ_DATA_R & 0x2;

	if(data==0x0)
	{
		while(!(GPIO_PORTJ_DATA_R & 0x2)){};
		SysTick_Wait10ms(5);
		pauseStat^=1;
	}
}

void clocktest(void) {

    GPIO_PORTM_DEN_R |= 0x01;  // Enable digital function on PM0
    GPIO_PORTM_DIR_R |= 0x01;  // Set PM0 as output
    
    while(1) 
		{
				SysTick_Wait10ms(100);
			  GPIO_PORTM_DATA_R ^= 0x01;
    }
}


void home(void)
{

		SysTick_Wait10ms(5);
		if(currAngle>=0)
		{
			direction_status = 1;
			while(currAngle>0)
			{
				spin(1, angle_step);
			}
			
			currAngle = 0;
			direction_status = 0;
			
		}
		
		else if(currAngle<=0)
		{
			direction_status = 0;
			while(currAngle<0)
				spin(0, angle_step);
			
			currAngle = 0;
			direction_status = 1;
		}
		
}






//---------------------------------mmmaaaain function------------------------------------------------------

uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
int status=0;

int main(void) 
{
		uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
		uint16_t wordData;
		uint16_t Distance;
		uint16_t SignalRate;
		uint16_t AmbientRate;
		uint16_t SpadNum; 
		uint8_t RangeStatus;
		uint8_t dataReady;

		//initialize
		PLL_Init();	
		SysTick_Init();
		onboardLEDs_Init();
		I2C_Init();
		UART_Init();
		PortH_Init();
		PortJ_Init();
		PortM_Init();
		
		direction_status = 0;
		spinStat = 0;
		pauseStat = 0;
		angle_step = 16;
		currAngle = 0;
		
		//clocktest(); dont run ts WHEN DOING THE DEMO IT WILL NOT WORK :sob: :sob: :sob:
		
		

		//Those basic I2C read functions can be used to check your own I2C functions
		status = VL53L1X_GetSensorId(dev, &wordData);
		
		
		// 1 Wait for device booted
		while(sensorState==0){
			status = VL53L1X_BootState(dev, &sensorState);
			SysTick_Wait10ms(10);
		}
		FlashAllLEDs();
		//UART_printf("ToF Chip Booted!\r\n Please Wait...\r\n");
		
		status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
		
		/* 2 Initialize the sensor with the default setting  */
		status = VL53L1X_SensorInit(dev);
		//Status_Check("SensorInit", status);

	
  /* 3 Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances */
//  status = VL53L1X_SetDistanceMode(dev, 2); /* 1=short, 2=long */
//  status = VL53L1X_SetTimingBudgetInMs(dev, 100); /* in ms possible values [20, 50, 100, 200, 500] */
//  status = VL53L1X_SetInterMeasurementInMs(dev, 200); /* in ms, IM must be > = TB */

    
		status = VL53L1X_StartRanging(dev);   // This function has to be called to enable the ranging
		
		int layerNum = 3;
		int measurementNum = 32;
		int angle = 512/measurementNum;
	
	
		while(spinStat == 0)
		{
			startEm();
		}

		for(int i = 0; i<layerNum; i++)
		{
			
			SysTick_Wait10ms(50);
			
			while(pauseStat == 1)
			{
				holdOn();
				SysTick_Wait10us(1);
			}

			for(int j = 0; j<measurementNum; j++)
			{
				startEm();
				
				while(dataReady == 0)
				{
					status = VL53L1X_CheckForDataReady(dev, &dataReady);
					FlashLED3(1);
					VL53L1_WaitMs(dev, 5);
				}
				dataReady = 0;
				
				spin(0, angle);
				SysTick_Wait10ms(5);
				status = VL53L1X_GetDistance(dev, &Distance);
				sprintf(printf_buffer,"%u %u %u\r\n", Distance, currAngle, spinStat);
				UART_printf(printf_buffer);
				
				FlashLED3(1);
				FlashLED1(1);
				
			}
			
			pauseStat = 1;
			home();
			
			while(pauseStat)
			{
				Button2();
				SysTick_Wait10us(1);
			}
			
			if(spinStat == 0)
			{
				break;
			}
		}
		

		VL53L1X_StopRanging(dev);
		while(1) {};
		
			
}