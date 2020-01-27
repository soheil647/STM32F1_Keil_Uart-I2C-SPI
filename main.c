#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
//#include "USART_STM32F10x.h"
 #include "stm32f10x_i2c.h"
 #include "stm32f10x_adc.h"
 


#define I2Cx_RCC        RCC_APB1Periph_I2C2
#define I2Cx            I2C2
#define I2C_GPIO_RCC    RCC_APB2Periph_GPIOB
#define I2C_GPIO        GPIOB
#define I2C_PIN_SDA     GPIO_Pin_11
#define I2C_PIN_SCL     GPIO_Pin_10

#define SLAVE_ADDRESS    0x40


int hum0=0;
int hum1=0;	
int temp0=0;	
int temp1=0;	
int  lig0=0;
int  lig1=0;	
int  soil0=0;
int  soil1=0;


 static int joint_state;
 static int joint_state2;

void read_ligth_soil(void);
 void read_temp_humid(void);



// seria functions 
void GPIOC_Init(void);
void USART1_Init(void);
void led_toggle(void);
void delay(unsigned int nCount);
void USART1_IRQHandler(void);
int flag_connect=0; 
int  wait_joint =0;
 
//I2C functions
void i2c_init(void);
void i2c_start(void);
void i2c_stop(void);
void i2c_address_direction(uint8_t address, uint8_t direction);
void i2c_transmit(uint8_t byte);
uint8_t i2c_receive_ack(void);
uint8_t i2c_receive_nack(void);
void i2c_write(uint8_t address, uint8_t data);
void i2c_read(uint8_t address, uint8_t* data);
 
uint8_t receivedByte;


void ADC_Configuration(void)
{
  ADC_InitTypeDef  ADC_InitStructure;
  /* PCLK2 is the APB2 clock */
  /* ADCCLK = PCLK2/6 = 72/6 = 12MHz*/
  RCC_ADCCLKConfig(RCC_PCLK2_Div6);

  /* Enable ADC1 clock so that we can talk to it */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  /* Put everything back to power-on defaults */
  ADC_DeInit(ADC1);

  /* ADC1 Configuration ------------------------------------------------------*/
  /* ADC1 and ADC2 operate independently */
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  /* Disable the scan conversion so we do one at a time */
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  /* Don't do contimuous conversions - do them on demand */
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  /* Start conversin by software, not an external trigger */
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  /* Conversions are 12 bit - put them in the lower 12 bits of the result */
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  /* Say how many channels would be used by the sequencer */
  ADC_InitStructure.ADC_NbrOfChannel = 1;

  /* Now do the setup */
  ADC_Init(ADC1, &ADC_InitStructure);
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);

  /* Enable ADC1 reset calibaration register */
  ADC_ResetCalibration(ADC1);
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1));
  /* Start ADC1 calibaration */
  ADC_StartCalibration(ADC1);
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));
}

u16 readADC1(u8 channel)
{
  ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_41Cycles5);
  // Start the conversion
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  // Wait until conversion completion
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
  // Get the conversion value
  return ADC_GetConversionValue(ADC1);
}


int main(void)
{
GPIO_SetBits(GPIOC, GPIO_Pin_13);
//const uint16_t V25 = 1750;// when V25=1.41V at ref 3.3V
//const uint16_t Avg_Slope = 5; //when avg_slope=4.3mV/C at ref 3.3V 


	  led_toggle();
	  delay(1000);
	  led_toggle();
	  delay(1000);
	  led_toggle();
	  delay(1000);
	  led_toggle();
  // Initialize GPIOC PIN13  
     GPIOC_Init();
    // Initialize USART1  
    USART1_Init();
	ADC_Configuration();
	

	USART1_IRQHandler();
	
//	USART_SendData(USART1,0x01);
	  // I2c init
//	  i2c_init();
//	USART_SendData(USART1,0x02);
	
	  delay(1000);
	  led_toggle();
	  wait_joint=1;
	  flag_connect=0;
	
	//USART_SendData(USART1,0x03);	
	
	//while(1){
//read_temp_humid();
//read_ligth_soil();
//	};
	

 
for(int count=0; count<100 &&   flag_connect==0;count++)
{	
	
// start init 	
	 wait_joint=1;
	//Serial.println("ATZ\r\n");
    USART_SendData(USART1,'A');
	  delay(10);
    USART_SendData(USART1,'T');
	  delay(10);
    USART_SendData(USART1,'Z');
	  delay(10);	
	  USART_SendData(USART1,'\r');
	  delay(10);
	  USART_SendData(USART1,'\n');
	  
	  delay(500);
    led_toggle();
	
 //Serial.println("AT+VER\r);
    USART_SendData(USART1,'A');
	  delay(10);
    USART_SendData(USART1,'T');
	  delay(10);
    USART_SendData(USART1,'+');
	  delay(10);	
		USART_SendData(USART1,'V');
	  delay(10);
    USART_SendData(USART1,'E');
	  delay(10);
    USART_SendData(USART1,'R');
	  delay(10);
	  USART_SendData(USART1,'\r');
	  delay(10);
	  //USART_SendData(USART1,'\n');
	  delay(500);	
		led_toggle();
	
 //Serial.println("AT+DC=0\r\n");
    USART_SendData(USART1,'A');
	  delay(10);
    USART_SendData(USART1,'T');
	  delay(10);
    USART_SendData(USART1,'+');
	  delay(10);	
		USART_SendData(USART1,'D');
	  delay(10);
    USART_SendData(USART1,'C');
	  delay(10);
    USART_SendData(USART1,'=');
	  delay(10);
		USART_SendData(USART1,'0');
	  delay(10);
	  USART_SendData(USART1,'\r');
	  delay(10);
	  //USART_SendData(USART1,'\n');
	  delay(500);
    led_toggle();
 

//Serial.println("AT+DR=3\r\n");
     USART_SendData(USART1,'A');
	  delay(10);
    USART_SendData(USART1,'T');
	  delay(10);
    USART_SendData(USART1,'+');
	  delay(10);	
		USART_SendData(USART1,'D');
	  delay(10);
    USART_SendData(USART1,'R');
	  delay(10);
    USART_SendData(USART1,'=');
	  delay(10);
		USART_SendData(USART1,'3');
	  delay(10);
	  USART_SendData(USART1,'\r');
	  delay(10);
	  //USART_SendData(USART1,'\n');
	  delay(500);
    led_toggle();


//Serial.println("AT+RX2DR=3\r\n");
    USART_SendData(USART1,'A');
	  delay(10);
    USART_SendData(USART1,'T');
	  delay(10);
    USART_SendData(USART1,'+');
	  delay(10);	
		USART_SendData(USART1,'R');
	  delay(10);
    USART_SendData(USART1,'X');
	  delay(10);
    USART_SendData(USART1,'2');
	  delay(10);
		USART_SendData(USART1,'D');
	  delay(10);
    USART_SendData(USART1,'R');
	  delay(10);
		USART_SendData(USART1,'=');
	  delay(10);
		USART_SendData(USART1,'3');
	  delay(10);
	  USART_SendData(USART1,'\r');
	  delay(10);
	  USART_SendData(USART1,'\n');
	  delay(500);
    led_toggle();
 
//Serial.println("AT+JOIN=1\r\n"); 
	  USART_SendData(USART1,'A');
	  delay(10);
    USART_SendData(USART1,'T');
	  delay(10);
    USART_SendData(USART1,'+');
	  delay(10);	
		USART_SendData(USART1,'J');
	  delay(10);
    USART_SendData(USART1,'O');
	  delay(10);
    USART_SendData(USART1,'I');
	  delay(10);
		USART_SendData(USART1,'N');
	  delay(10);
    USART_SendData(USART1,'=');
	  delay(10);
		USART_SendData(USART1,'1');
	  delay(10);
	  USART_SendData(USART1,'\r');
	  delay(10);
	  //USART_SendData(USART1,'\n');
	  delay(500);
    led_toggle();
	
		
		
		wait_joint=1;
		
		for(int i=0;i<150 && flag_connect==0 ;i++){ 
	          delay(1000);
	          led_toggle();
			
			      //flag_connect=1; // just for test
			
		   }
		
}			 
	
while(flag_connect==1){
	

	read_temp_humid();
  read_ligth_soil();
	
	//****  sensor data
//uint8_t hum0;
//uint8_t hum1;	
//uint8_t temp0;	
//uint8_t temp1;	
//uint8_t  lig0;
//uint8_t  lig1;	
//uint8_t  soil0;
//uint8_t  soil1;
	
		// AT+SEND=2,000000000000007F0000000000000000,0
				     USART_SendData(USART1,'A');
	            delay(10);
              USART_SendData(USART1,'T');
	            delay(10);
              USART_SendData(USART1,'+');
	            delay(10);	
		          USART_SendData(USART1,'S');
	            delay(10);
              USART_SendData(USART1,'E');
	            delay(10);
              USART_SendData(USART1,'N');
	            delay(10);
		          USART_SendData(USART1,'D');
	            delay(10);
              USART_SendData(USART1,'=');
	           delay(10);
		         USART_SendData(USART1,'2');
						 	delay(10);
		         USART_SendData(USART1,',');
						 	delay(10);
								
						//	USART_SendData(USART1,0x59);
						USART_SendData(USART1,'0');
						delay(10);
						USART_SendData(USART1,'x');
						delay(10);
						USART_SendData(USART1,'3');
						delay(10);
						USART_SendData(USART1,'3');
						delay(10);
//						USART_SendData(USART1,'0');
//						delay(10);
//						USART_SendData(USART1,'x');
//						delay(10);
//						USART_SendData(USART1,'3');
//						delay(10);
//						USART_SendData(USART1,'5');
//						delay(10);
//						USART_SendData(USART1,'0');
//						delay(10);
//						USART_SendData(USART1,'x');
//						delay(10);
//						USART_SendData(USART1,'3');
//						delay(10);
//						USART_SendData(USART1,'7');
//						delay(10);

//							USART_SendData(USART1,hum0);
//						 	 delay(10);
//							 
//		         USART_SendData(USART1,hum1);
//						 	  delay(10);
//		         USART_SendData(USART1,temp0);
//						 	 delay(10);
//		         USART_SendData(USART1,temp1);
//						 	  delay(10);								
//		         USART_SendData(USART1,lig0);
//						 	 delay(10);
//		         USART_SendData(USART1,lig1);
//						 	  delay(10);								
//		         USART_SendData(USART1,soil0);
//						 	 delay(10);
//		         USART_SendData(USART1,soil1);
//						 	  delay(10);								
//		         USART_SendData(USART1,'0');
//						 	 delay(10);
//		         USART_SendData(USART1,'0');
//						 	 delay(10);
//		         USART_SendData(USART1,'0');
//						 	  delay(10);								
//		         USART_SendData(USART1,'0');
//						 	 delay(10);
//		         USART_SendData(USART1,'0');
//						 	  delay(10);								
//		         USART_SendData(USART1,'0');
//						 	 delay(10); 
//		         USART_SendData(USART1,'0');
//						 	 delay(10);
//		         USART_SendData(USART1,'0');
//						 	 delay(10);
//		         USART_SendData(USART1,'0');
//						 	  delay(10);								
//		         USART_SendData(USART1,'0');
//						 	 delay(10);
//		         USART_SendData(USART1,'0');
//						 	  delay(10);								
//		         USART_SendData(USART1,'0');
//						 	 delay(10); 
//USART_SendData(USART1,'0');
//						 	  delay(10);								
//		         USART_SendData(USART1,'0');
//						 	 delay(10); 								 
//		         USART_SendData(USART1,'0');//7
//						 	  delay(10);								
//		         USART_SendData(USART1,'0');//E
//						 	 delay(10); 								
//	
//	 		         USART_SendData(USART1,'0');
//						 	 delay(10);
//		         USART_SendData(USART1,'0');
//						 	  delay(10);

//	 		         USART_SendData(USART1,'0');
//						 	 delay(10);
//		         USART_SendData(USART1,'0');
//						 	  delay(10);
//		         USART_SendData(USART1,'0');
//						 	 delay(10);
//		         USART_SendData(USART1,'0');
//						 	  delay(10);								
//		         USART_SendData(USART1,'0');
//						 	 delay(10);
//		         USART_SendData(USART1,'0');
//						 	  delay(10);								
//		         USART_SendData(USART1,'0');
//						 	 delay(10);
//		         USART_SendData(USART1,'0');
//						 	  delay(10);															
//		         USART_SendData(USART1,'0');
//						 	 delay(10);
//		         USART_SendData(USART1,'0');
//						 	 delay(10);
//		         USART_SendData(USART1,'0');
//						 	  delay(10);								
//		         USART_SendData(USART1,'0');
//						 	 delay(10);
//		         USART_SendData(USART1,'0');
//						 	  delay(10);								
//		         USART_SendData(USART1,'0');
//						 	 delay(10);												 

		         USART_SendData(USART1,',');
						 	delay(10);
		         USART_SendData(USART1,'1');
						 	 delay(10);
		   	     USART_SendData(USART1,'\r');
	            delay(10);
	            USART_SendData(USART1,'\n');
	            delay(5000);
              led_toggle();
		
    }
}   
 
/***********************************************
 * Initialize GPIOA PIN8 as push-pull output
 ***********************************************/
void GPIOC_Init(void)
{
 
																					 
		GPIO_InitTypeDef GPIO_InitStruct;																			 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
																					 
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStruct);
																					 
																					 
																					 
																					 
}
 
/*****************************************************
 * Initialize USART1: enable interrupt on reception
 * of a character
 *****************************************************/
void USART1_Init(void)
{
    /* USART configuration structure for USART1 */
    USART_InitTypeDef usart1_init_struct;
    /* Bit configuration structure for GPIOA PIN9 and PIN10 */
    GPIO_InitTypeDef gpioa_init_struct;
     
    /* Enalbe clock for USART1, AFIO and GPIOA */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO | 
                           RCC_APB2Periph_GPIOA, ENABLE);
                            
    /* GPIOA PIN9 alternative function Tx */
    gpioa_init_struct.GPIO_Pin = GPIO_Pin_9;
    gpioa_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
    gpioa_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpioa_init_struct);
    /* GPIOA PIN9 alternative function Rx */
    gpioa_init_struct.GPIO_Pin = GPIO_Pin_10;
    gpioa_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
    gpioa_init_struct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpioa_init_struct);
 
    /* Enable USART1 */
    USART_Cmd(USART1, ENABLE);  
    /* Baud rate 115200, 8-bit data, One stop bit
     * No parity, Do both Rx and Tx, No HW flow control
     */
    usart1_init_struct.USART_BaudRate = 115200;   
    usart1_init_struct.USART_WordLength = USART_WordLength_8b;  
    usart1_init_struct.USART_StopBits = USART_StopBits_1;   
    usart1_init_struct.USART_Parity = USART_Parity_No ;
    usart1_init_struct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    usart1_init_struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    /* Configure USART1 */
    USART_Init(USART1, &usart1_init_struct);
    /* Enable RXNE interrupt */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    /* Enable USART1 global interrupt */
    NVIC_EnableIRQ(USART1_IRQn);
}
 
/*******************************************
 * Toggle LED 
 *******************************************/
void led_toggle(void)
{
    /* Read LED output (GPIOA PIN8) status */
    uint8_t led_bit = GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13);
     
    /* If LED output set, clear it */
    if(led_bit == (uint8_t)Bit_SET)
    {
        GPIO_ResetBits(GPIOC, GPIO_Pin_13);
    }
    /* If LED output clear, set it */
    else
    {
        GPIO_SetBits(GPIOC, GPIO_Pin_13);
    }
}
 
/**********************************************************
 * USART1 interrupt request handler: on reception of a 
 * character 't', toggle LED and transmit a character 'T'
 *********************************************************/
void USART1_IRQHandler(void)
{
 static char str[15]; 
 static char str2[6]; 
	/* RXNE handler */
	

    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        /* If received 't', toggle LED and transmit 'T' */
        if((char)USART_ReceiveData(USART1) == '+' && joint_state==0 && wait_joint==1)
				{
					str[0]='+';
					joint_state=1;
				}
	      else if((char)USART_ReceiveData(USART1) == 'J' && joint_state==1 && wait_joint==1)
				{
					str[1]='J';
					joint_state=2;
				}				
				else if((char)USART_ReceiveData(USART1) == 'o' && joint_state==2 && wait_joint==1)
				{
					str[2]='o';
					joint_state=3;
				} 
				else if((char)USART_ReceiveData(USART1) == 'i' && joint_state==3 && wait_joint==1)
				{
					str[3]='i';
					joint_state=4;
				} 				
				else if((char)USART_ReceiveData(USART1) == 'n' && joint_state==4 && wait_joint==1)
				{
					str[4]='n';
					joint_state=5;
				} 
				else if((char)USART_ReceiveData(USART1) == 'A' && joint_state==5 && wait_joint==1)
				{
					str[5]='A';
					joint_state=6;
				} 			

				else if((char)USART_ReceiveData(USART1) == 'c' && joint_state==6 && wait_joint==1)
				{
					str[6]='c';
					joint_state=7;
				} 

				else if((char)USART_ReceiveData(USART1) == 'c' && joint_state==7 && wait_joint==1)
				{
					str[7]='c';
					joint_state=8;
				} 
				else if((char)USART_ReceiveData(USART1) == 'e' && joint_state==8 && wait_joint==1)
				{
					str[8]='e';
					joint_state=9;
				} 
				else if((char)USART_ReceiveData(USART1) == 'p' && joint_state==9 && wait_joint==1)
				{
					str[9]='p';
					joint_state=10;
				} 
        else if((char)USART_ReceiveData(USART1) == 't' && joint_state==10 && wait_joint==1)
				{
					str[10]='t';
					joint_state=11;
				} 
				else if((char)USART_ReceiveData(USART1) == 'e' && joint_state==11 && wait_joint==1)
				{
					str[11]='e';
					joint_state=12;
					USART_SendData(USART1,5);
				} 
				else if((char)USART_ReceiveData(USART1) == 'd' && joint_state==12 && wait_joint==1)
				{
					str[12]='d';
					joint_state=0;
					flag_connect=1;
					USART_SendData(USART1,5);
				} 
				else if ( wait_joint==1)
					joint_state=0;
				
				
				USART_ClearITPendingBit(USART1,USART_IT_RXNE);
    
		} 
    /* ------------------------------------------------------------ */
    /* Other USART1 interrupts handler can go here ...             */
//		if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
//    {
//        /* If received 't', toggle LED and transmit 'T' */
//        if((char)USART_ReceiveData(USART1) == '+' && joint_state2==0 && wait_joint==1)
//				{
//					str2[0]='+';
//					joint_state2=1;
//					
//						USART_SendData(USART1,'W');
//					  delay(10);
//						USART_SendData(USART1,'T');
//					  delay(10);
//						USART_SendData(USART1,'F');
//						delay(10);
//					
//				}
//	      else if((char)USART_ReceiveData(USART1) == 'V' && joint_state2==1 && wait_joint==1)
//				{
//					str2[1]='V';
//					joint_state2=2;
//				}				
//				else if((char)USART_ReceiveData(USART1) == 'E' && joint_state2==2 && wait_joint==1)
//				{
//					str2[2]='E';
//					joint_state2=3;
//				} 
//				else if((char)USART_ReceiveData(USART1) == 'R' && joint_state2==3 && wait_joint==1)
//				{
//					str2[3]='R';
//					joint_state2=4;
//				} 				
//				else if((char)USART_ReceiveData(USART1) == '=' && joint_state2==4 && wait_joint==1)
//				{
//					str2[4]='=';
//					joint_state2=0;
//						USART_SendData(USART1,'W');
//					  delay(10);
//						USART_SendData(USART1,'T');
//					  delay(10);
//						USART_SendData(USART1,'F');
//						delay(10);
//				} 
//				
//				USART_ClearITPendingBit(USART1,USART_IT_RXNE);
//			}
}		






/*
GPIO_InitTypeDef GPIO_InitStruct;
 
int main (void)
{
    // Enable clock for GPIOA
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
 
    // Configure PA0 as push-pull output
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStruct);
 
    while (1)
    {
        
        // Reset bit will turn on LED (because the logic is interved)
        GPIO_ResetBits(GPIOC, GPIO_Pin_13);
        delay(1000);
        // Set bit will turn off LED (because the logic is interved)
        GPIO_SetBits(GPIOC, GPIO_Pin_13);
        delay(1000);
    }
}
*/ 
// Delay function
 
void delay(unsigned int nCount)
{
    unsigned int i, j;
 
    for (i = 0; i < nCount; i++)
        for (j = 0; j < 0x2AFF; j++);
}
 
void i2c_init()
{
    // Initialization struct
    I2C_InitTypeDef I2C_InitStruct;
    GPIO_InitTypeDef GPIO_InitStruct;
 
    // Step 1: Initialize I2C
    RCC_APB1PeriphClockCmd(I2Cx_RCC, ENABLE);
    I2C_InitStruct.I2C_ClockSpeed = 100000;
    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStruct.I2C_OwnAddress1 = 0x00;
    I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2Cx, &I2C_InitStruct);
    I2C_Cmd(I2Cx, ENABLE);
 
    // Step 2: Initialize GPIO as open drain alternate function
    RCC_APB2PeriphClockCmd(I2C_GPIO_RCC, ENABLE);
    GPIO_InitStruct.GPIO_Pin = I2C_PIN_SCL | I2C_PIN_SDA;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(I2C_GPIO, &GPIO_InitStruct);
}
 
void i2c_start()
{
    // Wait until I2Cx is not busy anymore
    while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
 
    // Generate start condition
    I2C_GenerateSTART(I2Cx, ENABLE);
 
    // Wait for I2C EV5. 
    // It means that the start condition has been correctly released 
    // on the I2C bus (the bus is free, no other devices is communicating))
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
}
 
void i2c_stop()
{
    // Generate I2C stop condition
    I2C_GenerateSTOP(I2Cx, ENABLE);
    // Wait until I2C stop condition is finished
    while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF));
}
 
void i2c_address_direction(uint8_t address, uint8_t direction)
{
    // Send slave address
    I2C_Send7bitAddress(I2Cx, address, direction);
 
    // Wait for I2C EV6
    // It means that a slave acknowledges his address
    if (direction == I2C_Direction_Transmitter)
    {
        while (!I2C_CheckEvent(I2Cx,
            I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    }
    else if (direction == I2C_Direction_Receiver)
    { 
        while (!I2C_CheckEvent(I2Cx,
            I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
    }
}
 
void i2c_transmit(uint8_t byte)
{
    // Send data byte
    I2C_SendData(I2Cx, byte);
    // Wait for I2C EV8_2.
    // It means that the data has been physically shifted out and 
    // output on the bus)
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}
 
uint8_t i2c_receive_ack()
{
    // Enable ACK of received data
    I2C_AcknowledgeConfig(I2Cx, ENABLE);
    // Wait for I2C EV7
    // It means that the data has been received in I2C data register
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED));
 
    // Read and return data byte from I2C data register
    return I2C_ReceiveData(I2Cx);
}
 
uint8_t i2c_receive_nack()
{
    // Disable ACK of received data
    I2C_AcknowledgeConfig(I2Cx, DISABLE);
    // Wait for I2C EV7
    // It means that the data has been received in I2C data register
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED));
 
    // Read and return data byte from I2C data register
    return I2C_ReceiveData(I2Cx);
}
 
void i2c_write(uint8_t address, uint8_t data)
{
    i2c_start();
    i2c_address_direction(address << 1, I2C_Direction_Transmitter);
    i2c_transmit(data);
    i2c_stop();
}
 
void i2c_read(uint8_t address, uint8_t* data)
{
    i2c_start();
    i2c_address_direction(address << 1, I2C_Direction_Receiver);
    *data = i2c_receive_nack();
    i2c_stop();
}

 void read_temp_humid(void)
	 
	{
	
	//	USART_SendData(USART1,0x04);	
		i2c_write(SLAVE_ADDRESS, 0xF5);   // read humidity
	//	USART_SendData(USART1,0x05);	
	  delay(100);
	  i2c_read(SLAVE_ADDRESS, &receivedByte);
	//	USART_SendData(USART1,0x06);	
	  hum0=receivedByte;
	  i2c_read(SLAVE_ADDRESS, &receivedByte);
	//	USART_SendData(USART1,0x07);	
	  hum1=receivedByte;	
 	
	  i2c_write(SLAVE_ADDRESS, 0xF3);   // read temp.
	//	USART_SendData(USART1,0x08);	
	  delay(100);
	  i2c_read(SLAVE_ADDRESS, &receivedByte);
		//USART_SendData(USART1,0x09);	
	  temp0=receivedByte;
	  i2c_read(SLAVE_ADDRESS, &receivedByte);
	//	USART_SendData(USART1,0x0A);	
	  temp1=receivedByte;	
		
//	USART_SendData(USART1,0x44);
	 // delay(10);
		//USART_SendData(USART1,hum0);
	  //delay(10);
    //USART_SendData(USART1,hum1);
	  //delay(10);
		//USART_SendData(USART1,temp0);
	 // delay(10);
  //  USART_Se++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ndData(USART1,temp1);
	 // delay(10);
		led_toggle();
}
void read_ligth_soil(void)
{
	   uint16_t AD_value;
	    AD_value=readADC1(0);
		
	//	USART_SendData(USART1,0x55);
	//  delay(50);
//    USART_SendData(USART1,(uint8_t)((AD_value>>8)&0x00FF));	
		lig0=(uint8_t)((AD_value>>8)&0x00FF);
	//  delay(50);
//		USART_SendData(USART1,(uint8_t)(AD_value&0x00FF));
		lig1=(uint8_t)(AD_value&0x00FF);
   //  delay(1000);
		 
		 AD_value=readADC1(1);
	//	 USART_SendData(USART1,0x66);
	//  delay(50);
  //  USART_SendData(USART1,(uint8_t)((AD_value>>8)&0x00FF));	
		soil0=(uint8_t)((AD_value>>8)&0x00FF);
	//  delay(50);
	//	USART_SendData(USART1,(uint8_t)(AD_value&0x00FF));
		soil1=(uint8_t)(AD_value&0x00FF);
     delay(10);
		 
		 
	  led_toggle();
	
	
}/**/////////////*