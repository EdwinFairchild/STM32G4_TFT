#include <stm32g4xx_ll_bus.h>
#include <stm32g4xx_ll_gpio.h>
#include <stm32g4xx_ll_usart.h>
#include <stm32g4xx_ll_tim.h>
#include <stm32g4xx_ll_rcc.h>
#include <stm32g4xx_ll_system.h>
#include <stm32g4xx_ll_utils.h>
#include <stm32g4xx_ll_spi.h>
#include <stm32g4xx_ll_dma.h>
#include <stm32g4xx_ll_dmamux.h>
#include <stm32g4xx_ll_cortex.h>

//-------- COMM LIBS
#include "CL_CONFIG.h"
#include "CL_delay.h"
#include "CL_systemClockUpdate.h"
#include "CL_printMsg.h"


//-------- ST7735
#include "st7735.h"
#include "fonts.h"
#include "testimg.h"
#include "string.h"


#define LED_A1 LL_GPIO_PIN_0	

uint16_t msTICKS = 0x0000;
uint16_t msTicks = 0x0000;
uint16_t RGB565 = 0x0000;
uint8_t y_offset = 5;  // height of font character divided by 2
uint8_t x_offset = 7;  //width of 1 font character
uint8_t current = 0x00;
void tim2_init(void);
void tim2_LL_init(void);
void tim2_ch1_pwm_init(void);
void timDelaySetup(void);
void delayMS(uint32_t ms);
void uart2Init(void);
void led0Setup(void); 
void initSPI(void);
void spiSend(uint8_t * data, uint32_t len);
void spiSend16(uint16_t *data, uint32_t len);
void spi_send_dma(uint8_t *data, uint32_t len);
void initSPI_DMA(void);
void initSPI2(void);
void tftGPIO_init(void);
void loop();
void moveBox( uint8_t next);
void draw_menu(void);
void draw_menu_item_highlight(void);
void draw_menu_item(void);
#define USE_DEBUG 
#define top_to_bottom  0
#define bottom_to_top  1
#define TFA  0                                       // Top Fixed Area 0 pixel
#define BFA  50 
#define _width         128
#define _height        128


//---menu stuff
#define MENU_HIGHLIGHT_HEIGHT 20

char screen_main[8][17] =
{
	"  Menu Item 1 ",
	"  Menu Item 2 ",
	"  Menu Item 3 ",
	"  Menu Item 4 ",
	"  Menu Item 5 ",
	"  Menu Item 6 ",
	"  Menu Item 7 ",
	"  Menu Item 8 ",
};
char screen_main_highlight[8][17] =
{
	"Menu Item 1 <  ",
	"Menu Item 2 <  ",
	"Menu Item 3 <  ",
	"Menu Item 4 <  ",
	"Menu Item 5 <  ",
	"Menu Item 6 <  ",
	"Menu Item 7 <  ",
	"Menu Item 8 <  ",
};

int main(void)
{

	CL_setSysClockTo170();
#ifdef USE_DEBUG
	//uart2Init();
//	
#endif
	led0Setup();
	//initSPI2();
	initSPI_DMA();
	tftGPIO_init();
	CL_delay_init();	
	ST7735_Init();
	
	for (int i = 0; i < 11; i++)
	{
		LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_0);
		delayMS(10);
	}
	
	
	ST7735_SetRotation(4);
	ST7735_FillScreen(ST7735_BLACK);

	uint8_t scroll = 0 , y = 10;
	setScrollDefinition(TFA, 128, 1);
	uint16_t x, y2 = 0;
	//tim2_LL_init();
	
	//tim2_ch1_pwm_init();
	
	CL_printMsg_init_Default(false);
	CL_printMsg("UART init!");
	
	//ST7735_WriteString(0, 0, "System Core Clock: ", Font_11x18, ST7735_BLUE, ST7735_BLACK);
	//ST7735_printMsg("%d", SystemCoreClock);
	//tim2_init();
	ST7735_FillScreen(ST7735_BLACK);
	//ST7735_WriteString(0, 0, "Font_7x10, red on black, lorem ipsum dolor sit amet", Font_7x10, ST7735_RED, ST7735_BLACK);
	//ST7735_DrawRoundRect(0, 0, ST7735_WIDTH, 20, 5, ST7735_YELLOW);
	uint8_t delayTime = 10;
	uint8_t inc = 10;
	//draw_menu();
	//draw_menu_item_highlight(2);
	uint8_t current = 0; 
	uint8_t next = 80;
	for (;;)
	{
		
	
		moveBox( 40);
		delayMS(2000);
		moveBox( 120);
		delayMS(2000);
		
	/* PWM STUFF 
		for (int i = 0; i < 1000; i += 10)
		{			
			LL_TIM_OC_SetCompareCH1(TIM2, i);
				delayMS(10);			
		}
		for (int i = 1000; i > 0; i -= 10)
		{			
			LL_TIM_OC_SetCompareCH1(TIM2, i);
		delayMS(10);			
		}                                                                                                                                                                                                                                                                                                                                                                                                 
		*/
				
	}
}
void draw_menu(void)
{
	uint8_t item = 0;
	for (int i = 0; i <= 160 - y_offset; i += (y_offset * 4))
	{
		ST7735_WriteString(0 + x_offset, i + y_offset, &screen_main[item++], Font_7x10, ST7735_RED, ST7735_BLACK);	
		//ST7735_printMsg((0 + x_offset + 91), i + y_offset,ST7735_RED, "%d" ,item++);
			
	}
	
	//ST7735_DrawRoundRect(0, 0, ST7735_WIDTH, MENU_HIGHLIGHT_HEIGHT, 5, ST7735_YELLOW);
}
void draw_menu_item_highlight(void)
{
	
	ST7735_WriteString(0 + x_offset, current + y_offset, &screen_main_highlight[current / 20], Font_7x10, ST7735_WHITE, ST7735_BLACK);	
		//ST7735_printMsg((0 + x_offset + 91), i + y_offset,ST7735_RED, " %d   " ,item++);
	    //ST7735_DrawRoundRect(0, 0, ST7735_WIDTH, MENU_HIGHLIGHT_HEIGHT, 5, ST7735_YELLOW);
}
void draw_menu_item(void)
{
	ST7735_WriteString(0 + x_offset, current + y_offset, &screen_main[current / 20], Font_7x10, ST7735_RED, ST7735_BLACK);
	
	//ST7735_printMsg((0 + x_offset + 91), i + y_offset,ST7735_RED, " %d   " ,item++);
	//ST7735_DrawRoundRect(0, 0, ST7735_WIDTH, MENU_HIGHLIGHT_HEIGHT, 5, ST7735_YELLOW);
}
void moveBox(uint8_t next)
{
	uint8_t increment, delay;
	increment = 20;
	delay = 1;
	int i =0, j =0;
	if (current <= next)
	{
	
		/*move rectangle from current pos to next position in crements
		 *if during the move we align with a menu item then highlight it,
		 *the display has persistance, so if youy dont redrawe a section it will stay how it was last
		*/
		for (int i = current; i <= (next); i +=increment)
		{
			ST7735_DrawRoundRect(0, i, ST7735_WIDTH, MENU_HIGHLIGHT_HEIGHT, 5, ST7735_YELLOW);
			delayMS(delay);
			
			uint8_t item = 1; // not use full but for now yes
			/*redraw the menu increments ofrectangle height */
			for(j = 0 ; j <= 160 - y_offset ; j += MENU_HIGHLIGHT_HEIGHT) 
			{
				if (j == i ) //if rectangle location and menu item match highlight
				{
					
					//ST7735_WriteString(0 + x_offset, j + y_offset, "> Menu Item: ", Font_7x10, ST7735_WHITE, ST7735_BLACK);
					//ST7735_printMsg((0 + x_offset + 84), i + y_offset, ST7735_WHITE, " %d  <", item++);
					current = i;
					draw_menu_item_highlight();
					delayMS(delay);
					item++; //needs to be incrmeted for the numbers
				}
				else // otherwise redraw it normal
				{
					current = j;
					draw_menu_item();
						
					//ST7735_WriteString(0 + x_offset, j + y_offset, &screen_main[j / 20], Font_7x10, ST7735_RED, ST7735_BLACK);
					//ST7735_printMsg((0 + x_offset + 91), j + y_offset, ST7735_RED, "%d   ", item++); 
					//delayMS(10);
				}
				
			}
			ST7735_DrawRoundRect(0, i, ST7735_WIDTH, MENU_HIGHLIGHT_HEIGHT, 5, ST7735_BLACK);
			
			
		
		}
		ST7735_DrawRoundRect(0, next, ST7735_WIDTH, MENU_HIGHLIGHT_HEIGHT, 5, ST7735_YELLOW);
		current = next;
		
	
	}
	else
	{
		for (int i = current; i >= next; i -= increment)
		{
			ST7735_DrawRoundRect(0, i, ST7735_WIDTH, 20, 5, ST7735_YELLOW);
		    delayMS(delay);
			
			uint8_t item = 1;
			for (int j = 0; j <= 160 - y_offset; j += MENU_HIGHLIGHT_HEIGHT)
			{
				//ST7735_WriteString(0 + x_offset, j + y_offset, "  Menu Item: ", Font_7x10, ST7735_RED, ST7735_BLACK);
				
				if (j == i )
				{
					
					current = i;
					draw_menu_item_highlight();
					delayMS(delay);
					item++;
				}
				else
				{
					current = j;
					draw_menu_item();
					
				}
					
			}
			
			ST7735_DrawRoundRect(0, i, ST7735_WIDTH, 20, 5, ST7735_BLACK);
		}
		ST7735_DrawRoundRect(0, next, ST7735_WIDTH, 20, 5, ST7735_YELLOW);
		current = next;
	}	
}

void tim2_LL_init(void)
{
	//RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
	LL_TIM_InitTypeDef myTim2;
	LL_TIM_StructInit(&myTim2);
	myTim2.Autoreload = 1000;
	myTim2.Prescaler = 7200;
	LL_TIM_Init(TIM2, &myTim2);
	
	LL_TIM_OC_InitTypeDef OCmyTim2;
	LL_TIM_OC_StructInit(&OCmyTim2);
	OCmyTim2.CompareValue = 8000;
	TIM2->CR1 |=  TIM_CR1_URS | TIM_CR1_ARPE;
	LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH1, &OCmyTim2);
	LL_TIM_DisableUpdateEvent(TIM2);
	LL_TIM_SetUpdateSource(TIM2, LL_TIM_UPDATESOURCE_COUNTER);
	LL_TIM_EnableIT_CC1(TIM2);
	NVIC_EnableIRQ(TIM2_IRQn);
	TIM2->CR1 |= TIM_CR1_CEN;
	
}
void tim2_init(void)
{
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
	NVIC_EnableIRQ(TIM2_IRQn);		
	TIM2->ARR = 60000; //random high values
	TIM2->PSC = 7000;	
	TIM2->DIER |=  TIM_DIER_CC1IE; //enable Channel 1 interrupt
	TIM2->DIER |=  TIM_DIER_CC2IE; //enable Channel 2 interrupt
	/*Frozen - The comparison between the output compare register TIMx_CCR1 and the
	counter TIMx_CNT has no effect on the outputs.(this mode is used to generate a timing
	base).*/
	TIM2->CCMR1 |= TIM_CCMR1_OC1M_0; //frozen mode ; no action on any pins
	TIM2->CCMR1 &= ~TIM_CCMR1_CC2S;  //CC2 channel is configured as output.
	TIM2->CCMR1 &= ~TIM_CCMR1_CC1S;  //CC1 channel is configured as output.
	//TIM2->CCER |= TIM_CCER_CC1E; //enable ch1 output , but makes no difference here
	TIM2->CCR1 = 10000;   //higher value than ARR , thus interrupt should not tirgger, but it does
    TIM2->CCR2 = 2000;
	TIM2->CR1 |= TIM_CR1_CEN;
}
void TIM2_IRQHandler(void)
{
	uint32_t reg = TIM2->SR;	
	TIM2->SR = 0x00;
	bool iscc1 = ((reg & TIM_SR_CC1IF));
	bool iscc2 = ((reg & TIM_SR_CC2IF) );
	if (iscc1 )
	{
		//read new delay value, and set pins high according to bresenham
		//Motion::Core::Output::Segment::pntr_driver();
		//ST7735_printMsg("ch1 :%d : %d", TIM2->CNT, msTICKS++);
	}
	if (iscc2)
	{
		//Stepper::step_port(0);
		//ST7735_printMsg("ch2 : %d : %d", TIM2->CNT, msTICKS++);
	}	
}


void loop() {
	// Check border
	ST7735_FillScreen(ST7735_BLACK);

	for (int x = 0; x < ST7735_WIDTH; x++) {
		ST7735_DrawPixel(x, 0, ST7735_RED);
		ST7735_DrawPixel(x, ST7735_HEIGHT - 1, ST7735_RED);
	}

	for (int y = 0; y < ST7735_HEIGHT; y++) {
		ST7735_DrawPixel(0, y, ST7735_RED);
		ST7735_DrawPixel(ST7735_WIDTH - 1, y, ST7735_RED);
	}
	//HAL_GPIO_TogglePin(GPIOC,	LED0_Pin);
 //   HAL_Delay(3000);
  //  HAL_GPIO_TogglePin(GPIOC,	LED0_Pin);
    // Check fonts
    ST7735_FillScreen(ST7735_BLACK);
	ST7735_WriteString(0, 0, "Font_7x10, red on black, lorem ipsum dolor sit amet", Font_7x10, ST7735_RED, ST7735_BLACK);
	ST7735_WriteString(0, 3 * 10, "Font_11x18, green, lorem ipsum", Font_11x18, ST7735_GREEN, ST7735_BLACK);
	ST7735_WriteString(0, 3 * 10 + 3 * 18, "Font_16x26", Font_16x26, ST7735_BLUE, ST7735_BLACK);
	// HAL_Delay(2000);
	// HAL_GPIO_TogglePin(GPIOC,	LED0_Pin);
	 // Check colors
	 ST7735_FillScreen(ST7735_BLACK);
	ST7735_WriteString(0, 0, "BLACK", Font_11x18, ST7735_WHITE, ST7735_BLACK);
	//  HAL_Delay(500);
	 // HAL_GPIO_TogglePin(GPIOC,	LED0_Pin);
	  ST7735_FillScreen(ST7735_BLUE);
	ST7735_WriteString(0, 0, "BLUE", Font_11x18, ST7735_BLACK, ST7735_BLUE);
	//  HAL_Delay(500);
	//  HAL_GPIO_TogglePin(GPIOC,	LED0_Pin);
	  ST7735_FillScreen(ST7735_RED);
	ST7735_WriteString(0, 0, "RED", Font_11x18, ST7735_BLACK, ST7735_RED);
	// HAL_Delay(500);
   //  HAL_GPIO_TogglePin(GPIOC,	LED0_Pin);
     ST7735_FillScreen(ST7735_GREEN);
	ST7735_WriteString(0, 0, "GREEN", Font_11x18, ST7735_BLACK, ST7735_GREEN);
	//  HAL_Delay(500);
	//  HAL_GPIO_TogglePin(GPIOC,	LED0_Pin);
	  ST7735_FillScreen(ST7735_CYAN);
	ST7735_WriteString(0, 0, "CYAN", Font_11x18, ST7735_BLACK, ST7735_CYAN);
	//  HAL_Delay(500);
   //   HAL_GPIO_TogglePin(GPIOC,	LED0_Pin);
      ST7735_FillScreen(ST7735_MAGENTA);
	ST7735_WriteString(0, 0, "MAGENTA", Font_11x18, ST7735_BLACK, ST7735_MAGENTA);
	//  HAL_Delay(500);
	//  HAL_GPIO_TogglePin(GPIOC,	LED0_Pin);
	  ST7735_FillScreen(ST7735_YELLOW);
	ST7735_WriteString(0, 0, "YELLOW", Font_11x18, ST7735_BLACK, ST7735_YELLOW);
	// HAL_Delay(500);
   //  HAL_GPIO_TogglePin(GPIOC,	LED0_Pin);
     ST7735_FillScreen(ST7735_WHITE);
	ST7735_WriteString(0, 0, "WHITE", Font_11x18, ST7735_BLACK, ST7735_WHITE);
	//  HAL_Delay(500);
   //   HAL_GPIO_TogglePin(GPIOC,	LED0_Pin);
#ifdef ST7735_IS_128X128
       // Display test image 128x128
       ST7735_DrawImage(0, 0, ST7735_WIDTH, ST7735_HEIGHT, (uint16_t*)test_img_128x128);
	//   HAL_GPIO_TogglePin(GPIOC,	LED0_Pin);
	/*
	    // Display test image 128x128 pixel by pixel
	    for(int x = 0; x < ST7735_WIDTH; x++) {
	        for(int y = 0; y < ST7735_HEIGHT; y++) {
	            uint16_t color565 = test_img_128x128[y][x];
	            // fix endiness
	            color565 = ((color565 & 0xFF00) >> 8) | ((color565 & 0xFF) << 8);
	            ST7735_DrawPixel(x, y, color565);
	    }
	}
	*/
	  //  HAL_Delay(15000);
	    //HAL_GPIO_TogglePin(GPIOC,	LED0_Pin);
#endif // ST7735_IS_128X128

}

void tftGPIO_init(void)
{
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_5 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7);
	LL_GPIO_InitTypeDef myGPIO;
	LL_GPIO_StructInit(&myGPIO);
	myGPIO.Mode			= LL_GPIO_MODE_OUTPUT;
	myGPIO.OutputType	= LL_GPIO_OUTPUT_PUSHPULL;
	myGPIO.Pin			=  LL_GPIO_PIN_5 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7;
	myGPIO.Speed		= LL_GPIO_SPEED_LOW;
	LL_GPIO_Init(GPIOB, &myGPIO);
	

}
void spi_send_dma(uint8_t *data, uint32_t len)
{
	//CL_printMsg("%d |%d | %d | %c ", &myString , data, &data, *data);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, (uint32_t)len);
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)data);
	DMA1_Channel1->CCR |= (DMA_CCR_EN);	
}
void spiSend(uint8_t *data, uint32_t len)
{
	
	uint8_t volatile *spidr = ((__IO uint8_t *)&SPI1->DR);
	while (len > 0)
	{
		while (!(SPI1->SR&SPI_SR_TXE)){
			;}
		*spidr = *data++;
		len--;
	}	
	while ((SPI1->SR&SPI_SR_BSY)) {
		;
	}
	
	
}

void spiSend16(uint16_t *data, uint32_t len)
{	
	uint16_t volatile *spidr = ((__IO uint16_t *)&SPI1->DR);

	while (len > 0)
	{
		while (!(SPI1->SR&SPI_SR_TXE)) ;
		len--;
		//*spidr = (uint8_t)*data++;
		LL_SPI_TransmitData16(SPI1,  (uint8_t)*data++);
	}	
	while ((SPI1->SR&SPI_SR_BSY)) ;		
}
void tim2_ch1_pwm_init(void)
{

	//enable GPIOA and TIMER clocks
	LL_AHB2_GRP1_EnableClock(RCC_AHB2ENR_GPIOAEN); 
	LL_APB1_GRP1_EnableClock(RCC_APB1ENR1_TIM2EN);
	
	//setup GPIO for alt-func 1 
	LL_GPIO_InitTypeDef timGPIO;
	LL_GPIO_StructInit(&timGPIO);
	timGPIO.Mode =	LL_GPIO_MODE_ALTERNATE;
	timGPIO.Alternate =	LL_GPIO_AF_1;
	timGPIO.Speed  =	LL_GPIO_SPEED_HIGH;
	timGPIO.Pin     =	LL_GPIO_PIN_0;
	LL_GPIO_Init(GPIOA, &timGPIO); 
	
	//setup tim2 ch1 OC 
	LL_TIM_InitTypeDef tim2ch1;
	LL_TIM_StructInit(&tim2ch1);
	tim2ch1.Autoreload = 1000;
	tim2ch1.Prescaler  = 170;
	//LL_TIM_EnableARRPreload(TIM2);
	
	
	LL_TIM_OC_InitTypeDef tim2ch1_oc;
	LL_TIM_OC_StructInit(&tim2ch1_oc);
	tim2ch1_oc.CompareValue = 500; 
	tim2ch1_oc.OCIdleState  = LL_TIM_OCIDLESTATE_LOW;
	tim2ch1_oc.OCMode		= LL_TIM_OCMODE_PWM1;
	tim2ch1_oc.OCPolarity	= LL_TIM_OCPOLARITY_HIGH;
	tim2ch1_oc.OCState		= LL_TIM_OCSTATE_ENABLE;
	
	
	LL_TIM_Init(TIM2, &tim2ch1);
	LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH1, &tim2ch1_oc);
	
	LL_TIM_EnableCounter(TIM2);	

#ifdef USE__REG
	//enable clocks GPIOA TIM2
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
	
	//setup GPIO A1 ALT FUNCTION 1
	GPIOA->MODER &= ~(GPIO_MODER_MODE0);
	GPIOA->MODER |= GPIO_MODER_MODE0_1;
	GPIOA->AFR[0] = GPIO_AFRL_AFSEL0_0;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR0;
	
	//setup TIM2 CH1 OC
	TIM2->PSC = 170;
	TIM2->ARR =  1000;
	TIM2->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
	TIM2->CCR1 = 500;
	TIM2->CCER |= TIM_CCER_CC1E;	
	TIM2->CR1  |= TIM_CR1_CEN;
	
	
#endif
	
}




void led0Setup(void)
{
 
	
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
	
	LL_GPIO_InitTypeDef myGPIO;
	LL_GPIO_StructInit(&myGPIO);
	myGPIO.Mode			= LL_GPIO_MODE_OUTPUT;
	myGPIO.OutputType	= LL_GPIO_OUTPUT_PUSHPULL;
	myGPIO.Pin			= LED_A1;
	myGPIO.Speed		= LL_GPIO_SPEED_HIGH;
	LL_GPIO_Init(GPIOA, &myGPIO);
}





void uart2Init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	//gpio
	GPIOA->MODER &= ~(GPIO_MODER_MODE9);
	GPIOA->MODER |= GPIO_MODER_MODE9_1;   // 0b10 = AF mode
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9_1;
	GPIOA->AFR[1] |= 0x07 << GPIO_AFRH_AFSEL9_Pos;
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
	LL_USART_InitTypeDef myUART;
	LL_USART_StructInit(&myUART);
	LL_USART_Init(USART1, &myUART);
	
}


void initSPI(void)
{
	
	// Config SPI1 on GPIOA ALT FUNC 5 
	// enable clocks	
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);

	LL_GPIO_InitTypeDef spiPort;
	LL_GPIO_StructInit(&spiPort);
	
	spiPort.Alternate	= LL_GPIO_AF_5;
	spiPort.Mode		= LL_GPIO_MODE_ALTERNATE;
	spiPort.Pin			= LL_GPIO_PIN_5;
	spiPort.Speed		= LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOA, &spiPort);
	GPIOA->AFR[0] = 5;
	
	spiPort.Pin = LL_GPIO_PIN_7;
	LL_GPIO_Init(GPIOA, &spiPort);

	// config SPI
	LL_SPI_InitTypeDef mySPI;
	LL_SPI_StructInit(&mySPI); 
	

	
	
	mySPI.BaudRate			= LL_SPI_BAUDRATEPRESCALER_DIV2;  //40Mbits/s
	mySPI.Mode				= LL_SPI_MODE_MASTER;
	mySPI.TransferDirection = LL_SPI_HALF_DUPLEX_TX;
	mySPI.NSS				= LL_SPI_NSS_SOFT;
	mySPI.DataWidth			= LL_SPI_DATAWIDTH_8BIT;
	LL_SPI_Init(SPI1, &mySPI);
	LL_SPI_Enable(SPI1);	
	
	
}
void initSPI_DMA(void) //non circular mode
{			
	//clocks	
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
	
	
	LL_GPIO_InitTypeDef myGPIO;
	LL_GPIO_StructInit(&myGPIO);
	myGPIO.Pin       = LL_GPIO_PIN_5 | LL_GPIO_PIN_7;
	myGPIO.Speed     = LL_GPIO_SPEED_FREQ_HIGH;
	myGPIO.Mode      = LL_GPIO_MODE_ALTERNATE;
	myGPIO.Alternate = LL_GPIO_AF_5;
	LL_GPIO_Init(GPIOA, &myGPIO);
	
	LL_SPI_InitTypeDef mySPI;
	LL_SPI_StructInit(&mySPI);
	mySPI.Mode				= LL_SPI_MODE_MASTER;
	mySPI.TransferDirection	= LL_SPI_HALF_DUPLEX_TX;
	mySPI.BaudRate          = LL_SPI_BAUDRATEPRESCALER_DIV16;
	mySPI.NSS				= LL_SPI_NSS_SOFT;
	mySPI.DataWidth			= LL_SPI_DATAWIDTH_8BIT;
	
	LL_SPI_Init(SPI1, &mySPI);
	LL_SPI_EnableDMAReq_TX(SPI1);
	LL_SPI_Enable(SPI1);
	
	//DMA DMAMUX
	LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_1, LL_DMAMUX_REQ_SPI1_TX);
	LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_LOW);
	LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);
	LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);
	LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_BYTE);
	LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_BYTE);
	//LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1,(uint32_t)&myString);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, (uint32_t)11);
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&(SPI1->DR));
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
	
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}
void DMA1_Channel1_IRQHandler(void)
{
	//this be sort of like circular mode since i restart 
	//the dma once its done sending 
	DMA1_Channel1->CCR &= ~(DMA_CCR_EN); 
	DMA1->IFCR |= DMA_IFCR_CTCIF1;
	//DMA1_Channel1->CNDTR = 11;
	//DMA1_Channel1->CCR |= (DMA_CCR_EN);

}
void initSPI2(void)
{
	
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
	//gpio set to alt function mode 
	GPIOA->MODER &=  ~(GPIO_MODER_MODE5 | GPIO_MODER_MODE7);
	GPIOA->MODER |=  (GPIO_MODER_MODE5_1 | GPIO_MODER_MODE7_1);
	LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_5, LL_GPIO_AF_5);
	LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_7, LL_GPIO_AF_5);
	
	/*gpio set which alt function
		nothing to set in the alternate function registers because 
		by default they are set to 00 and 00 is the value we need to
		set it to SPI*/
	
	
	SPI1->CR1 = SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE | SPI_CR1_SSM | SPI_CR1_MSTR  | SPI_CR1_BR_1 ;
	SPI1->CR2 = SPI_CR2_SSOE; 
	SPI1->CR1 |= SPI_CR1_SPE;

	
		
}



