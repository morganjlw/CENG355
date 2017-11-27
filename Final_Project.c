// ----------------------------------------------------------------------------
// School: University of Victoria, Canada.
// Course: CENG 355 "Microprocessor-Based Systems".
// By: Tanner Zinck and Morgan Williams (2016)
// ----------------------------------------------------------------------------

#include <stdio.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"
#include "stm32f0-stdperiph/stm32f0xx_spi.h" // enable SPI libraries

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

/*----Defines----*/

//TIM2
#define myTIM2_PRESCALER ((uint16_t)0x0000)//no prescaler
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)//max value before overflow

//Delay
#define DELAY_PSC_1KHZ (47999); //scale to 1ms
#define DELAY_PERIOD (100) //100 ms


//POT
#define ADC_MAX_VALUE ((float)0xFFF) //due to 12 bit resoultion
#define DAC_MAX_VALUE ((float)0xFFF) //due to 12 bit resolution
#define MAX_POT_RES (5000) //5K POT

/*----Prototypes----*/

//GPIO
void myGeneralGPIOA_Init(void);
void mySPIGPIOB_Init(void);

//Timers
void myTIM2_Init(void);
void myTIM3_Init(void);

void mySleep(uint32_t);

//Interrupts
void TIM2_IRQHandler(void);
void myEXTI_Init(void);

//ADC & DAC
void myDAC_Init(void);
void myADC_Init (void);
void POT_Value_Analysis(void);

//LCD
void mySPI1_Init(void);
void myLCD_Init(void);
void sendCommand(uint8_t);
void sendData(uint8_t);
void send(uint8_t);
void Writes_Values(float, float);

/*		Global variables		*/
float gbPOT_Resistance = 0.0; //POT resistance (regulated value)
float gbfrequency = 0.0; //Frequency
uint8_t data = 0x00; //initialize to zero

/*		Method		*/
int main(int argc, char* argv[]){

	//Hot Topic rawr initialization
	trace_printf("Prepare to be cucccc XD'd\n");

	//Initialize GPIO, DAC, and ADC
    myGeneralGPIOA_Init(); // Initialize GPIO port A
    mySPIGPIOB_Init(); // Initialize SPI related GPIO port B
    mySPI1_Init(); //Initialize SPI1
	myADC_Init (); //Initialize ADC
    myDAC_Init (); //Initialize DAC

	//Initialize Timers & Interrupts
	myTIM2_Init(); // Initialize timer TIM2
    myTIM3_Init(); //Initialize timer TIM3
    myEXTI_Init(); // Initialize EXTI

    myLCD_Init(); //Initialize LCD

    while (1){
			
	//measures POT and transfers value to DAC for output 
	//regulates POT value read from ADC conversion
	//transfers POT value to DAC					
    POT_Value_Analysis();
	mySleep(250); //delay b/w LCD writes
	Writes_Values (gbPOT_Resistance, gbfrequency); //update LCD

    }

    return 0;
}

void Writes_Values(float resistance, float frequency){

    //Write "F" (cursor auto-increments)
    sendData('F');

    //Write ":"
    sendData(':');

    //Update Frequency measurement
    uint8_t fones, ftens, fhundreds, fthousands;
    fones = (uint32_t)frequency % 10;
    ftens = ((uint32_t)frequency/10) % 10;
    fhundreds = ((uint32_t)frequency/100) % 10;
    fthousands = ((uint32_t)frequency/1000) % 10;

    sendData(fthousands + 48);
    sendData(fhundreds + 48);
    sendData(ftens + 48);
    sendData(fones + 48);

    //Write "H"
    sendData('H');

    //Write "z"
    sendData('z');

    //Write "R"
    sendData('R');

    //Write ":"
    sendData(':');

    //Update POT measurement
    uint8_t rones, rtens, rhundreds, rthousands;
    rones = (uint32_t)resistance % 10;
    rtens = ((uint32_t)resistance/10) % 10;
    rhundreds = ((uint32_t)resistance/100) % 10;
    rthousands = ((uint32_t)resistance/1000) % 10;

    sendData(rthousands + 48);
    sendData(rhundreds + 48);
    sendData(rtens + 48);
    sendData(rones + 48);

    //Write "ohm symbol" = "234"
  
    sendData('O');
    sendData('h');
}

void POT_Value_Analysis(){

    uint32_t ADC_POT_Value;

    //start ADC conversion
    ADC1->CR |= ADC_CR_ADSTART;

    // wait for new ADC value
    while((ADC1->ISR & ADC_ISR_EOC)== 0);

    ADC1->ISR &= ~(ADC_ISR_EOC); //clear done conversion flag

	//transfer 12-bit ADC value to DAC_OUT1 
	//via DAC_DHR12R1 register
    //since, in "single" mode, this register 
	//automatically transfer this to output...
    DAC->DHR12R1 = ADC1->DR;

	//Mask ADC value with data mask
    ADC_POT_Value = ((ADC1->DR)& ADC_DR_DATA);

    //regulate ADC value with respect to POT parameters i.e. (5K)
    gbPOT_Resistance = (((float)ADC_POT_Value)/ADC_MAX_VALUE)*MAX_POT_RES;
}

void myGeneralGPIOA_Init(){
    //PA0 -> ADC input (Mode: Analog)
    //PA1 -> PWM Signal input (Mode: Input)
    //PA4 -> DAC output (Mode: Analog)

    // Enable clock for GPIOA peripheral
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    // Configure PA1 as input, writing "00" to bits 2 and 1.
    GPIOA-> MODER &= ~(GPIO_MODER_MODER1);

    //Configure DAC and ADC GPIO pins: PA4 and PA0 as analog mode pin,
    GPIOA-> MODER |= (GPIO_MODER_MODER0 | GPIO_MODER_MODER4);

    // Ensure no pull-up/pull-down for PA0, PA1, and PA4.
    GPIOA-> PUPDR &= ~(GPIO_PUPDR_PUPDR0 | GPIO_PUPDR_PUPDR1 | GPIO_PUPDR_PUPDR4);

}

void mySPIGPIOB_Init(){

    //PB3 -> SPI1_SCK (Mode: AF)
    //PB5 -> SPI1_MOSI (Mode: AF)
    //PB7 -> SPI1_LCK (Mode: General Output)

    // Enable clock for GPIOB peripheral
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    // Configure PB3 and PB5 as AF.
    GPIOB-> MODER &= ~(GPIO_MODER_MODER3_0 | GPIO_MODER_MODER5_0);
    GPIOB-> MODER |= (GPIO_MODER_MODER3_1 | GPIO_MODER_MODER5_1); 

    // Configure PB7 as General Output.
    GPIOB-> MODER &= ~(GPIO_MODER_MODER7_1); 
    GPIOB-> MODER |= GPIO_MODER_MODER7_0; 

    // Ensure no pull-up/pull-down for PB3, PB5, and PB7.
    GPIOB-> PUPDR &= ~(GPIO_PUPDR_PUPDR3 | GPIO_PUPDR_PUPDR5 | GPIO_PUPDR_PUPDR7);
}

void myDAC_Init(){

	//Enable DAC clock
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;

    //Defaults to single mode where there is no trigger required and
    //when data is written into the DHRx, 
    DAC->CR |= DAC_CR_EN1; // enabling DAC channel 1 (DAC_OUT1)
}

void myTIM2_Init(){

    // Enable clock for TIM2 peripheral
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    /* Configure TIM2: buffer auto-reload, count up, stop on overflow,
     * enable update events, interrupt on overflow only */
    TIM2->CR1 = ((uint16_t)0x008C);

    /* Set clock prescaler value */
    TIM2->PSC = myTIM2_PRESCALER; //set to 0...

    /* Set auto-reloaded delay */
    TIM2->ARR = myTIM2_PERIOD;

    /* Update timer registers */
    TIM2->EGR |= TIM_EGR_UG; 

    /* Assign TIM2 interrupt priority = 0 in NVIC */
    NVIC_SetPriority(TIM2_IRQn, 0);

    /* Enable TIM2 interrupts in NVIC */
    NVIC_EnableIRQ(TIM2_IRQn);

    /* Enable update interrupt generation */
    TIM2->DIER |= TIM_DIER_UIE;
}

void myTIM3_Init(){
    //Enable TIM3 peripheral clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    // Configure TIM3: buffer auto-reload, count up, stop on overflow,
    //enable update events, interrupt on overflow only
    TIM3->CR1 = 0x8C;

    //set the prescaler
    TIM3->PSC = DELAY_PSC_1KHZ;

    TIM3->ARR = DELAY_PERIOD;

    // Enable update interrupt generation
    TIM3->DIER |= TIM_DIER_UIE;

	//enable udpate events
    TIM3->EGR |= 0x0001; 

}

void myEXTI_Init(){

    /* Map EXTI1 line to PA1 */
    SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI1_PA;

    /* EXTI1 line interrupts: set rising-edge trigger */
    EXTI->RTSR |= EXTI_RTSR_TR1;

    /* Unmask interrupts from EXTI1 line */
    //writing 1 into pos. MRx unmasks interrupt
    EXTI->IMR |= EXTI_IMR_MR1;

    /* Assign EXTI1 interrupt priority = 0 in NVIC */
    NVIC_SetPriority(EXTI0_1_IRQn,0);

    /* Enable EXTI1 interrupts in NVIC */
    NVIC_EnableIRQ(EXTI0_1_IRQn);
}

void TIM2_IRQHandler(){
    /* Check if update interrupt flag is indeed set */
    if ((TIM2->SR & TIM_SR_UIF) != 0)
    {
        trace_printf("\n*** Overflow! ***\n");


        /* Clear update interrupt flag */
        TIM2->SR &= ~(TIM_SR_UIF);


        /* Restart stopped timer */
        TIM2->CR1 |= TIM_CR1_CEN;
    }
}

 void mySPI1_Init(){

    // enable SPI clock by writing "1" into bit 12
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    // Initialize and configure SPI1 CR1 and CR2 registers
    SPI_InitTypeDef SPI_InitStructInfo;
    SPI_InitTypeDef* SPI_InitStruct = &SPI_InitStructInfo;

    SPI_InitStruct->SPI_Direction = SPI_Direction_1Line_Tx;
    SPI_InitStruct->SPI_Mode = SPI_Mode_Master;
    SPI_InitStruct->SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStruct->SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStruct->SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStruct->SPI_NSS = SPI_NSS_Soft;
    SPI_InitStruct->SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256; ;
    SPI_InitStruct->SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStruct->SPI_CRCPolynomial = 7;

    SPI_Init(SPI1, SPI_InitStruct);

    SPI_Cmd(SPI1, ENABLE); // Enable SPI1


}

 void send(uint8_t word) {
      //Force LCK = 0
     GPIOB->BSRR = GPIO_BSRR_BR_7;

     //wait for SPI (check  busy flag for SPI)
     while(SPI1->SR & SPI_SR_BSY);

      //send 8 bit character via SPI
      SPI_SendData8(SPI1, word);

      //wait for SPI
      while(SPI1->SR & SPI_SR_BSY);

      //Force LCK = 1
      GPIOB->BSRR = GPIO_BSRR_BS_7;

  }

 void sendData(uint8_t dataWord) {
    //dataWord = dataWord + 48;
     uint8_t high = (dataWord & 0xF0)>> 4;
     uint8_t low = (dataWord & 0x0F);

	 //split 8 bit word to high and low
	 //4 bit words
     send(high | 0x40);
     send(high | 0xC0);
     send(high | 0x40);
     mySleep(2); //delay
     send(low | 0x40);
     send(low | 0xC0);
     send(low | 0x40);
 }

 void sendCommand(uint8_t command) {
     uint8_t high = ((command >> 4) & 0x0F);
     uint8_t low = (command & 0x0F);
     send(high);
     send(high | 0x80);
     send(high);
     mySleep(2);
     send(low);
     send(low | 0x80);
     send(low);
 }

 void myLCD_Init(){
     
	 //set 4 bit mode explicitly
     sendCommand(0x02);
     mySleep(2);//delay

     //set 4 bit and 2 row mode
	 //doesn't require delay
	 sendCommand(0x28);

      //turn on the display and turn off the cursor
      sendCommand(0x0C);
      mySleep(2); //delay

      //set cursor move dir to increment, and no disp. shift
      sendCommand(0x06);

      //clear display and send the cursor home
      sendCommand(0x01);
  }

 void mySleep(uint32_t sleepLength){

     //reset counter register to 0
     TIM3-> CNT = (uint32_t)0x0;

	 //set auto reload register set to delay length...
     TIM3->ARR = sleepLength;

	 //enable update events
     TIM3->EGR |= 0x0001;

     //start the timer
     TIM3->CR1 |= TIM_CR1_CEN;

     //wait for the interrupt update flag to trigger...
     //waiting for timer to finish, takes length of delay...
     while((TIM3->SR & TIM_SR_UIF) == 0);

     TIM3->SR &= ~(TIM_SR_UIF); //reset the interrupt flag

     TIM3->CR1 &= ~(TIM_CR1_CEN);// stop timer

     TIM3-> CNT = (uint32_t)0x0;//clear timer
 }

void myADC_Init(){

    // enables ADC clock
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    //start calibration
    ADC1->CR = ADC_CR_ADCAL;

	//waiting until ADCAL = 0...
    while (ADC1->CR == ADC_CR_ADCAL); 

    //enable continuous conversion and overrun modes.
    ADC1->CFGR1 |= (ADC_CFGR1_OVRMOD | ADC_CFGR1_CONT);

    // configured to ADC input to PA0
    ADC1->CHSELR = ADC_CHSELR_CHSEL0;

    // Enable ADC
    ADC1->CR |= ADC_CR_ADEN;

    //wait for ADC to calibrate...
    while((ADC1->ISR & ADC_ISR_EOC) != 0);

    //clear conversion flag
    ADC1->ISR &= ~(ADC_ISR_EOC);
}

void EXTI0_1_IRQHandler(){

    if ((EXTI->PR & EXTI_PR_PR1) != 0){
		
		//checking if timer is enabled
        uint16_t TimerStatus = (TIM2->CR1 & TIM_CR1_CEN); 

        if (TimerStatus){//This is the second edge:
		
            TIM2->CR1 &= ~(TIM_CR1_CEN); //Stop timer
            uint32_t count = TIM2->CNT;// Read count register

            // Calculate signal period and frequency
            gbfrequency = ((float)SystemCoreClock)/((float)count); //Hz

        } else { //This is first edge:

            TIM2->CNT &= ((uint32_t)0x00000000);//Clear count register
            TIM2->CR1 |= TIM_CR1_CEN;//Restart timer
        }

		//Clear EXTI1 interrupt pending flag (EXTI->PR).
        EXTI->PR |= EXTI_PR_PR1; 

    }

    }

#pragma GCC diagnostic pop
