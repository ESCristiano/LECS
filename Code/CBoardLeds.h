#ifndef __MAIN_H__
#define __MAIN_H__

class CLeds{
		public:
			CLeds()
			{
				// GPIO structure declaration
				GPIO_InitTypeDef GPIO_InitStruct;
				// Enabling GPIO peripheral clock
				RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
				// GPIO peripheral properties specification
				GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15; // LED3 GPIO pin
				GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; // output
				GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; // clock speed
				GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; // push/pull 
				GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; // pullup/pulldown resistors inactive
				// Setting GPIO peripheral corresponding bits
				GPIO_Init(GPIOD, &GPIO_InitStruct);
			};
			~CLeds(){};
			void setGreen() {GPIO_WriteBit(GPIOD, GPIO_Pin_12, Bit_SET); return;}
			void setOrange() {GPIO_WriteBit(GPIOD, GPIO_Pin_13, Bit_SET);return;}
			void setRed() {GPIO_WriteBit(GPIOD, GPIO_Pin_14, Bit_SET);return;}
			void setBlue() {GPIO_WriteBit(GPIOD, GPIO_Pin_15, Bit_SET);return;}
			void resetGreen() {GPIO_WriteBit(GPIOD, GPIO_Pin_12, Bit_RESET); return;}
			void resetOrange() {GPIO_WriteBit(GPIOD, GPIO_Pin_13, Bit_RESET);return;}
			void resetRed() {GPIO_WriteBit(GPIOD, GPIO_Pin_14, Bit_RESET);return;}
			void resetBlue() {GPIO_WriteBit(GPIOD, GPIO_Pin_15, Bit_RESET);return;}
};

#endif
