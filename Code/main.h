#ifndef __MAIN_H__
#define __MAIN_H__



class CLeds{
		public:
			CLeds(){};
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
CLeds leds;

#endif
