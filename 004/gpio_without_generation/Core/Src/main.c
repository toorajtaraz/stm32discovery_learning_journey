#include "stm32f3xx_hal.h"
#include "main.h"

void init_for_sevensegment()
{
	//GPIOA 1 2 3 5 6 7
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitTypeDef gpio_A;
	gpio_A.Pin = GPIO_PIN_1 | GPIO_PIN_2| GPIO_PIN_3| GPIO_PIN_5| GPIO_PIN_6| GPIO_PIN_7;
	gpio_A.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOA, &gpio_A);
	//GPIOB 1 2
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitTypeDef gpio_B;
	gpio_B.Pin = GPIO_PIN_1 | GPIO_PIN_2;
	gpio_B.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOB, &gpio_B);
}

void button_init()
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void split_number(int num, int digits[4]) {
    if(num > 9999) {
        return;
    }
    for(int i = 0; num > 0 && i < 4; i++) {
        int digit = num % 10;
        digits[3 - i] = digit;
        num /= 10;
    }
}

void to_bcd(int digit, int b[4]) {
   int bcd = 0;
   if (digit > 0) {
      bcd |= (digit % 10) << (0 << 2);
   }
   b[3] = (bcd & 8) > 0 ? 1 : 0;
   b[2] = (bcd & 4) > 0 ? 1 : 0;
   b[1] = (bcd & 2) > 0 ? 1 : 0;
   b[0] = (bcd & 1) > 0 ? 1 : 0;
}

void off_all() {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1 , 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 1);
	HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_3, 1);
	HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_5, 1);
}

void write_number(int num) {
	if (num > 9999 || num < 0) return;
	int digits[4] = {0};
	int b[4] = {0};
	int controllers[4][2] = {{GPIOA, GPIO_PIN_1}, {GPIOA, GPIO_PIN_2},
							{GPIOA, GPIO_PIN_5}, {GPIOA, GPIO_PIN_3}};
	int ic[4][2] = {{GPIOA, GPIO_PIN_6}, {GPIOA, GPIO_PIN_7},
				   {GPIOB, GPIO_PIN_1}, {GPIOB, GPIO_PIN_2}};
	split_number(num, digits);
	for(int i = 0; i < 4; i++) {
		off_all();
		to_bcd(digits[i], b);
		for(int j = 0; j < 4; j++) {
			HAL_GPIO_WritePin(ic[j][0],  ic[j][1], b[j]);
		}
		HAL_GPIO_WritePin(controllers[i][0], controllers[i][1], 0);
		HAL_Delay(3);
	}
}

int main()
{
	HAL_SYSTICK_Config(SystemCoreClock / (1000U / 1));
	HAL_Init();
	init_for_sevensegment();
	button_init();
	while(1) {
		write_number(2257);
	}
}
