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
	//GPIOB 1
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitTypeDef gpio_B;
	gpio_B.Pin = GPIO_PIN_1;
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

void write_number(int num) {
	int seg[7][2] = {{GPIOB, GPIO_PIN_1}, //A
					 {GPIOA, GPIO_PIN_6}, //B
					 {GPIOA, GPIO_PIN_5}, //C
					 {GPIOA, GPIO_PIN_3}, //D
					 {GPIOA, GPIO_PIN_1}, //E
					 {GPIOA, GPIO_PIN_7}, //F
					 {GPIOA, GPIO_PIN_2}  //G
	};
	char digit[10][9] = {
	           {1,1,1,1,1,1,0,0,'0'},//0
	           {0,1,1,0,0,0,0,0,'1'},//1
	           {1,1,0,1,1,0,1,0,'2'},//2
	           {1,1,1,1,0,0,1,0,'3'},//3
	           {0,1,1,0,0,1,1,0,'4'},//4
	           {1,0,1,1,0,1,1,0,'5'},//5
	           {1,0,1,1,1,1,1,0,'6'},//6
	           {1,1,1,0,0,0,0,0,'7'},//7
	           {1,1,1,1,1,1,1,0,'8'},//8
	           {1,1,1,1,0,1,1,0,'9'} //9
	};
	if (num > 9 || num < 0) return;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1 , 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);
	HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_3, 0);
	HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_5, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0);
	for(int i = 0; i < 7; i++) {
		HAL_GPIO_WritePin(seg[i][0], seg[i][1], digit[num][i]);
	}
}
int main()
{
	HAL_Init();
	init_for_sevensegment();
	button_init();
	int debounce_count = 0;
	int sw = 0;
	int num = 0;
	while (1) {
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)){
			debounce_count ++;
			if (debounce_count > 999) {
				debounce_count = 200;
			}
			sw = 0;
		} else if(debounce_count >= 200) {
			debounce_count = 0;
			sw = 1;
		}
		if (sw == 1) {
			num++;
			if (num == 10) {
				num = 0;
			}
			sw = 0;
		}
		write_number(num);
	}
}
