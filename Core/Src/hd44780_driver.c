#include "hd44780_driver.h"
#include "cmsis_os.h"
#include "main.h"

static void lcd_init();
static void lcd_send(uint8_t byte, dat_or_comm dc);
static void lcd_set_4bit_mode(void);
static void lcd_set_state(lcd_state state, cursor_state cur_state, cursor_mode cur_mode);
static void lcd_clear(void);
//static void lcd_out(char * txt);
static void lcd_set_xy(uint8_t x, uint8_t y);
//static void lcd_set_user_char(uint8_t char_num, uint8_t * char_data);
static void lcd_delay(void);

static void lcd_delay(void) {
	volatile uint32_t tmpvar;
	__DSB();
	__DMB();
//	__DSB();
//	__ISB();
	for (tmpvar=1000000; tmpvar!=0; tmpvar--) {
		tmpvar++;
		tmpvar--;
	}
	//osDelay(10);
	__DMB();
	__DSB();
	__ISB();
}

static void lcd_init() {
//	LCD_PORT->CRH |= LCD_PORT_CRH_S;
//	LCD_PORT->CRL |= LCD_PORT_CRL_S;
//	LCD_PORT->CRH &= ~(LCD_PORT_CRH_C);
//	LCD_PORT->CRL &= ~(LCD_PORT_CRL_C);
	lcd_set_4bit_mode();
	lcd_set_state(LCD_ENABLE,CURSOR_ENABLE,NO_BLINK);
	lcd_clear();
	lcd_send(0x06,COMMAND);
}

//static void lcd_set_user_char(uint8_t char_num, uint8_t * char_data) {
//	uint8_t i;
//	lcd_send(((1<<6) | (char_num * 8) ), COMMAND);
//	for (i=0;i<=7;i++) {
//		lcd_send(char_data[i],DATA);
//	}
//	lcd_send((1<<7), COMMAND);
//}

static void lcd_set_xy(uint8_t x, uint8_t y)  {
	if (y==0) {
		lcd_send( ((1<<7) | (x)),COMMAND);
	} else {
		lcd_send( ((3<<6) | (x)),COMMAND);
	}
}


//static void lcd_out(char * txt) {
//	while(*txt) {
//		lcd_send(*txt,DATA);
//		txt++;
//	}
//}

static void lcd_clear(void) {
	lcd_send(0x01,COMMAND);
}

static void lcd_set_state(lcd_state state, cursor_state cur_state, cursor_mode cur_mode)  {
	if (state==LCD_DISABLE)  {
		lcd_send(0x08,COMMAND);
	} else {
		if (cur_state==CURSOR_DISABLE) {
			if (cur_mode==NO_BLINK)  {
				lcd_send(0x0C,COMMAND);
			} else {
				lcd_send(0x0D,COMMAND);
			}
		} else  {
			if (cur_mode==NO_BLINK)  {
				lcd_send(0x0E,COMMAND);
			} else {
				lcd_send(0x0F,COMMAND);
			}
		}
	}
}

static void lcd_set_4bit_mode(void) {

	LCD_PORT->BSRR=(LCD_DB7_BC | LCD_DB6_BC | LCD_DB5_BC | LCD_DB4_BC | LCD_CD_BC | LCD_EN_BC);
	LCD_PORT->BSRR=(LCD_DB5_BS);

	LCD_PORT->BSRR=LCD_EN_BS;
	lcd_delay();
	LCD_PORT->BSRR=LCD_EN_BC;
	lcd_delay();

	LCD_PORT->BSRR=(LCD_DB7_BC | LCD_DB6_BC | LCD_DB5_BC | LCD_DB4_BC | LCD_CD_BC | LCD_EN_BC);
	LCD_PORT->BSRR=(LCD_DB5_BS);

	LCD_PORT->BSRR=LCD_EN_BS;
	lcd_delay();
	LCD_PORT->BSRR=LCD_EN_BC;
	lcd_delay();

	LCD_PORT->BSRR=(LCD_DB7_BC | LCD_DB6_BC | LCD_DB5_BC | LCD_DB4_BC | LCD_CD_BC | LCD_EN_BC);
	LCD_PORT->BSRR=(LCD_DB7_BS);

	LCD_PORT->BSRR=LCD_EN_BS;
	lcd_delay();
	LCD_PORT->BSRR=LCD_EN_BC;
	lcd_delay();

}

static void lcd_send(uint8_t byte, dat_or_comm dc)  {

	LCD_PORT->BSRR=(LCD_DB7_BC | LCD_DB6_BC | LCD_DB5_BC | LCD_DB4_BC | LCD_CD_BC | LCD_EN_BC);

	if (dc) {
		LCD_PORT->BSRR=LCD_CD_BS;
	}

	if (byte & 0x10) {
		LCD_PORT->BSRR=LCD_DB4_BS;
	}
	if (byte & 0x20) {
		LCD_PORT->BSRR=LCD_DB5_BS;
	}
	if (byte & 0x40) {
		LCD_PORT->BSRR=LCD_DB6_BS;
	}
	if (byte & 0x80) {
		LCD_PORT->BSRR=LCD_DB7_BS;
	}

	LCD_PORT->BSRR=LCD_EN_BS;
	lcd_delay();
	LCD_PORT->BSRR=LCD_EN_BC;
	lcd_delay();


	LCD_PORT->BSRR=(LCD_DB7_BC | LCD_DB6_BC | LCD_DB5_BC | LCD_DB4_BC );

	if (byte & 0x01) {
		LCD_PORT->BSRR=LCD_DB4_BS;
	}
	if (byte & 0x02) {
		LCD_PORT->BSRR=LCD_DB5_BS;
	}
	if (byte & 0x04) {
		LCD_PORT->BSRR=LCD_DB6_BS;
	}
	if (byte & 0x08) {
		LCD_PORT->BSRR=LCD_DB7_BS;
	}

	LCD_PORT->BSRR=LCD_EN_BS;
	lcd_delay();
	LCD_PORT->BSRR=LCD_EN_BC;
	lcd_delay();


}

void LcdInit(void) {
	osDelay(10);
	lcd_init(); //Инициализируем дисплей
	osDelay(10);
	lcd_clear();
	lcd_set_xy(0, 0);
	lcd_set_state(LCD_ENABLE, CURSOR_DISABLE, NO_BLINK);
}

static void writeParallel(uint8_t byte) {
	GPIO_TypeDef *lcdPort = LCD_D4_GPIO_Port;
	int8_t delay = 1;

	HAL_GPIO_WritePin(lcdPort, LCD_D4_Pin, !!(byte & 0b00010000));
	HAL_GPIO_WritePin(lcdPort, LCD_D5_Pin, !!(byte & 0b00100000));
	HAL_GPIO_WritePin(lcdPort, LCD_D6_Pin, !!(byte & 0b01000000));
	HAL_GPIO_WritePin(lcdPort, LCD_D7_Pin, !!(byte & 0b10000000));
	HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, 1);
	osDelay(delay);
	HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, 0);
	osDelay(delay);

	HAL_GPIO_WritePin(lcdPort, LCD_D4_Pin, !!(byte & 0b00000001));
	HAL_GPIO_WritePin(lcdPort, LCD_D5_Pin, !!(byte & 0b00000010));
	HAL_GPIO_WritePin(lcdPort, LCD_D6_Pin, !!(byte & 0b00000100));
	HAL_GPIO_WritePin(lcdPort, LCD_D7_Pin, !!(byte & 0b00001000));
	HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, 1);
	osDelay(delay);
	HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, 0);
	osDelay(delay);
}

static void Lcd_CmdWrite(uint8_t byte) {
	HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, 0);
	HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, 0);
	writeParallel(byte);
}

static void Lcd_DataWrite(uint8_t byte) {
	HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, 1);
	HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, 0);
	writeParallel(byte);
}

void LcdShow4Lines(const char *l1, const char *l2, const char *l3, const char *l4) {
	Lcd_CmdWrite(0x01);        // Clear Display
	Lcd_CmdWrite(0x80); // Move the cursor to cell 1
	while(*l1) {
		Lcd_DataWrite(*l1++);
	}

	Lcd_CmdWrite(0b10001000); // Move the cursor to cell 2
	while(*l2) {
		Lcd_DataWrite(*l2++);
	}

	Lcd_CmdWrite(0b11000000); // Move the cursor to cell 3
	while(*l3) {
		Lcd_DataWrite(*l3++);
	}

	Lcd_CmdWrite(0b11001000); // Move the cursor to cell 4
	while(*l4) {
		Lcd_DataWrite(*l4++);
	}
}

//void LcdShowTwoLines(const char *line1, const char *line2) {
//	Lcd_CmdWrite(0x01);        // Clear Display
//	Lcd_CmdWrite(0x80); // Move the cursor to beginning of first line
//	while(*line1) {
//		Lcd_DataWrite(*line1++);
//	}
//
//	Lcd_CmdWrite(0b11000000); // Move the cursor to beginning of second line
//	while(*line2) {
//		Lcd_DataWrite(*line2++);
//	}
//}

void LcdShowMessage(const char *msg) {
//	LcdShowTwoLines(msg, "");
	LcdShow4Lines(msg, "", "", "");
//	LcdShow4Lines(msg, "1234", "789", "GAS:4095");
}


