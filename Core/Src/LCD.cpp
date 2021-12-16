/*
 * LCD.cpp
 *
 *  Created on: Dec 14, 2021
 *      Author: jinwoochoi
 */
#include "LCD.h"

void LCD::Delay_us(uint16_t us){
  uint32_t  Div = (SysTick->LOAD+1)/1000;
  uint32_t  StartMicros = HAL_GetTick()*1000 + (1000- SysTick->VAL/Div);
  while((HAL_GetTick()*1000 + (1000-SysTick->VAL/Div)-StartMicros < us));
}
//############################################################################################
void LCD::Delay_ms(uint8_t ms){
  #ifdef CMSIS_OS_H_
  osDelay(ms);
  #else
  HAL_Delay(ms);
  #endif
}
//############################################################################################
void LCD::begin(I2C_HandleTypeDef* _hi2c, uint8_t _address){
	hi2c = _hi2c;
	address = _address << 1;
	uint8_t GPIOA_config = 0x1F;
	uint8_t GPIOA_turnon = 0xC0;
	uint8_t GPIOB_config = 0x00;
	HAL_I2C_Mem_Write(hi2c, address, MCP23017_IODIRA, 1, &GPIOA_config, 1, 100);
	HAL_I2C_Mem_Write(hi2c, address, MCP23017_GPPUA, 1, &GPIOA_config, 1, 100);
	HAL_I2C_Mem_Write(hi2c, address, MCP23017_GPIOA, 1, &GPIOA_turnon, 1, 100);
	HAL_I2C_Mem_Write(hi2c, address, MCP23017_IODIRB, 1, &GPIOB_config, 1, 100);

	/* Set cursor pointer to beginning for LCD */
	Opts.currentX = 0;
	Opts.currentY = 0;
	Opts.DisplayFunction = LCD_4BITMODE | LCD_5x8DOTS | LCD_1LINE;
	if (LCD_ROWS > 1)
		Opts.DisplayFunction |= LCD_2LINE;
	/* Try to set 4bit mode */
	RawCmd(0x03);
	Delay_ms(5);
	/* Second try */
	RawCmd(0x03);
	Delay_ms(5);
	/* Third goo! */
	RawCmd(0x03);
	Delay_ms(5);
	/* Set 4-bit interface */
	RawCmd(0x02);
	Delay_ms(5);
	/* Set # lines, font size, etc. */
	Cmd(LCD_FUNCTIONSET | Opts.DisplayFunction);
	/* Turn the display on with no cursor or blinking default */
	Opts.DisplayControl = LCD_DISPLAYON;
	DisplayOn();
	Clear();
	/* Default font directions */
	Opts.DisplayMode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
	Cmd(LCD_ENTRYMODESET | Opts.DisplayMode);
	Delay_ms(5);
}
LCD::LCD(){
}
LCD::~LCD() {
}
//############################################################################################
void LCD::Clear(void)
{
	Cmd(LCD_CLEARDISPLAY);
	Delay_ms(5);
}
//############################################################################################
void LCD::Puts(uint8_t x, uint8_t y, const char* str)
{
	LCD::CursorSet(x, y);
	while (*str)
  {
		if (LCD::Opts.currentX >= LCD_COLS)
    {
			Opts.currentX = 1;
			Opts.currentY++;
			CursorSet(Opts.currentX, Opts.currentY);
		}
		if (*str == '\n')
    {
			Opts.currentX = 1;
			Opts.currentY++;
			CursorSet(Opts.currentX, Opts.currentY);
		}
    else if (*str == '\r')
    {
			CursorSet(0, Opts.currentY);
		}
    else
    {
			Data(*str);
			Opts.currentX++;
		}
		str++;
	}
}
//############################################################################################
void LCD::Print(const char* str){
	Clear();
	Puts(1, 0, str);
}
//############################################################################################
void LCD::DisplayOn(void){
	Opts.DisplayControl |= LCD_DISPLAYON;
	Cmd(LCD_DISPLAYCONTROL | Opts.DisplayControl);
}
//############################################################################################
void LCD::DisplayOff(void){
	Opts.DisplayControl &= ~LCD_DISPLAYON;
	Cmd(LCD_DISPLAYCONTROL | Opts.DisplayControl);
}
//############################################################################################
void LCD::BlinkOn(void){
	Opts.DisplayControl |= LCD_BLINKON;
	Cmd(LCD_DISPLAYCONTROL | Opts.DisplayControl);
}
//############################################################################################
void LCD::BlinkOff(void){
	Opts.DisplayControl &= ~LCD_BLINKON;
	Cmd(LCD_DISPLAYCONTROL | Opts.DisplayControl);
}
//############################################################################################
void LCD::CursorOn(void){
	Opts.DisplayControl |= LCD_CURSORON;
	Cmd(LCD_DISPLAYCONTROL | Opts.DisplayControl);
}
//############################################################################################
void LCD::CursorOff(void){
	Opts.DisplayControl &= ~LCD_CURSORON;
	Cmd(LCD_DISPLAYCONTROL | Opts.DisplayControl);
}
//############################################################################################
void LCD::ScrollLeft(void){
	Cmd(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}
//############################################################################################
void LCD::ScrollRight(void){
	Cmd(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}
//############################################################################################
void LCD::CreateChar(uint8_t location, uint8_t *data){
	uint8_t i;
	/* We have 8 locations available for custom characters */
	location &= 0x07;
	Cmd(LCD_SETCGRAMADDR | (location << 3));

	for (i = 0; i < 8; i++) {
		Data(data[i]);
	}
}
//############################################################################################
void LCD::PutCustom(uint8_t x, uint8_t y, uint8_t location){
	CursorSet(x, y);
	Data(location);
}
//############################################################################################
void LCD::Cmd(uint8_t cmd){
	RSCmd();
	RawCmd(cmd >> 4);
	RawCmd(cmd & 0x0F);
}
//############################################################################################
void LCD::Data(uint8_t data){
	RSCmd(1);
	RawCmd(data >> 4, 1);
	RawCmd(data & 0x0F, 1);
}
//############################################################################################
void LCD::RawCmd(uint8_t cmd, uint8_t rs){
	/*
	 *
	 * I2C GPIOA Pin Configuration
	 * GREEN | RED | -- | LEFT | UP | DOWN | RIGHT | SELECT
	 *
	 * I2C GPIOB Pin Configuration
	 * RS | RW | E  | DB4 | DB5 | DB6 | DB7 | BLUE
	 *
	 * Command Configuration
	 * -- | -- | -- | -- | D7 | D6 | D5 | D4
	 */

	uint8_t i2ccmd = 0x01;
	i2ccmd |= ((cmd & 0x08) >> 2);
	i2ccmd |= (cmd & 0x04);
	i2ccmd |= ((cmd & 0x02) << 2);
	i2ccmd |= ((cmd & 0x01) << 4);
	//i2ccmd |= cmd << 1;
	i2ccmd |= 1 << 5;
	i2ccmd |= rs << 7;

	#ifdef CMSIS_OS_H_
		osThreadFlagsClear((1 << 0)|(1 << 1));
		HAL_I2C_Mem_Write_IT(hi2c, address, MCP23017_GPIOB, 1, &i2ccmd, 1);
		osThreadFlagsWait(1 << 0, osFlagsWaitAll, 100);
	#else
		HAL_I2C_Mem_Write(hi2c, address, MCP23017_GPIOB, 1, &i2ccmd, 1, 100);
	#endif
	ECmd(0);
}
//############################################################################################
void LCD::ECmd(uint8_t onOff){
	OneCmd(onOff, 5);
}
//############################################################################################
void LCD::RWCmd(uint8_t onOff){
	OneCmd(onOff, 6);
}
//############################################################################################
void LCD::RSCmd(uint8_t onOff){
	OneCmd(onOff, 7);
}
//############################################################################################
void LCD::OneCmd(uint8_t onOff, uint8_t pinNum){
	uint8_t i2ccmd = (onOff << pinNum)|1;
#ifdef CMSIS_OS_H_
  osThreadFlagsClear((1 << 0)|(1 << 1));
	HAL_I2C_Mem_Write_IT(hi2c, address, MCP23017_GPIOB, 1, &i2ccmd, 1);
  osThreadFlagsWait(1 << 0, osFlagsWaitAll, 100);
#else
	HAL_I2C_Mem_Write(hi2c, address, MCP23017_GPIOB, 1, &i2ccmd, 1, 100);
#endif
}
//############################################################################################
void LCD::CursorSet(uint8_t col, uint8_t row){
	uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
	if (row >= LCD_ROWS)
		row = 0;
	Opts.currentX = col;
	Opts.currentY = row;
	Cmd(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}
//############################################################################################
void LCD::Put(uint8_t data){
	Data(data);
}
//############################################################################################
uint8_t LCD::ReadPins(){
	/*
	 * I2C GPIOA Pin Configuration
	 * GREEN | RED | -- | LEFT | UP | DOWN | RIGHT | SELECT
	 */
	uint8_t i2ccmd = 0xC0;
	uint8_t val = 0;
#ifdef CMSIS_OS_H_
  osThreadFlagsClear((1 << 0)|(1 << 1));
	HAL_I2C_Mem_Write_IT(hi2c, address, MCP23017_GPIOA, 1, &i2ccmd, 1);
  osThreadFlagsWait(1 << 0, osFlagsWaitAll, 100);
  osThreadFlagsClear((1 << 0)|(1 << 1));
	HAL_I2C_Mem_Read_IT(hi2c, address, MCP23017_GPIOA, 1, &val, 1);
  osThreadFlagsWait(1 << 0, osFlagsWaitAll, 100);
#else
	HAL_I2C_Mem_Write(hi2c, address, MCP23017_GPIOB, 1, &i2ccmd, 1, 100);
	HAL_I2C_Mem_Read(hi2c, address, MCP23017_GPIOB, 1, &val, 1, 100);
#endif
	return val & 0x1F;
}
#ifdef CMSIS_OS_H_
void LCD::setTask(osThreadId_t* _LCDTask) {
	LCDTask = _LCDTask;
}

void LCD::setTXFlag(void) {
	osThreadFlagsSet(*LCDTask, 1 << 0);
}

void LCD::setRXFlag(void) {
	osThreadFlagsSet(*LCDTask, 1 << 1);
}
#endif
