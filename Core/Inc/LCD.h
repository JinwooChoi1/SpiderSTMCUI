/*
+++   Nima Askari
+++   www.github.com/NimaLTD
+++   www.instagram.com/github.NimaLTD
+++   Version: 1.1.0
*/

/*
 * LCD.h
 *
 *  Created on: Dec 14, 2021
 *      Author: jinwoochoi
 */

#ifndef SRC_LCD_H_
#define SRC_LCD_H_
#include "main.h"

#include "cmsis_os.h"

//############################################################################################
typedef struct {
	uint8_t DisplayControl;
	uint8_t DisplayFunction;
	uint8_t DisplayMode;
	uint8_t currentX;
	uint8_t currentY;

} LCD_Options_t;


//############################################################################################
/* MCP23017 definitions */
#define MCP23017_ADDRESS    0x20 //!< MCP23017 serial address
#define MCP23017_IODIRA     0x00   //!< I/O direction register for Port A
#define MCP23017_IODIRB     0x01   //!< I/O direction register for Port B
#define MCP23017_IPOLA      0x02   //!< Input polarity register for Port A
#define MCP23017_IPOLB      0x03   //!< Input polarity register for Port B
#define MCP23017_GPINTENA   0x04   //!< Interrupt-on-change control register for Port A
#define MCP23017_GPINTENB   0x05   //!< Interrupt-on-change control register for Port B
#define MCP23017_DEFVALA    0x06   //!< Default value register for Port A
#define MCP23017_DEFVALB    0x07   //!< Default value register for Port B
#define MCP23017_INTCONA    0x08   //!< Interrupt control register for Port A
#define MCP23017_INTCONB    0x09   //!< Interrupt control register for Port B
#define MCP23017_IOCONA     0x0A   //!< Configuration register for Port A
#define MCP23017_IOCONB     0x0B 			//!< Configuration register for Port B
#define MCP23017_GPPUA      0x0C   		//!< Pull-up resistor configuration register for Port A
#define MCP23017_GPPUB      0x0D   //!< Pull-up resistor configuration register for Port B
#define MCP23017_INTFA      0x0E   //!< Interrupt flag register for Port A
#define MCP23017_INTFB      0x0F   //!< Interrupt flag register for Port B
#define MCP23017_INTCAPA    0x10   //!< Interrupt capture register for Port A
#define MCP23017_INTCAPB    0x11   //!< Interrupt capture register for Port B
#define MCP23017_GPIOA      0x12   //!< Port register for Port A
#define MCP23017_GPIOB      0x13   //!< Port register for Port B
#define MCP23017_OLATA      0x14   //!< Output latch register for Port A
#define MCP23017_OLATB      0x15   //!< Output latch register for Port B


//############################################################################################
/* Pin definitions */
#define LCD_RS_LOW              HAL_GPIO_WritePin(_LCD_RS_PORT, _LCD_RS_PIN,GPIO_PIN_RESET)
#define LCD_RS_HIGH             HAL_GPIO_WritePin(_LCD_RS_PORT, _LCD_RS_PIN,GPIO_PIN_SET)
#define LCD_E_LOW               HAL_GPIO_WritePin(_LCD_E_PORT,  _LCD_E_PIN,GPIO_PIN_RESET)
#define LCD_E_HIGH              HAL_GPIO_WritePin(_LCD_E_PORT,  _LCD_E_PIN,GPIO_PIN_SET)
#define LCD_E_BLINK             LCD_E_HIGH; LCD_Delay_us(50); LCD_E_LOW; LCD_Delay_us(50)

//############################################################################################
/* Commands*/
#define LCD_CLEARDISPLAY        0x01
#define LCD_RETURNHOME          0x02
#define LCD_ENTRYMODESET        0x04
#define LCD_DISPLAYCONTROL      0x08
#define LCD_CURSORSHIFT         0x10
#define LCD_FUNCTIONSET         0x20
#define LCD_SETCGRAMADDR        0x40
#define LCD_SETDDRAMADDR        0x80
/* Flags for display entry mode */
#define LCD_ENTRYRIGHT          0x00
#define LCD_ENTRYLEFT           0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00
/* Flags for display on/off control */
#define LCD_DISPLAYON           0x04
#define LCD_CURSORON            0x02
#define LCD_BLINKON             0x01
/* Flags for display/cursor shift */
#define LCD_DISPLAYMOVE         0x08
#define LCD_CURSORMOVE          0x00
#define LCD_MOVERIGHT           0x04
#define LCD_MOVELEFT            0x00
/* Flags for function set */
#define LCD_8BITMODE            0x10
#define LCD_4BITMODE            0x00
#define LCD_2LINE               0x08
#define LCD_1LINE               0x00
#define LCD_5x10DOTS            0x04
#define LCD_5x8DOTS             0x00
//############################################################################################
#define LCD_COLS         16
#define LCD_ROWS         2

class LCD {
public:
	LCD();
	virtual ~LCD();
	void begin(I2C_HandleTypeDef* _hi2c, uint8_t _address = MCP23017_ADDRESS);
	void DisplayOn(void);
	void DisplayOff(void);
	void Clear(void);
	void Puts(uint8_t x, uint8_t y, const char* str);
	void Print(const char* str);
	void BlinkOn(void);
	void BlinkOff(void);
	void CursorOn(void);
	void CursorOff(void);
	void ScrollLeft(void);
	void ScrollRight(void);
	void CreateChar(uint8_t location, uint8_t* data);
	void PutCustom(uint8_t x, uint8_t y, uint8_t location);
	void Put(uint8_t Data);
	void Cmd(uint8_t cmd);
	void RawCmd(uint8_t cmd, uint8_t rs = 0);
	void ECmd(uint8_t onOff = 0);
	void RWCmd(uint8_t onOff = 0);
	void RSCmd(uint8_t onOff = 0);
	void Data(uint8_t data);
	void CursorSet(uint8_t col, uint8_t row);
	uint8_t ReadPins(void);
	LCD_Options_t Opts;
	uint8_t lcdTxFlag = 0;
#ifdef CMSIS_OS_H_
  void setTask(osThreadId_t* _DRVTask);
  void setTXFlag(void);
  void setRXFlag(void);
#endif
private:
	I2C_HandleTypeDef* hi2c;
	uint8_t address;
	static void Delay_us(uint16_t us);
	static void Delay_ms(uint8_t ms);
	void OneCmd(uint8_t onOff, uint8_t pinNum);
#ifdef CMSIS_OS_H_
  osThreadId_t* LCDTask;
#endif
};

#endif /* SRC_LCD_H_ */
