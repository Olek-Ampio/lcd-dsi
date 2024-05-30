#include "main.h"
#include "lcd.h"

#define CMD_SLEEP_OUT					0x11
#define CMD_DISPLAY_INVERSION_OFF		0x20
#define CMD_DISPLAY_INVERSION_ON		0x21
#define CMD_DISPLAY_ON					0x29
#define CMD_MEMORY_ACCESS_CONTROL		0x36
#define	CMD_INTERFACE_PIXEL_FORMAT		0x3A
#define CMD_FRAME_RATE_CONTROL			0xB1
#define CMD_DISPLAY_INVERSION_CONTROL	0xB4
#define CMD_DISPLAY_FUNCTION_CONTROL	0xB6
#define CMD_POWER_CONTROL_1				0xC0
#define CMD_POWER_CONTROL_2				0xC1
#define CMD_VCOM_CONTROL_1				0xC5
#define CMD_PGAMCTRL					0xE0
#define CMD_NGAMCTRL					0xE1
#define CMD_SET_IMAGE_FUNCTION			0xE9
#define CMD_ADJUST_CONTROL_3			0xF7
#define CMD_ADJUST_CONTROL_4			0xF8

extern DSI_HandleTypeDef hdsi;

//##############################################################################
void lcd_reset(void) {
	HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(120);
}

//##############################################################################
static void write_cmd_no_params(uint8_t cmd) {
	HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P0, cmd, 0x00);
}

//##############################################################################
static void write_cmd_one_param(uint8_t cmd, uint8_t param) {
	HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, cmd, param);
}

//##############################################################################
static void write_cmd(uint8_t *params, uint16_t params_len) {
	if (params_len == 1) {
		write_cmd_no_params(params[0]);
	} else if (params_len == 2) {
		write_cmd_one_param(params[0], params[1]);
	} else {
		HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, params_len - 1,
						  params[0], &params[1]);
	}
}

//##############################################################################
void lcd_init(void) {
	uint8_t cmd_e0[] = {0xE0, 0x0D, 0x13, 0x14, 0x01, 0x0C, 0x03, 0x31, 0x46, 0x45, 0x03, 0x0C, 0x0A, 0x2A, 0x30, 0x0D};
//	write_cmd(cmd_e0, sizeof(cmd_e0));

	uint8_t cmd_e1[] = {0xE1, 0x0A, 0x10, 0x16, 0x05, 0x12, 0x08, 0x3D, 0x45, 0x53, 0x07, 0x11, 0x0E, 0x30, 0x33, 0x0A};
//	write_cmd(cmd_e1, sizeof(cmd_e1));

	uint8_t cmd_c0[] = {0xC0, 0x0A, 0x0A};
//	write_cmd(cmd_c0, sizeof(cmd_c0));

	write_cmd_one_param(0xC1, 0x41);

	uint8_t cmd_c5[] = {0xC5, 0x00, 0x25, 0x80};
//	write_cmd(cmd_c5, sizeof(cmd_c5));

	write_cmd_one_param(0x36, 0x48);

	write_cmd_one_param(0x3A, 0x55);	//0x55 - 16bit

	write_cmd_one_param(0xF8, 0x05);

	uint8_t cmd_b1[] = {0xB1, 0xA0, 0x11};
//	write_cmd(cmd_b1, sizeof(cmd_b1));

	write_cmd_one_param(0xB4, 0x02);

	uint8_t cmd_b6[] = {0xB6, 0x82, 0x22, 0x3B};
//	write_cmd(cmd_b6, sizeof(cmd_b6));

	write_cmd_one_param(0xE9, 0x01);

	uint8_t cmd_f7[] = {0xF7, 0xA9, 0x51, 0x2C, 0x82};
//	write_cmd(cmd_f7, sizeof(cmd_f7));

	write_cmd_no_params(CMD_DISPLAY_INVERSION_ON);

	write_cmd_no_params(DSI_EXIT_SLEEP_MODE);

	HAL_Delay(120);

	write_cmd_no_params(DSI_SET_DISPLAY_ON);

	HAL_Delay(100);
}
