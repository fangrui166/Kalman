#ifndef __PX8468WM_H__
#define __PX8468WM_H__

#define CHIP_VERSION				0x4727a000
#define cpu_to_le32(x)				(x)

#define MAX_ADDR_COMP_LEN			16
#define MAX_READ_MSG_LEN			MAX_ADDR_COMP_LEN
#define MAX_WRITE_MSG_LEN			MAX_ADDR_COMP_LEN

#define PANEL_1080_1200

#define USE_HAL_I2C					1
#define PX8468WM_SLAVE_ADDR			(0x44)	// w0x44 r0x45


struct addr_val {
	uint32_t addr;
	uint32_t data;
};

enum mipi_rx_work_mode {
	bypass_video = 0,
	bypass_command = 1,
	mcu_video = 2,
	mcu_command = 3,
	pwil_video = 4,
	pwil_command = 5,
};

enum ap_to_romcode_cmd {
	config_data_path = 0,
	enable_dport = 1,
};

enum mipi_input_timing {
	timing_no_define = 0,
	timing_960_1080 = 1,
	timing_667_750 = 2,
	timing_1280_1440 = 3,
};

static char* mipi_input_timing_str[] = {
	"timing_no_define",
	"timing_960_1080",
	"timing_667_750",
	"timing_1280_1440",
};

enum mipi_output_timing {
	timing_default = 1,
};

int iris2_init(uint8_t Instance);
int iris2_panel_on(uint8_t Instance);
int iris2_panel_off(uint8_t Instance);
int iris2_panel_command_write(uint8_t Instance, uint32_t cmd_header, uint32_t cmd_payload);
int iris2_check_status(uint8_t Instance);
int iris2_input_timing_select(uint8_t Instance, enum mipi_input_timing timing_mode);

#endif /* __PX8468WM_H__ */

