/* This file contains the definition of BMC + FX3 as I2c slave test.
 * Firmware download unit test.*/

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

//#include "FreeRTOS.h"
#include "usart.h"
#include "stm32f4xx_hal.h"
//#include "cmsis_os.h"
#include "integer.h"
//#include "gpio.h"
#include "htc_memory_define.h"

#include "flash_drv.h"
//#include "hlog_api.h"
//#include "ccgx_ccg4.h"
//#include "ccgx_ec.h"
#include "ccgx_fw_io.h"
#include "ccgx_fw.h"

// Macro definitions.

#define ccgx_fw_pr(fmt, args...)              printf("%s : " fmt, "CCGX_FW", ##args)

#if CCGX_FW_DEBUG
#define ccgx_fw_pr_dbg(fmt, args...)       printf("%s : " fmt, "CCGX_FW", ##args)
#else
#define ccgx_fw_pr_dbg(fmt, args...)
#endif

#define CCGX_FW_UPGRADE_BL           1
#define CCGX_FW_UPGRADE_FAST         1


static CY_HANDLE handle_ccg4 = NULL;

uint16_t flash_row_size;

typedef enum CyCtrlRegsAddr_t {
	CCG_REG_DEVICE_MODE = 0x0000,	// 0
	CCG_REG_BOOTMODE_RSN,	// 1
	CCG_REG_SIID_LSB,	// 2
	CCG_REG_SIID_MSB,	// 3
	CCG_REG_BL_LASTROW_LSB,	// 4
	CCG_REG_BL_LASTROW_MSB,	// 5
	CCG_REG_INTR_STAT,	// 6
	CCG_REG_JUMP_TO_BOOT,	// 7
	CCG_REG_RESET_RQT,	// 8
	CCG_REG_RESET_TYPE,	// 9
	CCG_REG_FLASHMODE_ENTER,	// A
	CCG_REG_VALIDATE_FW,	// B
	CCG_REG_FLASH_SIG,	// C
	CCG_REG_FLASH_CMD,	// D
	CCG_REG_FLASH_ROW_LSB,	// E
	CCG_REG_FLASH_ROW_MSB,	// F
	CCG_REG_BLVER_B0,	// 10
	CCG_REG_BLVER_B1,	// 11
	CCG_REG_BLVER_B2,	// 12
	CCG_REG_BLVER_B3,	// 13
	CCG_REG_BLVER_B4,	// 14
	CCG_REG_BLVER_B5,	// 15
	CCG_REG_BLVER_B6,	// 16
	CCG_REG_BLVER_B7,	// 17
	CCG_REG_FW1VER_B0,	// 18
	CCG_REG_FW1VER_B1,	// 19
	CCG_REG_FW1VER_B2,	// 1A
	CCG_REG_FW1VER_B3,	// 1B
	CCG_REG_FW1VER_B4,	// 1C
	CCG_REG_FW1VER_B5,	// 1D
	CCG_REG_FW1VER_B6,	// 1E
	CCG_REG_FW1VER_B7,	// 1F
	CCG_REG_FW2VER_B0,	// 20
	CCG_REG_FW2VER_B1,	// 21
	CCG_REG_FW2VER_B2,	// 22
	CCG_REG_FW2VER_B3,	// 23
	CCG_REG_FW2VER_B4,	// 24
	CCG_REG_FW2VER_B5,	// 25
	CCG_REG_FW2VER_B6,	// 26
	CCG_REG_FW2VER_B7,	// 27
	CCG_REG_FW1_LOC_LSB,	// 28
	CCG_REG_FW1_LOC_MSB,	// 29
	CCG_REG_FW2_LOC_LSB,	// 2A
	CCG_REG_FW2_LOC_MSB,	// 2B
	CCG_REG_PDPORT_EN,	// 2C
	CCG_REG_RESP_TYPE = 0x007E,	// 7E
	CCG_REG_RESP_LEN	// 7F
} CyCtrlRegsAddr_t;


static unsigned ccgx_fws_offset[2] = {CCGX_FLASH_FW_START, CCGX_FLASH_FW1_START};
static unsigned ccgx_fws_size[2]  =   {CCGX_FLASH_FW_SIZE, CCGX_FLASH_FW1_SIZE};

static int ccgx_get_fw_info(int fw_idx, unsigned *offset, unsigned *size)
{
	if (fw_idx != 0 && fw_idx != 1)
		return  -1;

	*offset = ccgx_fws_offset[fw_idx];
	*size = ccgx_fws_size[fw_idx];
	return 0;
}

int ccgx_set_fw_info(int fw_idx, unsigned offset, unsigned size)
{
	if (fw_idx != 0 &&  fw_idx != 1)
		return  -1;

	ccgx_fws_offset[fw_idx] = offset;
	ccgx_fws_size[fw_idx] = size;

	return 0;
}

static void ccgx_fw_delay(unsigned ms)
{
	unsigned tv = 0;
	//printf("in %s : ms = %d\n", __func__, ms);
	while (tv < ms) {

		watchdog_refresh();
		//osDelay(20);
		//HAL_Delay(20);
		{
			int i;
			for (i = 0; i < 200000; i++)
				asm volatile("nop\n\t");
		}
		tv += 20;
	}

	watchdog_refresh();
	//printf("exit %s\n", __func__);

	return ;
}

bool CyUsGetFwInfo(CY_HANDLE handle, uint8_t * mode_p)
{
	bool stat;
	uint8_t buf[64];
	uint16_t len = 0;
#if CCGX_FW_UPGRADE_FAST
	stat =
		CyUsRegisterRead(handle, CCG4_SLAVE_I2C_ADDR, CCG_REG_DEVICE_MODE,
				40, buf, &len);
#else
	stat =
		CyUsRegisterRead(handle, CCG4_SLAVE_I2C_ADDR, CCG_REG_DEVICE_MODE,
				64, buf, &len);
#endif

	if (stat == true) {

		if ((buf[2] == 0x00) && (buf[3] == 0x18))

			ccgx_fw_pr_dbg("CYPD4225-40LQXI device found\n");

		if ((buf[2] == 0x01) && (buf[3] == 0x18))

			ccgx_fw_pr_dbg("CYPD4125-40LQXI device found\n");

		if ((buf[2] == 0x04) && (buf[3] == 0x1D))

			ccgx_fw_pr_dbg("CYPD3125-40LQXI device found\n");

		switch (buf[0] & 0x03) {

			case 0:

				ccgx_fw_pr_dbg("Running Bootloader version %x\n",
						(uint16_t) buf[16]);

				*mode_p = 0;

				break;

			case 1:

				ccgx_fw_pr_dbg("Running FW1 version %x\n", (uint16_t) buf[24]);

				*mode_p = 1;

				break;

			case 2:

				ccgx_fw_pr_dbg("Running FW2 version %x\n", (uint16_t) buf[32]);

				*mode_p = 2;

				break;

			default:

				ccgx_fw_pr_dbg("Running unknown firmware version\n");

				return false;

		}

		if (buf[0] & 0x70)

			flash_row_size = 256;

		else

			flash_row_size = 128;

		//		printf("\n");

		return true;

	}

	else {

		ccgx_fw_pr_dbg("Error: Register read failed with status %d \n", stat);

		return false;

	}

}

int ccgx_get_fw_version(int idx, unsigned char *v)
{

	bool stat;
	uint8_t buf[32];
	uint16_t len = 0;

	memset(buf, 0x00, sizeof(buf));
	stat =
		CyUsRegisterRead(handle_ccg4, CCG4_SLAVE_I2C_ADDR, CCG_REG_BLVER_B0,
				24, buf, &len);

	if (stat == true) {
		memcpy(v, buf + (idx << 3), 8);
		return true;
	} else {
		ccgx_fw_pr_dbg("Error: Register read failed with status %d \n", stat);
		return false;
	}
}

bool CyUsReadResponse(CY_HANDLE handle, uint8_t * resp_p)
{

	bool stat;
	uint8_t buf[64], intstat;
	uint16_t len = 0;

#if CCGX_FW_UPGRADE_FAST
	stat =
		CyUsRegisterRead(handle, CCG4_SLAVE_I2C_ADDR, CCG_REG_INTR_STAT,
				1, buf + CCG_REG_INTR_STAT, &len);
#else
	stat =
		CyUsRegisterRead(handle, CCG4_SLAVE_I2C_ADDR, CCG_REG_DEVICE_MODE,
				64, buf, &len);
#endif

	if (stat == true) {
		intstat = buf[CCG_REG_INTR_STAT];
		if ((intstat & 0x01) != 0) {
			stat =
				CyUsRegisterRead(handle, CCG4_SLAVE_I2C_ADDR,
						CCG_REG_RESP_TYPE, 2, buf, &len);

			if (stat == true) {

				if (buf[1] != 0) {

					ccgx_fw_pr_dbg
						("Error: Invalid response length %d\n",
						 buf[1]);

					return false;

				}

				*resp_p = buf[0];

				switch (buf[0]) {

					case 0:

						ccgx_fw_pr_dbg("No response\n");

						break;

					case 2:
						ccgx_fw_pr_dbg("Success response\n");
						break;

					case 3:
						ccgx_fw_pr_dbg("Flash data available\n");
						break;

					case 5:

						ccgx_fw_pr_dbg("Invalid command\n");

						break;

					case 7:

						ccgx_fw_pr_dbg("Flash Op failed\n");

						break;

					case 8:

						ccgx_fw_pr_dbg("Bad firmware\n");

						break;

					case 9:

						ccgx_fw_pr_dbg("Bad arguments\n");

						break;

					case 10:

						ccgx_fw_pr_dbg("Not supported\n");

						break;

					case 128:

						ccgx_fw_pr_dbg("Reset complete event\n");

						break;

					case 129:

						ccgx_fw_pr_dbg("Message queue overflow\n");

						return false;

					case 1:

					case 4:

					case 6:

					default:

						ccgx_fw_pr_dbg("Invalid response\n");

						return false;

				}

				ccgx_fw_pr_dbg("Clearing the interrupt\n");

				buf[0] = 0x01;

				len = 0;

				stat =
					CyUsRegisterWrite(handle,
							CCG4_SLAVE_I2C_ADDR,
							CCG_REG_INTR_STAT, 1, buf,
							&len);

				if (stat == false) {

					ccgx_fw_pr_dbg
						("Error: Clearing interrupt failed with status %d \n",
						 stat);

					return false;

				}

			}

			else {

				ccgx_fw_pr_dbg
					("Error: Response read failed with status %d \n",
					 stat);

				return false;

			}

		}
	} else {

		ccgx_fw_pr_dbg("Error: Register read failed with status %d\n", stat);

		return false;

	}

	return true;

}

#define CCG4_HPI_SUCCESS_RESPONSE       (0x02)
#define CCG4_FLASH_READ_RESPONSE        (0x03)
#define CCG4_FLASH_ENTER_SIG            ('P')

bool CyUsEnterFlashingMode(CY_HANDLE handle)
{

	bool stat;

	bool ret;

	uint8_t buf[64];

	uint16_t len = 0;

	buf[0] = CCG4_FLASH_ENTER_SIG;

	len = 1;

	stat =
		CyUsRegisterWrite(handle, CCG4_SLAVE_I2C_ADDR,
				CCG_REG_FLASHMODE_ENTER, 1, buf, &len);

	if (stat == true) {

		ret = CyUsReadResponse(handle, buf);

		if (!ret) {

			ccgx_fw_pr_dbg("Error: Response read failed\n");

			return ret;

		}

		else {

			if (buf[0] != CCG4_HPI_SUCCESS_RESPONSE) {

				ccgx_fw_pr_dbg
					("Error: Bad response to enter flashing mode command : 0x%02x\n", buf[0]);

				return false;

			}

			return true;

		}

	}

	else {

		ccgx_fw_pr_dbg("Error: Failed to initiate flash mode entry\n");

		return false;

	}

}

bool CyUsPDPortDisable(CY_HANDLE handle)
{

	bool stat;

	uint8_t buf[4];

	uint16_t len = 0;

	buf[0] = 0;

	len = 1;

	stat =
		CyUsRegisterWrite(handle, CCG4_SLAVE_I2C_ADDR, CCG_REG_PDPORT_EN, 1,
				buf, &len);

	if (stat == true) {

		ccgx_fw_pr_dbg("Disabled all PD ports\n");
#if CCGX_FW_UPGRADE_FAST
		ccgx_fw_delay(3 * 1000);
#else
		ccgx_fw_delay(500 * 1000);
#endif

#if 1
		stat = CyUsReadResponse(handle, buf);

		if (!stat) {

			ccgx_fw_pr_dbg("Error: Response read failed\n");

			return false;

		} else if (buf[0] != 2) {

			return false;
		}
#endif

	} else {

		ccgx_fw_pr_dbg("Error: Failed to disable PD ports\n");

		return false;

	}

	return true;

}

#define CCG4_RESET_SIG          ('R')
#define CCG4_RESET_DEV_CMD      (1)

bool CyUsDeviceReset(CY_HANDLE handle, uint16_t delay)
{

	bool stat;

	bool ret;

	uint8_t buf[64];

	uint16_t len = 0;

	buf[0] = CCG4_RESET_SIG;

	buf[1] = CCG4_RESET_DEV_CMD;

	len = 2;

	stat =
		CyUsRegisterWrite(handle, CCG4_SLAVE_I2C_ADDR, CCG_REG_RESET_RQT, 2,
				buf, &len);

	if (stat == true) {

		ccgx_fw_pr_dbg("Reset completed\n");

		ccgx_fw_delay(delay* 1000);

		ret = CyUsReadResponse(handle, buf);

		if (!ret) {

			ccgx_fw_pr_dbg("Error: Response read failed\n");

			return ret;

		}

		else {

			return true;

		}

	}

	else {

		ccgx_fw_pr_dbg("Error: Failed to initiate device reset\n");

		return false;

	}

}

bool CyUsValidateFirmware(CY_HANDLE handle, uint8_t fwid)
{

	bool stat;

	bool ret;

	uint8_t buf[4];

	uint16_t len = 0;

	buf[0] = fwid;

	len = 1;

	stat =
		CyUsRegisterWrite(handle, CCG4_SLAVE_I2C_ADDR, CCG_REG_VALIDATE_FW,
				1, buf, &len);

	if (stat == true) {

		// Wait for validate to be completed.
#if CCGX_FW_UPGRADE_FAST
		ccgx_fw_delay(8 * 1000);
#else
		ccgx_fw_delay(1000 * 1000);
#endif
		ret = CyUsReadResponse(handle, buf);

		if (!ret) {

			ccgx_fw_pr_dbg("Error: Response read failed\n");

			return ret;

		}

		else {

			if (buf[0] != CCG4_HPI_SUCCESS_RESPONSE) {

				ccgx_fw_pr_dbg("Error: Firmware validation failed\n");

				return false;

			}

			return true;

		}

	}

	else {

		ccgx_fw_pr_dbg("Error: Failed to initiate validate firmware command\n");

		return false;

	}

}

#define CCG_FLASH_RW_MEM        (0x0200)
#define CCG_FLASH_WRITE_CMD     (1)
#define CCG_FLASH_READ_CMD      (0)
#define CCG_FLASH_RW_SIG        ('F')

bool CyUsWriteFlashRow(CY_HANDLE handle, uint16_t row_num, uint8_t * buffer)
{

	bool stat;

	bool ret;

	uint8_t cmdbuf[4];

	uint16_t len = 0;

	/* First write the data to the flash write memory. */
	stat =
		CyUsRegisterWrite(handle, CCG4_SLAVE_I2C_ADDR, CCG_FLASH_RW_MEM,
				flash_row_size, buffer, &len);

	if (stat == false) {

		ccgx_fw_pr_dbg("Error: Failed to write flash content\n");

		return false;

	}

	cmdbuf[0] = CCG_FLASH_RW_SIG;

	cmdbuf[1] = CCG_FLASH_WRITE_CMD;

	cmdbuf[2] = (row_num & 0xFF);

	cmdbuf[3] = (row_num >> 8);

	len = 4;

	stat =
		CyUsRegisterWrite(handle, CCG4_SLAVE_I2C_ADDR, CCG_REG_FLASH_SIG, 4,
				cmdbuf, &len);

	if (stat == true) {

		/* Allow a delay for the flash write to complete. */
#if CCGX_FW_UPGRADE_FAST
		ccgx_fw_delay(40);
#else
		ccgx_fw_delay(50 * 1000);
#endif
		ret = CyUsReadResponse(handle, cmdbuf);

		if (!ret) {

			ccgx_fw_pr_dbg("Error: Response read failed\n");

			return ret;

		}

		else {

			if (cmdbuf[0] != CCG4_HPI_SUCCESS_RESPONSE) {

				ccgx_fw_pr_dbg
					("Bad response to flash row write command\n");

				return false;

			}

			return true;

		}

	}

	else {

		ccgx_fw_pr_dbg("Error: Failed to initiate flash row write\n");

		return false;

	}

}

bool CyUsReadFlashRow(CY_HANDLE handle, uint16_t row_num, uint8_t * buffer)
{

	bool stat;

	bool ret;

	uint8_t cmdbuf[4];

	uint16_t len = 0;

	/* Initiate the flash read command. */
	cmdbuf[0] = CCG_FLASH_RW_SIG;

	cmdbuf[1] = CCG_FLASH_READ_CMD;

	cmdbuf[2] = (row_num & 0xFF);

	cmdbuf[3] = (row_num >> 8);

	len = 4;

	stat =
		CyUsRegisterWrite(handle, CCG4_SLAVE_I2C_ADDR, CCG_REG_FLASH_SIG, 4,
				cmdbuf, &len);

	if (stat == true) {

		/* Allow a delay for the flash read to complete. */
#if CCGX_FW_UPGRADE_FAST
		ccgx_fw_delay(20);
#else
		ccgx_fw_delay(10 * 1000);
#endif
		ret = CyUsReadResponse(handle, cmdbuf);

		if (!ret) {

			ccgx_fw_pr_dbg("Error: Response read failed\n");

			return ret;

		}

		else {

			if (cmdbuf[0] != CCG4_FLASH_READ_RESPONSE) {

				ccgx_fw_pr_dbg
					("Error: Bad response to flash row read command\n");

				return false;

			}

			stat =
				CyUsRegisterRead(handle, CCG4_SLAVE_I2C_ADDR,
						CCG_FLASH_RW_MEM, flash_row_size,
						buffer, &len);

			if (stat == true) {

				if (len == flash_row_size) {

					return true;

				}

				else {

					ccgx_fw_pr_dbg
						("Error: Unexpected read length of %d bytes\n",
						 len);

					return false;

				}

			}

			else {

				ccgx_fw_pr_dbg
					("Error: Failed to read flash data out\n");

				return false;

			}

		}

	}

	else {

		ccgx_fw_pr_dbg("Error: Failed to initiate flash row read\n");

		return false;

	}

}

#define CYACD_MAX_LINE_LEN      (1024)

int GetHexVal(char c)
{

	if ((c >= '0') && (c <= '9'))

		return (c - '0');

	if ((c >= 'A') && (c <= 'F'))

		return (10 + c - 'A');

	if ((c >= 'a') && (c <= 'f'))

		return (10 + c - 'a');

	return 0;

}

static int ccgx_get_fw_offset(unsigned *fw1_offset, unsigned *fw2_offset)
{
#if 0
	bool stat;
	uint8_t buf[32];
	uint16_t len = 0;
#endif

	*fw1_offset = 0x14;
	*fw2_offset = 0x100;
#if 0
	memset(buf, 0x00, 4);

	stat =
		CyUsRegisterRead(CyFindDevices(), CCG4_SLAVE_I2C_ADDR, CCG_REG_FW1_LOC_LSB,
				4, buf, &len);
	if (!stat)
		return -1;

	*fw1_offset = (buf[1] << 8) | buf[0];
	*fw2_offset = (buf[3] << 8) | buf[2];
#endif
	return 0;
}

int ccgx_enter_bl_mode(void)
{
	bool stat;
	uint8_t buf[32];
	uint16_t len = 0;

	buf[0] = '\0';
	buf[1] = '\0';
	stat =
		CyUsRegisterRead(CyFindDevices(), CCG4_SLAVE_I2C_ADDR, CCG_REG_DEVICE_MODE,
				2, buf, &len);
	if (!stat)
		return -1;
	if ((buf[0] & 0x03) == 0x00)
		return 0;

	stat = CyUsPDPortDisable(CyFindDevices());
	if (!stat)
		return -2;
#if 0
	buf[0] = 0;
	stat = CyUsReadResponse(CyFindDevices(), buf);
	if (!stat)
		return -3;
	if (buf[0] != 2)
		return -4;
#endif

	buf[0] = 'J';
	stat =
		CyUsRegisterWrite(CyFindDevices(), CCG4_SLAVE_I2C_ADDR,
				CCG_REG_JUMP_TO_BOOT, 1, buf, &len);
	if (!stat)
		return -5;
#if CCGX_FW_UPGRADE_FAST
	ccgx_fw_delay(400);
#else
	ccgx_fw_delay(60 * 1000);
#endif
	buf[0] = 0;
	stat = CyUsReadResponse(CyFindDevices(), buf);
	if (!stat)
		return -6;
	if (buf[0] != 128)
		return -7;

	buf[0] = '\0';
	buf[1] = '\0';
	stat =
		CyUsRegisterRead(CyFindDevices(), CCG4_SLAVE_I2C_ADDR, CCG_REG_DEVICE_MODE,
				2, buf, &len);
	if (!stat)
		return -8;
	if ((buf[0] & 0x03) != 0x00)
		return -9;

	return 0;
}

#if CCGX_FW_UPGRADE_BL == 0
int  ccgx_upgrade_fw_fw(void)
{
	bool stat;
	int fw_idx_up = 1;
	uint8_t mode, resp;
	uint8_t flashbuf[256];
	uint8_t readbuf[256];
	char line[CYACD_MAX_LINE_LEN];
	unsigned char fw_ver[16];

	uint16_t flash_mid_row;
	unsigned f_offset;
	unsigned offset;
	unsigned f_size;

	handle_ccg4 = CyFindDevices();
	if (handle_ccg4 == NULL) {
		return -1;
	}

	stat = CyUsGetFwInfo(handle_ccg4, &mode);
	if (!stat)
		return -2;

	ccgx_fw_pr_dbg("flash row size : %d\n", flash_row_size);

	if (flash_row_size == 256)
		flash_mid_row = 256;
	else
		flash_mid_row = 512;

	if (mode == 1)
		fw_idx_up = 2;
	else if (mode == 2)
		fw_idx_up = 1;
	else {
		ccgx_fw_pr_dbg("Invalid device mode %d\n", mode);
		return -3;
	}

	memset(fw_ver, 0x00, sizeof(fw_ver));
	ccgx_get_fw_version(fw_idx_up, fw_ver);
	ccgx_fw_pr_dbg("upgrade FW%d version : 0x%02x 0x%02x 0x%02x 0x%02x \t 0x%02x 0x%02x 0x%02x 0x%02x\n", fw_idx_up,
			fw_ver[0], fw_ver[1],  fw_ver[2],  fw_ver[3],  fw_ver[4], fw_ver[5],  fw_ver[6],  fw_ver[7]);

	stat = CyUsReadResponse(handle_ccg4, &resp);
	if (!stat)
		return -4;

	stat = CyUsEnterFlashingMode(handle_ccg4);
	if (!stat)
		return -5;

	ccgx_fw_pr_dbg
		("Doing firmware upgrade. This might take a couple of minutes\n\n");

	ccgx_get_fw_info(fw_idx_up, &offset, &f_size);

	ccgx_fw_pr_dbg("fw_idx_up = %d, offset = 0x%08x, f_size = %u\n",
			fw_idx_up, offset, f_size);

	f_offset = offset;
	// Read row data and program to device.
	while (1) {

		if (offset - f_offset >= f_size)
			break;

		memset(line, 0xff, sizeof(line));
		if (ccgx_fw_fgets((uint8_t *) line, CYACD_MAX_LINE_LEN, &offset) < 0 ) {
			ccgx_fw_pr_dbg("FW image corrupted\n");
			return -6;
		}

		ccgx_fw_pr_dbg("offset = 0x%08x\n", offset);

		if (line[0] == ':') {

			uint16_t row =
				((GetHexVal(line[3]) << 12) |
				 (GetHexVal(line[4]) << 8) |
				 (GetHexVal(line[5]) << 4) | (GetHexVal(line[6])));

			uint16_t len =
				((GetHexVal(line[7]) << 12) |
				 (GetHexVal(line[8]) << 8) |
				 (GetHexVal(line[9]) << 4) | (GetHexVal(line[10])));

			memset(flashbuf, 0xff, sizeof(flashbuf));

			for (int i = 0; i < len; i++) {

				flashbuf[i] =
					((GetHexVal(line[11 + 2 * i]) << 4) |
					 (GetHexVal(line[12 + 2 * i])));

			}

			stat = CyUsWriteFlashRow(handle_ccg4, row, flashbuf);

			if (stat) {

				ccgx_fw_pr_dbg("Finished writing flash row %x\n", row);
			}

			else {

				ccgx_fw_pr_dbg("Error: Flash write to row %x failed\n",
						row);

				if ((mode == 1) && (row < flash_mid_row)) {

					ccgx_fw_pr_dbg
						("\n\nFW1 cannot be updated while FW1 is running. Please choose FW2 cyacd file\n");

					return -7;

				}

				if ((mode == 2) && (row >= flash_mid_row)) {

					ccgx_fw_pr_dbg
						("\n\nFW2 cannot be updated while FW2 is running. Please choose FW1 cyacd file\n");

					return -8;

				}

			}

			memset(readbuf, 0xff, sizeof(readbuf));

			stat = CyUsReadFlashRow(handle_ccg4, row, readbuf);

			if (stat) {

				if (memcmp(readbuf, flashbuf, flash_row_size)
						== 0) {
					ccgx_fw_pr_dbg
						("Flash read-verify on row %x successful\n",
						 row);
				}

				else {

					if ((flash_row_size == 256 && row < 510)
							||
							(flash_row_size == 128
							 && row < 1022))

						ccgx_fw_pr_dbg ("Error: Flash data miscompare on row %x row\n",row);
					return -9;

				}

			}

			else {

				ccgx_fw_pr_dbg("Error: Flash read to row %x failed\n",
						row);
				return -10;

			}

		}
		else {

			ccgx_fw_pr_dbg("Skipped line: %x \n", line);

		}

	}

	ccgx_fw_pr_dbg("Finished updating flash. Validating firmware\n");
	// Validate both copies of firmware
	if (mode != 1) {
		if (CyUsValidateFirmware(handle_ccg4, 1))  {

			ccgx_fw_pr_dbg("FW1 is valid\n");
		} else {

			ccgx_fw_pr_dbg("FW1 is not valid\n");
			return -11;
		}
	}

	else {
		ccgx_fw_pr_dbg("Skipped FW1 validation as FW1 is running\n");
	}

	if (mode != 2) {
		if (CyUsValidateFirmware(handle_ccg4, 2)) {
			ccgx_fw_pr_dbg("FW2 is valid\n");

		} else {
			ccgx_fw_pr_dbg("FW2 is not valid\n");
			return -12;
		}
	}

	else {
		ccgx_fw_pr_dbg("Skipped FW2 validation as FW2 is running\n");
	}

	ccgx_fw_pr_dbg("\n");

	// If in firmware mode, disable PD ports first
	if ((mode == 1) || (mode == 2)) {
		ccgx_fw_pr_dbg("Disabling PD ports before doing device reset\n");
		stat = CyUsPDPortDisable(handle_ccg4);
		if (!stat)
			return -13;
	}
	// Reset CCG4 and see whether it comes up again.
	ccgx_fw_pr_dbg("Resetting device to start new firmware\n");

#if CCGX_FW_UPGRADE_FAST
	stat = CyUsDeviceReset(handle_ccg4, 120);
#else
	stat = CyUsDeviceReset(handle_ccg4, 1000);
#endif
	if (!stat)
		return -14;

	stat = CyUsGetFwInfo(handle_ccg4, &mode);
	if (!stat)
		return -15;
	ccgx_fw_pr_dbg("Program execution completed\n");

	return 0;

}
#endif

int ccgx_upgrade_fw_bl(int fw_idx,bool second_fw)
{
	bool stat;
	int retval;
	uint8_t mode, resp;
	static uint8_t flashbuf[256];
#if CCGX_FW_UPGRADE_FAST == 0
	static uint8_t readbuf[256];
#endif
	char line[CYACD_MAX_LINE_LEN];
	unsigned char fw_ver[16];

	unsigned f_offset;
	unsigned offset;
	unsigned f_size;
	unsigned fw1_loc;
	unsigned fw2_loc;
	unsigned nr_row_meta;

	handle_ccg4 = CyFindDevices();
	if (handle_ccg4 == NULL) {
		return CCGX_ERROR_HANDLE;
	}

	stat = CyUsGetFwInfo(handle_ccg4, &mode);
	if (!stat)
		return CCGX_ERROR_FIRM_INFO;
	ccgx_fw_pr_dbg("flash row size : %d\n", flash_row_size);

	stat = CyUsReadResponse(handle_ccg4, &resp);
	if (!stat)
		return CCGX_ERROR_RESPONSE;

	if(second_fw == false){
	retval = ccgx_enter_bl_mode();
	if (retval < 0) {
		ccgx_fw_pr_dbg("enter BL mode error : %d\n", retval);
		return CCGX_ERROR_BL_MODE;
	} else {
		ccgx_fw_pr("ENTER BL mode Okay\n");
	}
	}

	memset(fw_ver, 0x00, sizeof(fw_ver));
	ccgx_get_fw_version(fw_idx + 1, fw_ver);
	ccgx_fw_pr("UPGRADE FW%d version : 0x%02x 0x%02x 0x%02x 0x%02x \t 0x%02x 0x%02x 0x%02x 0x%02x\n", fw_idx + 1,
			fw_ver[0], fw_ver[1],  fw_ver[2],  fw_ver[3],  fw_ver[4], fw_ver[5],  fw_ver[6],  fw_ver[7]);

	retval =  ccgx_get_fw_offset(&fw1_loc, &fw2_loc);
	if (retval < 0 ) {
		ccgx_fw_pr_dbg("GET FW loc error : %d\n", retval);
		return CCGX_ERROR_FW_OFFSET;
	}
	ccgx_fw_pr_dbg("fw1_loc = 0x%08x, fw2_loc = 0x%08x\n", fw1_loc, fw2_loc);
	if (fw1_loc != 0x0014 || fw2_loc != 0x0100) {
		ccgx_fw_pr_dbg("CHECK FW loc error\n");
		return CCGX_ERROR_FW_LOC;
	}
	if(second_fw == false){
		stat = CyUsEnterFlashingMode(handle_ccg4);
		if (!stat)
			return CCGX_ERROR_FLASH_MODE;
		ccgx_fw_pr("ENTER flash mode okay\n");
	}
	ccgx_get_fw_info(fw_idx, &offset, &f_size);
	ccgx_fw_pr("FW%d at 0x%08x@0x%08x\n", fw_idx + 1, f_size,  offset);
	ccgx_fw_pr("Upgrade FW start, maybe take many seconds......\n\n");

	nr_row_meta = 0x20000U/flash_row_size - 2;
#if 0
	/*
	 *   clear
	 */
	memset(flashbuf, 0x00, sizeof(flashbuf));
	stat = CyUsWriteFlashRow(handle_ccg4,  nr_row_meta + !fw_idx, flashbuf);
	if (!stat) {
		ccgx_fw_pr_dbg("CLEAR meta error : %d\n", stat);
		return -16;
	} else {
		ccgx_fw_pr_dbg("CLEAR meta Okay\n");
	}
#endif

	f_offset = offset;
	// Read row data and program to device.
	while (1) {

		if (offset - f_offset >= f_size)
			break;

		memset(line, 0xff, sizeof(line));
		if (ccgx_fw_fgets((uint8_t *) line, CYACD_MAX_LINE_LEN, &offset) < 0 ) {
			ccgx_fw_pr_dbg("FW image corrupted\n");
			return CCGX_ERROR_IMAGE_LINE;
		}

		ccgx_fw_pr_dbg("offset = 0x%08x\n", offset);

		if (line[0] == ':') {

			uint16_t row =
				((GetHexVal(line[3]) << 12) |
				 (GetHexVal(line[4]) << 8) |
				 (GetHexVal(line[5]) << 4) | (GetHexVal(line[6])));

			uint16_t len =
				((GetHexVal(line[7]) << 12) |
				 (GetHexVal(line[8]) << 8) |
				 (GetHexVal(line[9]) << 4) | (GetHexVal(line[10])));

			memset(flashbuf, 0xff, sizeof(flashbuf));

			for (int i = 0; i < len; i++) {

				flashbuf[i] =
					((GetHexVal(line[11 + 2 * i]) << 4) |
					 (GetHexVal(line[12 + 2 * i])));

			}
#if 0
			if (row < nr_row_meta) {
				if (fw_idx == 0 && row >= fw2_loc)
					row -= (fw2_loc - fw1_loc);
				else if (fw_idx == 1 && row < fw2_loc)
					row += (fw2_loc - fw1_loc);
			} else {
				row = nr_row_meta + 1 - fw_idx;
			}
#else
			if (row < nr_row_meta) {
				if (fw_idx == 0 && row >= fw2_loc) {
					ccgx_fw_pr_dbg("FW %d Row 0x%08x mismatch\n", fw_idx + 1, row);
					return CCGX_ERROR_IMAGE1_ROW;
				} else if (fw_idx == 1 && row < fw2_loc) {
					ccgx_fw_pr_dbg("FW %d Row 0x%08x mismatch\n", fw_idx + 1, row);
					return CCGX_ERROR_IMAGE2_ROW;
				}
			} else if (row != (nr_row_meta + 1 - fw_idx)) {
				ccgx_fw_pr_dbg("FW %d Row 0x%08x mismatch\n", fw_idx + 1, row);
				return CCGX_ERROR_IMAGE_ROW;
			}
#endif

			stat = CyUsWriteFlashRow(handle_ccg4, row, flashbuf);

			if (stat) {

				ccgx_fw_pr_dbg("Finished writing flash row %x\n", row);
			}

			else {

				ccgx_fw_pr_dbg("Error: Flash write to row 0x%x failed\n",
						row);
				return CCGX_ERROR_FLASH_ROW;
			}

#if CCGX_FW_UPGRADE_FAST == 0
			memset(readbuf, 0xff, sizeof(readbuf));

			stat = CyUsReadFlashRow(handle_ccg4, row, readbuf);

			if (stat) {

				if (memcmp(readbuf, flashbuf, flash_row_size)
						== 0) {

					ccgx_fw_pr_dbg
						("Flash read-verify on row %x successful\n",
						 row);

				}

				else {

					if ((flash_row_size == 256 && row < 510)
							||
							(flash_row_size == 128
							 && row < 1022)) {

						ccgx_fw_pr_dbg ("Error: Flash data miscompare on row %x row\n",row);
						return CCGX_ERROR_VERIFY_ROW;
					}
				}

			}

			else {

				ccgx_fw_pr_dbg("Error: Flash read to row %x failed\n",
						row);
				return CCGX_ERROR_READ_ROW;

			}
#endif
		}
		else {

			ccgx_fw_pr_dbg("Skipped line before @%x\n", offset);

		}

	}

	ccgx_fw_pr("Finished updating flash. Validating firmware\n");

	if (CyUsValidateFirmware(handle_ccg4, fw_idx + 1))  {

		ccgx_fw_pr("FW%d is valid\n",  fw_idx + 1);
	} else {

		ccgx_fw_pr_dbg("FW%d is not valid\n", fw_idx + 1);
		return CCGX_ERROR_VERIFY_FW;
	}

	ccgx_fw_pr("Upgrade FW%d successfully\n", fw_idx + 1);

#if 0
	// If in firmware mode, disable PD ports first
	ccgx_fw_pr_dbg("Disabling PD ports before doing device reset\n");
	stat = CyUsPDPortDisable(handle_ccg4);
	if (!stat)
		return -13;
#endif
	// Reset CCG4 and see whether it comes up again.

	if( second_fw==true){
	ccgx_fw_pr("Resetting device to start new firmware\n");
#if CCGX_FW_UPGRADE_FAST
	stat = CyUsDeviceReset(handle_ccg4, 4);
#else
	stat = CyUsDeviceReset(handle_ccg4, 1000);
#endif
	if (!stat)
		return CCGX_ERROR_RESET_ERROR;
	}
#if 0
	stat = CyUsGetFwInfo(handle_ccg4, &mode);
	if (!stat)
		return -15;
	ccgx_fw_pr_dbg("Program execution completed\n");
#endif
	return 0;
}

void ccgx_dump_fw_info(void)
{
	bool stat;
	int idx;
	uint8_t buf[64];
	uint16_t len = 0;

	memset(buf, 0x00, sizeof(buf));
	stat = CyUsRegisterRead(CyFindDevices(), CCG4_SLAVE_I2C_ADDR, CCG_REG_DEVICE_MODE,
			64, buf, &len);

	if (stat != true)
		return ;

	printf("DEVICE_MODE : 0x%02x\n", buf[0]);
	printf("BOOT_MODE_REASON : 0x%02x\n", buf[1]);
	printf("READ_SILICON_ID : 0x%02x 0x%02x\n", buf[2], buf[3]);
	printf("BOOT_LOADER_LAST_ROW : 0x%02x 0x%02x\n", buf[4], buf[5]);

	idx = 0x0010;
	printf("BL VERSION : 0x%02x 0x%02x 0x%02x 0x%02x \t 0x%02x 0x%02x 0x%02x 0x%02x\n",
			buf[idx], buf[idx + 1], buf[idx + 2], buf[idx + 3], buf[idx + 4], buf[idx + 5], buf[idx + 6], buf[idx +7 ]);

	idx = 0x0018;
	printf("FW1 VERSION : 0x%02x 0x%02x 0x%02x 0x%02x \t 0x%02x 0x%02x 0x%02x 0x%02x\n",
			buf[idx], buf[idx + 1], buf[idx + 2], buf[idx + 3], buf[idx + 4], buf[idx + 5], buf[idx + 6], buf[idx +7 ]);

	idx = 0x0020;
	printf("FW2 VERSION : 0x%02x 0x%02x 0x%02x 0x%02x \t 0x%02x 0x%02x 0x%02x 0x%02x\n",
			buf[idx], buf[idx + 1], buf[idx + 2], buf[idx + 3], buf[idx + 4], buf[idx + 5], buf[idx + 6], buf[idx +7 ]);

	idx = 0x0028;
	printf ("FIRMWARE_BINARY_LOCATION : 0x%02x 0x%02x 0x%02x 0x%02x\n",
			buf[idx], buf[idx + 1], buf[idx + 2], buf[idx + 3]);

	printf("READ_CUSTOM_INFO : 0x%02x\n", buf[0x30]);

	return ;
}

int ccgx_upgrade_fw(int fw_idx)
{
	if (fw_idx != 0 && fw_idx != 1) {
		ccgx_fw_pr("in %s : fw_idx %d error\n", __func__, fw_idx + 1);
		return CCGX_ERROR_FIRM_INDEX;
	}

	//return ccgx_upgrade_fw_bl(fw_idx);
	return 0;
}
