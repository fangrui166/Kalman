#include "main.h"
#include "stm32f4xx_hal.h"
#include "flash_drv.h"
#include "bl1_check.h"
#include "htc_memory_define.h"

typedef volatile unsigned int vu32;
CRC_HandleTypeDef hcrc;

static void MX_CRC_Init(void)
{

  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }

}

int CRCCheck(void)
{
	uint32_t BL1_CRC = 0;
	uint32_t CURRBL1_CRC = 0;
	BL1_CRC = (*(vu32 *)(IMAGE_END_BL1 + 1));

	MX_CRC_Init();
	CURRBL1_CRC = HAL_CRC_Calculate(&hcrc, (uint32_t *)IMAGE_START_BL1,
					((IMAGE_END_BL1 - IMAGE_START_BL1 + 1)>>2));

	if(BL1_CRC == CURRBL1_CRC){
		return 1;
	}
	else{
		return 0;
	}
}

int isBL1FwOK(void)
{
	if(CRCCheck()){
		return 1;
	}
    return 0;
}

