#include "bootmode.h"
#include "stm32f411xe.h"
#include "stm32f4xx_hal.h"

#if (__ARMCC_VERSION)
    __attribute__ ((section(".runtype"), zero_init))
#elif defined (__GNUC__)
    __attribute__ ((section(".runtype")))
#elif defined (__ICCARM__)
    #pragma location=".runtype"
#endif /* (__ARMCC_VERSION) */
volatile unsigned int blRunMode[2];

// check if need enter dfu mode, need system set the flag and reset system
BOOT_MODE check_mode()
{
	if((RCC->CSR & RCC_CSR_SFTRSTF) == RCC_CSR_SFTRSTF && DFU_MODE == blRunMode[0]){
		printf("bl Enter DFU mode.\n");
		return DFU_MODE;
	}
	else if((RCC->CSR & RCC_CSR_SFTRSTF) == RCC_CSR_SFTRSTF && FOTA_MODE == blRunMode[0]){
		printf("bl Enter FOTA mode.\n");
		return FOTA_MODE;
	}
	else{
		printf("bl Enter NORM mode.\n");
		return NORM_MODE;
	}
}

void  enter_dfu_mode()
{
	// now dfu update will work now.
	blRunMode[0] = BL_UPDATE_MODE;
	while(1){
		if(NORM_MODE == blRunMode[0]){
			HAL_Delay(2000U);
			break;
		}
	}

}

