#include "stdio.h"
#include "usart.h"


#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

static char prev_ch;

PUTCHAR_PROTOTYPE
{
	//From \STM32Cube_FW_F4_V1.5.0\Projects\STM32F411RE-Nucleo\Examples\UART\UART_Printf\Src
	/* Place your implementation of fputc here */
	/* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */

	/* IAR NOTE*//*by Ethan*/
	/* General Options -> Library Configuration -> Library = Full*/
	/* General Options -> Library Options -> Printf formatter = Small, Scanf formatter = Small*/
	/* Runtime Checking -> C/C+= Compiler -> Optimizations = High*/
	if (ch != '\n') {
		HAL_DBG_UART_Transmit(&dbg_uart, (uint8_t *)&ch, 1, 0xFFFF);
		goto done;
	}

	if (prev_ch != '\r') {
		HAL_DBG_UART_Transmit(&dbg_uart, (uint8_t *)"\r\n", 2, 0xFFFF);
	}

done:
	prev_ch = ch;
	return ch;

	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART */
	//USART_SendData(USART2, (uint8_t) ch);

	/* Loop until the end of transmission */
	//while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
	//{}

	//return ch;
	//
	//
}
