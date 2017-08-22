#ifndef __PROXIMITY_TASK_H
#define __PROXIMITY_TASK_H

#ifdef __cplusplus
extern "C" {
#endif



/* Includes ------------------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void proximityTaskInit(void);
void proximitySensorIsr(uint16_t GPIO_Pin);

#ifdef __cplusplus
}
#endif

#endif /* __PROXIMITY_TASK_H */

