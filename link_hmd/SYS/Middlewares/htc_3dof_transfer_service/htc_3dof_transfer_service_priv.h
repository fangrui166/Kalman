#ifndef __QUATERNION_SERVICE_PRIV_H__
#define __QUATERNION_SERVICE_PRIV_H__

#ifdef __cplusplus
extern "C" {
#endif

#ifndef __ARCHDEP__TYPES
#define __ARCHDEP__TYPES
typedef unsigned char u8_t;
typedef unsigned short int u16_t;
typedef unsigned int u32_t;
typedef int i32_t;
typedef short int i16_t;
typedef signed char i8_t;
#endif /*__ARCHDEP__TYPES*/

/* for test function, enable it if need test 3dof data transfer */
//#define HTC_3DOF_POLLING_TEST_EN

#ifdef HTC_3DOF_POLLING_TEST_EN
#define HTC_3DOF_TEST_TIMER_INTERVAL	\
			(configTICK_RATE_HZ / 1000) /* 1m Sec */
#endif /* HTC_3DOF_POLLING_TEST_EN */

#ifdef __cplusplus
}
#endif

#endif /* __QUATERNION_SERVICE_PRIV_H__ */
