#ifndef _CCGX_FW_IO_H
#define _CCGX_FW_IO_H

typedef  unsigned CY_HANDLE;

CY_HANDLE 
CyFindDevices(void);

bool 
CyUsRegisterWrite(
CY_HANDLE bridge_handle, 
uint16_t slaveAddr, 
uint16_t addr,
		  
uint16_t length, 
uint8_t * txBuffer,
		  
uint16_t * dataWritten);

 
bool 
 CyUsRegisterRead(
CY_HANDLE bridge_handle, 
uint16_t slaveAddr,
			   
uint16_t addr, 
uint16_t length,
			   
uint8_t * rxBuffer, 
uint16_t * dataRead);

int ccgx_fw_i2c_read(unsigned reg_addr, unsigned char *data, unsigned count);
int ccgx_fw_i2c_write(unsigned reg_addr, unsigned char *data, unsigned count);
int ccgx_read_flash(unsigned offset, uint8_t * buff, unsigned size);
int ccgx_fw_fgets(uint8_t * buff, unsigned size, unsigned *offset);

#if 0
extern void watchdog_refresh(void);
#else
#define watchdog_refresh()
#endif

#endif //_INCLUDED_WRAPPER_H_
    
