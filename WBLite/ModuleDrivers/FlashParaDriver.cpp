#include "FlashParaDriver.h"

#define IMUCALVALP   0x08060000 //陀螺仪零漂 ACCEL-GYRO 注意代码优化等级影响
#define IMUCALVALPLEN 24

/* 
	IMU Data is storage in 0x08060000 (BANK1 Sector 3 (H7), Sector7(F4))
	0x08060000
	Warning! 0x08060000 is not a safty sector in F446RCT6 XD
*/

void WriteIMUCal(float *GyroC, float *AccelC)
{
	/*准备数据*/	
	uint8_t pbufdata[32+IMUCALVALPLEN/32]={0};
	uint32_t pageError = 0;

	
	/*写入扇区 IMUCALVALP==0x08060000*/
	memcpy(pbufdata,	AccelC,12);
	memcpy(pbufdata+12,	GyroC , 12);
	
	/*擦除扇区 FLASH_SECTOR_7*/
	HAL_FLASH_Unlock();	
	
	FLASH_EraseInitTypeDef flash_erase;
	flash_erase.TypeErase 	= FLASH_TYPEERASE_SECTORS;
	flash_erase.Banks 		= FLASH_BANK_1;
	flash_erase.Sector 		= FLASH_SECTOR_3;
	flash_erase.NbSectors 	= 1;
	flash_erase.VoltageRange= FLASH_VOLTAGE_RANGE_3;	//3.3V
	
	HAL_FLASHEx_Erase(&flash_erase,&pageError);

	tx_thread_sleep(100);
	
	for(uint16_t i=0;i<3;i++)
	{
		__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
		FLASH_FLAG_STRBERR | FLASH_FLAG_INCERR|FLASH_FLAG_PGSERR);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, IMUCALVALP, (uint32_t)pbufdata);//写入数据 32Bytes
	}
	HAL_FLASH_Lock();
}

/*读取IMU校准数据*/
void ReadIMUCal(float *GyroC, float *AccelC)
{
	uint8_t pbufdata[24]={0};
	
	for(uint8_t i=0;i<24;i++)
	{
		pbufdata[i] = *(__IO uint8_t*)(IMUCALVALP+i);
	}
	
	memcpy(AccelC,	pbufdata,		12);
	memcpy(GyroC,	pbufdata+12,	12);
}