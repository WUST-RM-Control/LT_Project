#include "Flash.h"
#include "w25q64.h"

Config_StructTypedef Config;

void Config_Save(void)
{
    OSPI_W25Qxx_BlockErase_64K(0);
    OSPI_W25Qxx_WriteBuffer((uint8_t *)&Config,0,sizeof(Config));
}

void Config_Read(void)
{
    OSPI_W25Qxx_ReadBuffer((uint8_t *)&Config,0,sizeof(Config));
}

void Flash_Init(void)
{
    OSPI_W25Qxx_Init();   
    Config_Read();
}