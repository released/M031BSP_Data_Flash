/******************************************************************************
 * @file     DataFlashProg.h
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 18/04/03 10:23a $
 * @brief    Data flash programming driver header
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __DATA_FLASH_PROG_H__
#define __DATA_FLASH_PROG_H__


#define DATA_FLASH_BASE           (0x1F000)
#define FLASH_PAGE_SIZE           FMC_FLASH_PAGE_SIZE
#define BUFFER_PAGE_SIZE          4

void DataFlashInit(void);
void DataFlashWrite(uint32_t addr, uint32_t size, uint32_t buffer);
void DataFlashRead(uint32_t addr, uint32_t size, uint32_t buffer);

#endif  /* __DATA_FLASH_PROG_H__ */

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
