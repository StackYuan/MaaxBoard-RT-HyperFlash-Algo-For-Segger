/*
 * The Clear BSD License
 * Copyright 2018 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the
 * disclaimer below) provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
 * GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
 * HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "bl_api.h"

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
static bootloader_api_entry_t *g_bootloaderTree;

/*******************************************************************************
 * Codes
 ******************************************************************************/

void bl_api_init(void)
{
    g_bootloaderTree = (bootloader_api_entry_t *)*(uint32_t *)0x0021001c;
}

__root void runBootloader(void* arg)
{
  g_bootloaderTree-> runBootloader(arg);
}
/*******************************************************************************
 * FlexSPI NOR driver
 ******************************************************************************/
status_t flexspi_nor_flash_init(uint32_t instance,
                                flexspi_nor_config_t *config)
{
  return g_bootloaderTree->flexSpiNorDriver->init(instance, config);
}

status_t flexspi_nor_flash_page_program(uint32_t instance,
                                        flexspi_nor_config_t *config,
                                        uint32_t dstAddr,
                                        const uint32_t *src)
{
    return g_bootloaderTree->flexSpiNorDriver->page_program(instance, config, dstAddr, src);
}

status_t flexspi_nor_flash_erase_all(uint32_t instance,
                                     flexspi_nor_config_t *config)
{
    return g_bootloaderTree->flexSpiNorDriver->erase_all(instance, config);
}

status_t flexspi_nor_get_config(uint32_t instance,
                                flexspi_nor_config_t *config,
                                serial_nor_config_option_t *option)
{
  status_t status;

  status = g_bootloaderTree->flexSpiNorDriver->get_config(instance, config, option);
  printf("flexspi_nor_get_config option= 0x%X\r\n", option->option0.U);  
  /*********************************Constyu Start************************************/
  config->memConfig.lookupTable[20] = 0x082004dc;
  config->sectorSize = 0x40000;
  /*********************************Constyu end**************************************/  
  return status;  
  
  //return g_bootloaderTree->flexSpiNorDriver->get_config(instance, config, option);
}

status_t flexspi_nor_flash_erase(uint32_t instance,
                                 flexspi_nor_config_t *config,
                                 uint32_t start,
                                 uint32_t length)
{
    return g_bootloaderTree->flexSpiNorDriver->erase(instance, config, start, length);
}

status_t flexspi_nor_flash_read(uint32_t instance,
                                flexspi_nor_config_t *config,
                                uint32_t *dst,
                                uint32_t start,
                                uint32_t bytes)
{
    return g_bootloaderTree->flexSpiNorDriver->read(instance, config, dst, start, bytes);
}

status_t flexspi_update_lut(uint32_t instance,
                            uint32_t seqIndex,
                            const uint32_t *lutBase,
                            uint32_t numberOfSeq)
{
    return g_bootloaderTree->flexSpiNorDriver->update_lut(instance, seqIndex, lutBase, numberOfSeq);
}

status_t flexspi_command_xfer(uint32_t instance,
                              flexspi_xfer_t *xfer)
{
    return g_bootloaderTree->flexSpiNorDriver->xfer(instance, xfer);
}

void flexspi_clear_cache(uint32_t instance)
{
    g_bootloaderTree->flexSpiNorDriver->clear_cache(instance);
}

status_t flexspi_nor_set_clock_source(uint32_t instance,
                                      uint32_t source)
{
  return g_bootloaderTree->flexSpiNorDriver->set_clock_source(instance, source);
}

status_t flexspi_nor_configure_clock(uint32_t instance,
                                     uint32_t freq,
                                     uint32_t sampleClkMode)
{
  return g_bootloaderTree->flexSpiNorDriver->config_clock(instance, freq, sampleClkMode);
}

status_t flexspi_nor_flash_erase_sector(uint32_t instance,
                                        flexspi_nor_config_t *config,
                                        uint32_t start)
{
    return g_bootloaderTree->flexSpiNorDriver->erase_sector(instance, config, start);
}

