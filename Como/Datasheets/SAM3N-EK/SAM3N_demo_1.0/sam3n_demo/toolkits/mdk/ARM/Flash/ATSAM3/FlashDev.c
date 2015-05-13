/******************************************************************************/
/*  This file is part of the ARM Toolchain package                            */
/*  Copyright KEIL ELEKTRONIK GmbH 2003 - 2009                                */
/******************************************************************************/
/*                                                                            */
/*  FlashDev.C:  Device Description for ATSAM3  Flashes                       */
/*                                                                            */
/******************************************************************************/

#include "..\FlashOS.H"         // FlashOS Structures


#ifdef FLASH_MEM

#ifdef ATSAM3U_128
struct FlashDevice const FlashDevice  =  {
   FLASH_DRV_VERS,              // Driver Version, do not modify!
   "ATSAM3U Flash",             // Device Name
   ONCHIP,                      // Device Type
   0x00080000,                  // Device Start Address
   0x00020000,                  // Device Size in Bytes (128kB)
   256,                         // Programming Page Size
   0,                           // Reserved, must be 0
   0xFF,                        // Initial Content of Erased Memory
   100,                         // Program Page Timeout 100 mSec
   1000,                        // Erase Sector Timeout 1000 mSec

// Specify Size and Address of Sectors
   0x0100, 0x000000,            // Sector Size 256B (512 Sectors)
   SECTOR_END
};
#endif

#ifdef ATSAM3U_128_B1
struct FlashDevice const FlashDevice  =  {
   FLASH_DRV_VERS,              // Driver Version, do not modify!
   "ATSAM3U Bank 1 Flash",      // Device Name
   ONCHIP,                      // Device Type
   0x00100000,                  // Device Start Address
   0x00020000,                  // Device Size in Bytes (128kB)
   256,                         // Programming Page Size
   0,                           // Reserved, must be 0
   0xFF,                        // Initial Content of Erased Memory
   100,                         // Program Page Timeout 100 mSec
   1000,                        // Erase Sector Timeout 1000 mSec

// Specify Size and Address of Sectors
   0x0100, 0x000000,            // Sector Size 256B (512 Sectors)
   SECTOR_END
};
#endif

#ifdef ATSAM3S_256
struct FlashDevice const FlashDevice  =  {
   FLASH_DRV_VERS,              // Driver Version, do not modify!
   "ATSAM3S Flash",             // Device Name
   ONCHIP,                      // Device Type
   0x00400000,                  // Device Start Address
   0x00040000,                  // Device Size in Bytes (256kB)
   256,                         // Programming Page Size
   0,                           // Reserved, must be 0
   0xFF,                        // Initial Content of Erased Memory
   100,                         // Program Page Timeout 100 mSec
   1000,                        // Erase Sector Timeout 1000 mSec

// Specify Size and Address of Sectors
   0x0100, 0x000000,            // Sector Size 256B (1024 Sectors)
   SECTOR_END
};
#endif

#ifdef ATSAM3N_256
struct FlashDevice const FlashDevice  =  {
   FLASH_DRV_VERS,              // Driver Version, do not modify!
   "ATSAM3N Flash",             // Device Name
   ONCHIP,                      // Device Type
   0x00400000,                  // Device Start Address
   0x00040000,                  // Device Size in Bytes (256kB)
   256,                         // Programming Page Size
   0,                           // Reserved, must be 0
   0xFF,                        // Initial Content of Erased Memory
   100,                         // Program Page Timeout 100 mSec
   1000,                        // Erase Sector Timeout 1000 mSec

// Specify Size and Address of Sectors
   0x0100, 0x000000,            // Sector Size 256B (1024 Sectors)
   SECTOR_END
};
#endif

#endif


#ifdef FLASH_GPNVM

#ifdef ATSAM3U_128
struct FlashDevice const FlashDevice  =  {
   FLASH_DRV_VERS,              // Driver Version, do not modify!
   "ATSAM3U GPNVM bits",        // Device Name
   ONCHIP,                      // Device Type
   0x1FFFFFF0,                  // Device Start Address
   0x00000010,                  // Device Size in Bytes (16)
   16,                          // Programming Page Size
   0,                           // Reserved, must be 0
   0xFF,                        // Initial Content of Erased Memory
   100,                         // Program Page Timeout 100 mSec
   1000,                        // Erase Sector Timeout 1000 mSec

// Specify Size and Address of Sectors
   0x010, 0x000000,             // Sector Size 16B
   SECTOR_END
};
#endif

#ifdef ATSAM3S_256
struct FlashDevice const FlashDevice  =  {
   FLASH_DRV_VERS,              // Driver Version, do not modify!
   "ATSAM3S GPNVM bits",        // Device Name
   ONCHIP,                      // Device Type
   0x1FFFFFF0,                  // Device Start Address
   0x00000010,                  // Device Size in Bytes (16)
   16,                          // Programming Page Size
   0,                           // Reserved, must be 0
   0xFF,                        // Initial Content of Erased Memory
   100,                         // Program Page Timeout 100 mSec
   1000,                        // Erase Sector Timeout 1000 mSec

// Specify Size and Address of Sectors
   0x010, 0x000000,             // Sector Size 16B
   SECTOR_END
};
#endif

#ifdef ATSAM3N_256
struct FlashDevice const FlashDevice  =  {
   FLASH_DRV_VERS,              // Driver Version, do not modify!
   "ATSAM3N GPNVM bits",        // Device Name
   ONCHIP,                      // Device Type
   0x1FFFFFF0,                  // Device Start Address
   0x00000010,                  // Device Size in Bytes (16)
   16,                          // Programming Page Size
   0,                           // Reserved, must be 0
   0xFF,                        // Initial Content of Erased Memory
   100,                         // Program Page Timeout 100 mSec
   1000,                        // Erase Sector Timeout 1000 mSec

// Specify Size and Address of Sectors
   0x010, 0x000000,             // Sector Size 16B
   SECTOR_END
};
#endif

#endif
