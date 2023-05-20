
#ifndef EMMC_MTK_IOCTL_H__
#define EMMC_MTK_IOCTL_H__

#include <linux/mmc/ioctl.h>
#define MMC_BLOCK_MAJOR         179
#define MMC_SWITCH              6
#define MMC_SEND_EXT_CSD        8

/*
* EXT_CSD fields
*/
#define EXT_CSD_PART_CONFIG		179

/**/
typedef enum {
	EMMC_PART_UNKNOWN=0
	,EMMC_PART_BOOT1
	,EMMC_PART_BOOT2
	,EMMC_PART_RPMB
	,EMMC_PART_GP1
	,EMMC_PART_GP2
	,EMMC_PART_GP3
	,EMMC_PART_GP4
	,EMMC_PART_USER
	,EMMC_PART_END
} Region;

/* Copied from kernel linux/mmc/core.h */
#define MMC_RSP_OPCODE          (1 << 4) /* response contains opcode */
#define MMC_RSP_BUSY            (1 << 3) /* card may send busy */
#define MMC_RSP_CRC             (1 << 2) /* expect valid crc */
#define MMC_RSP_136             (1 << 1) /* 136 bit response */
#define MMC_RSP_PRESENT         (1 << 0)

#define MMC_CMD_ADTC            (1 << 5)
#define MMC_CMD_AC              (0 << 5)

#define MMC_RSP_R1  (MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE)
#define MMC_RSP_R1B (MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE|MMC_RSP_BUSY)

/* Copied from kernel linux/mmc/mmc.h */
#define EXT_CSD_CMD_SET_NORMAL		(1<<0)
#define EXT_CSD_CMD_SET_SECURE		(1<<1)
#define EXT_CSD_CMD_SET_CPSECURE	(1<<2)

#define MMC_SWITCH_MODE_CMD_SET		0x00	/* Change the command set */
#define MMC_SWITCH_MODE_SET_BITS	0x01	/* Set bits which are 1 in value */
#define MMC_SWITCH_MODE_CLEAR_BITS	0x02	/* Clear bits which are 1 in value */
#define MMC_SWITCH_MODE_WRITE_BYTE	0x03	/* Set target to value */

#endif
