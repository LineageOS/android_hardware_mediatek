#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <string>
#include <android-base/logging.h>

#if !defined(ARCH_X86)
#include "mmc-mtk-ioctl.h"
#include "ufs-mtk-ioctl.h"
#include <sys/ioctl.h>
#endif

#include "boot_region_control_private.h"
extern "C"{
    #include "mtk_ioctl.h"
}

namespace android {
namespace bootable {
#if !defined(ARCH_X86)
static bool mmc_read_extcsd(int fd, uint8_t *ext_csd)
{
  struct mmc_ioc_cmd mmc_ioctl_cmd;

  memset(ext_csd, 0, sizeof(uint8_t) * 512);
  memset(&mmc_ioctl_cmd, 0, sizeof(mmc_ioctl_cmd));
  mmc_ioctl_cmd.blocks = 1;
  mmc_ioctl_cmd.blksz = 512;
  mmc_ioctl_cmd.opcode = MMC_SEND_EXT_CSD;
  mmc_ioctl_cmd.flags = MMC_CMD_ADTC | MMC_RSP_R1;
  mmc_ioc_cmd_set_data(mmc_ioctl_cmd, ext_csd);

  if (ioctl(fd, MMC_IOC_CMD, &mmc_ioctl_cmd)) {
    LOG(ERROR) << "ioctl error, mmc_read_extcsd fail";
    return false;
  }
  return true;
}

static bool mmc_switch_bootpart(int fd, uint8_t *ext_csd, uint8_t bootpart)
{
  struct mmc_ioc_cmd mmc_ioctl_cmd;
  uint8_t val;

  val = (ext_csd[EXT_CSD_PART_CONFIG] & ~(0x38)) | (bootpart << 3);
  memset(&mmc_ioctl_cmd, 0, sizeof(mmc_ioctl_cmd));
  mmc_ioctl_cmd.opcode = MMC_SWITCH;

  mmc_ioctl_cmd.arg = (MMC_SWITCH_MODE_WRITE_BYTE << 24)
        | (EXT_CSD_PART_CONFIG << 16)
        | val << 8
        | EXT_CSD_CMD_SET_NORMAL;
  mmc_ioctl_cmd.flags = MMC_CMD_AC | MMC_RSP_R1B;
  mmc_ioc_cmd_set_data(mmc_ioctl_cmd, ext_csd);

  if (ioctl(fd, MMC_IOC_CMD, &mmc_ioctl_cmd)) {
    LOG(ERROR) << "ioctl error, mmc_switch_bootpart fail";
    return false;
  }
  return true;
}

static bool emmc_set_active_boot_part(int bootpart)
{
  uint8_t ext_csd[512];
  uint8_t cur_bootpart;
  int ret = true;

  int fd= open("/dev/block/mmcblk0", O_RDWR);
  if (fd < 0) {
     LOG(ERROR) << "open /dev/block/mmcblk0 fail";
     return false;
  }
  if (!mmc_read_extcsd(fd, ext_csd)) {
    LOG(ERROR) << "Could not read EXT_CSD";
    close(fd);
    ret = false;
  } else {
    /* check current boot part */
    cur_bootpart = (ext_csd[EXT_CSD_PART_CONFIG] >> 3) & 0x7;
    if (cur_bootpart != bootpart) {
      if (!mmc_switch_bootpart(fd, ext_csd, bootpart)) {
        LOG(ERROR) << "Could not switch boot part";
        ret = false;
      }
    }
    close(fd);
  }
  return ret;
}

static bool ufs_set_active_boot_part(int boot)
{
    struct ufs_ioctl_query_data idata;
    unsigned char buf[1];
    int fd, ret = true;

    fd = open("/dev/block/sdc", O_RDWR);
    if (fd < 0) {
        printf("%s: open device failed, err: %d\n", __func__, fd);
        return false;
    }

    buf[0] = boot;           /* 1: BootLU A, 2: BootLU B */

    idata.opcode = UPIU_QUERY_OPCODE_WRITE_ATTR;
    idata.idn = QUERY_ATTR_IDN_BOOT_LUN_EN;
    idata.idx = 0;
    idata.buf_ptr = &buf[0];
    idata.buf_byte = 1;

    if (ioctl(fd, UFS_IOCTL_QUERY, &idata) < 0) {
      LOG(ERROR) << "ufs_set boot_part old fail";
      ret = ioctrl_w_attr("/dev/ufs-bsg0", QUERY_ATTR_IDN_BOOT_LUN_EN, 0, 0, boot);
    }

    close(fd);
    return ret;
}

bool BootControlExt::SetBootRegionSlot(unsigned int slot) {
  int boot_part = 0;


  LOG(ERROR) << "setActiveBootSlot SetBootRegionSlot " << slot;
  /* slot 0 is A , slot 1 is B */
  if (slot >= 2) {
    LOG(ERROR) << "Wrong Slot value " << slot;
    return 0;
  }

  if(slot)
    boot_part = 2;
  else
    boot_part = 1;

  if (!ufs_set_active_boot_part(boot_part))
    if (!emmc_set_active_boot_part(boot_part))
      return false;

  return true;
}
#else
bool BootControlExt::SetBootRegionSlot(unsigned int slot) {
  return true;
}
#endif //#if !defined(ARCH_X86)
}
}