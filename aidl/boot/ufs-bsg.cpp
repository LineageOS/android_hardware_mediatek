/*
 * Copyright (c) 2020 The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <android-base/logging.h>
#include <dirent.h>
#include <endian.h>
#include <linux/bsg.h>
#include <scsi/scsi_bsg_ufs.h>
#include <string.h>

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "ufs-bsg.h"

/* Size of the buffer that needs to be passed to the UFS ioctl */
#define UFS_ATTR_DATA_SIZE 32

#define DWORD(b3, b2, b1, b0) htobe32((b3 << 24) | (b2 << 16) | (b1 << 8) | b0)

namespace aidl::android::hardware::boot {

static int ufs_bsg_ioctl(int fd, struct ufs_bsg_request* req, struct ufs_bsg_reply* rsp, __u8* buf,
                         __u32 buf_len, enum bsg_ioctl_dir dir) {
    int ret;
    struct sg_io_v4 sg_io;
    memset(&sg_io, 0, sizeof(sg_io));

    sg_io.guard = 'Q';
    sg_io.protocol = BSG_PROTOCOL_SCSI;
    sg_io.subprotocol = BSG_SUB_PROTOCOL_SCSI_TRANSPORT;
    sg_io.request_len = sizeof(*req);
    sg_io.request = (__u64)req;
    sg_io.response = (__u64)rsp;
    sg_io.max_response_len = sizeof(*rsp);
    if (dir == BSG_IOCTL_DIR_FROM_DEV) {
        sg_io.din_xfer_len = buf_len;
        sg_io.din_xferp = (__u64)(buf);
    } else {
        sg_io.dout_xfer_len = buf_len;
        sg_io.dout_xferp = (__u64)(buf);
    }

    ret = ioctl(fd, SG_IO, &sg_io);
    if (ret) {
        LOG(ERROR) << __func__ << ": Error from sg_io ioctl (return value: " << ret
                   << ", error no: " << errno << ", reply result from LLD: " << rsp->result
                   << "\n)";
    }

    if (sg_io.info || rsp->result) {
        LOG(ERROR) << __func__ << ": Error from sg_io info (check sg info: device_status: 0x"
                   << std::hex << sg_io.device_status << ", transport_status: 0x"
                   << sg_io.transport_status << ", driver_status: 0x" << sg_io.driver_status
                   << std::dec << ", reply result from LLD: " << rsp->result << "\n)";
        ret = -EAGAIN;
    }

    return ret;
}

static void compose_ufs_bsg_query_req(struct ufs_bsg_request* req, __u8 func, __u8 opcode, __u8 idn,
                                      __u8 index, __u8 sel, __u16 length) {
    struct utp_upiu_header* hdr = &req->upiu_req.header;
    struct utp_upiu_query* qr = &req->upiu_req.qr;

    req->msgcode = UTP_UPIU_QUERY_REQ;
    hdr->dword_0 = DWORD(UTP_UPIU_QUERY_REQ, 0, 0, 0);
    hdr->dword_1 = DWORD(0, func, 0, 0);
    hdr->dword_2 = DWORD(0, 0, length >> 8, (__u8)length);
    qr->opcode = opcode;
    qr->idn = idn;
    qr->index = index;
    qr->selector = sel;
    qr->length = htobe16(length);
}

static int ufs_query_attr(int fd, __u32 value, __u8 func, __u8 opcode, __u8 idn, __u8 index,
                          __u8 sel) {
    struct ufs_bsg_request req;
    memset(&req, 0, sizeof(req));

    struct ufs_bsg_reply rsp;
    memset(&rsp, 0, sizeof(rsp));

    enum bsg_ioctl_dir dir = BSG_IOCTL_DIR_FROM_DEV;
    int ret = 0;

    if (opcode == QUERY_REQ_OP_WRITE_DESC || opcode == QUERY_REQ_OP_WRITE_ATTR)
        dir = BSG_IOCTL_DIR_TO_DEV;

    req.upiu_req.qr.value = htobe32(value);

    compose_ufs_bsg_query_req(&req, func, opcode, idn, index, sel, 0);

    ret = ufs_bsg_ioctl(fd, &req, &rsp, 0, 0, dir);
    if (ret) {
        LOG(ERROR) << __func__ << ": Error from ufs_bsg_ioctl (return value: " << ret
                   << ", error no: " << errno << "\n)";
    }

    return ret;
}

int ufs_set_boot_part(int32_t in_slot) {
    int ret, fd;
    __u32 boot_lun_id = (in_slot == 1) ? 2 : 1;

    fd = open(UFS_BSG0_DEV, O_RDWR);
    if (fd < 0) {
        LOG(ERROR) << "Failed to open " UFS_BSG0_DEV ": " << errno;
        return errno;
    }
    LOG(INFO) << "Opened ufs bsg dev: " << UFS_BSG0_DEV << "\n";

    ret = ufs_query_attr(fd, boot_lun_id, QUERY_REQ_FUNC_STD_WRITE, QUERY_REQ_OP_WRITE_ATTR,
                         QUERY_ATTR_IDN_BOOT_LU_EN, 0, 0);
    if (ret)
        LOG(ERROR) << "Error requesting ufs attr idn " << QUERY_ATTR_IDN_BOOT_LU_EN
                   << " via query ioctl (return value: " << ret << ", error no: " << errno << ")";

out:
    close(fd);
    return ret;
}

};  // namespace aidl::android::hardware::boot
