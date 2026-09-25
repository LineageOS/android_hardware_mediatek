//
// SPDX-FileCopyrightText: The LineageOS Project
// SPDX-License-Identifier: Apache-2.0
//

#include <ion/ion.h>
#include <sys/mman.h>

int ion_set_client_name(int ion_fd, const char* name) {
    return -1;
}

int mt_ion_open(const char* name) {
    return -1;
}

int mt_ion_close(int fd) {
    return -1;
}

int ion_alloc_mm(int fd, size_t len, size_t align, unsigned int flags, ion_user_handle_t* handle) {
    return -1;
}

int ion_alloc_camera(int fd, size_t len, size_t align, unsigned int flags,
                     ion_user_handle_t* handle) {
    return -1;
}

int ion_alloc_syscontig(int fd, size_t len, size_t align, unsigned int flags,
                        ion_user_handle_t* handle) {
    return -1;
}

int ion_alloc_camera_pool(int fd, size_t len, size_t align, unsigned int flags, unsigned int* ret,
                          int cache_pool_cmd) {
    return -1;
}

void* ion_mmap(int fd, void* addr, size_t length, int prot, int flags, int share_fd, off_t offset) {
    return MAP_FAILED;
}

int ion_munmap(int fd, void* addr, size_t length) {
    return -1;
}

int ion_share_close(int fd, int share_fd) {
    return -1;
}

int ion_custom_ioctl(int fd, unsigned int cmd, void* arg) {
    return -1;
}

int ion_cache_sync_flush_all(int fd) {
    return -1;
}

int ion_cache_sync_flush_range(int fd) {
    return -1;
}

int ion_cache_sync_flush_range_va(int fd, void* addr, size_t length) {
    return -1;
}

int ion_dma_unmap_area(int fd, ion_user_handle_t handle, int dir) {
    return -1;
}

int ion_dma_map_area(int fd, ion_user_handle_t handle, int dir) {
    return -1;
}

int ion_dma_map_area_va(int fd, void* addr, size_t length, int dir) {
    return -1;
}

int ion_dma_unmap_area_va(int fd, void* addr, size_t length, int dir) {
    return -1;
}
