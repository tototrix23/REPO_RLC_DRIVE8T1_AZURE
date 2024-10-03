/*
 * lfs_impl.h
 *
 *  Created on: 6 juin 2024
 *      Author: Christophe
 */

#ifndef FILES_LFS_IMPL_H_
#define FILES_LFS_IMPL_H_

#include <c_common.h>
#include <files/lfs/lfs.h>
#include "hal_data.h"

extern const char dir_payloads[];
extern const char dir_events[];
extern const char dir_json[];
extern const char dir_firmware[];

extern lfs_t lfs;
extern bool_t lfs_init_success;


int LFS_Init(bool_t format);
int LFS_ParseFolders(char *dir);
int LFS_DeInit(void);
int LFS_ImplRead(const struct lfs_config *c, lfs_block_t block,
            lfs_off_t off, void *buffer, lfs_size_t size);

int LFS_ImplProg(const struct lfs_config *c, lfs_block_t block,
            lfs_off_t off, const void *buffer, lfs_size_t size);

int LFS_ImplErase(const struct lfs_config *c, lfs_block_t block);
int LFS_ImplSync(const struct lfs_config *c);
#endif /* FILES_LFS_IMPL_H_ */
