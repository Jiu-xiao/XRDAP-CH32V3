// SPDX-License-Identifier: Apache-2.0
#ifndef XRDAP_CHIP_UID_H_
#define XRDAP_CHIP_UID_H_

/*
 * The CH32V307's 96-bit unique id (RM, electronic signature): the probe serial and the
 * pairing identity are made from it.
 * CH32V307 的 96 位唯一标识（RM 电子签名一节）：探针序列号和配对身份由它得出。
 */
#define XRDAP_CHIP_UID_ADDR 0x1FFFF7E8u
#define XRDAP_CHIP_UID_SIZE 12u

#endif /* XRDAP_CHIP_UID_H_ */
