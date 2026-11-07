# CH32V30x (QingKe V4F, hardware single-precision float): the SDK in platform/.
set(XRDAP_PLATFORM_DIR "${XRDAP_ROOT}/platform")
set(XRDAP_CHIP_DEFINE CH32V30x_D8C)
set(XRDAP_CHIP_CONFIG_FILE ch32v30x_conf.h)
set(XRDAP_STARTUP_FILE startup_ch32v30x_D8C.S)
set(XRDAP_ARCH_FLAGS -march=rv32imcf_zicsr_zifencei_zaamo -mabi=ilp32f)
set(XRDAP_MULTILIB_FLAGS -march=rv32imafc_zaamo_zalrsc -mabi=ilp32f)
