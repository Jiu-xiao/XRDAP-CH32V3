# CH32V208 (QingKe V4C, no floating point): the SDK in platform/ch32v20x/.
set(XRDAP_PLATFORM_DIR "${XRDAP_ROOT}/platform/ch32v20x")
set(XRDAP_CHIP_DEFINE CH32V20x_D8W)
set(XRDAP_CHIP_CONFIG_FILE ch32v20x_conf.h)
set(XRDAP_STARTUP_FILE startup_ch32v20x_D8W.S)
set(XRDAP_ARCH_FLAGS -march=rv32imc_zicsr_zifencei_zaamo -mabi=ilp32)
set(XRDAP_MULTILIB_FLAGS -march=rv32imc -mabi=ilp32)
