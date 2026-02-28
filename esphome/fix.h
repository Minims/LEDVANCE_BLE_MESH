// Workaround for ESPHome API services linker bug on ESP32-C3/C6 (RISC-V)
// See: https://github.com/esphome/issues/issues/3564#issuecomment-1519345012
// This fixes: "undefined reference to
// esphome::api::to_service_arg_type<long>()"

#ifdef __riscv

#undef __INT32_TYPE__
#define __INT32_TYPE__ int

#undef __UINT32_TYPE__
#define __UINT32_TYPE__ unsigned int

#endif // __riscv
