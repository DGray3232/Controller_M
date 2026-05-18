
# Consider dependencies only in project.
set(CMAKE_DEPENDS_IN_PROJECT_ONLY OFF)

# The set of languages for which implicit dependencies are needed:
set(CMAKE_DEPENDS_LANGUAGES
  "ASM"
  )
# The set of files for implicit dependencies of each language:
set(CMAKE_DEPENDS_CHECK_ASM
  "/root/Controller_M/startup_stm32f411xe.s" "/root/Controller_M/Debug/CMakeFiles/Controller_M.dir/startup_stm32f411xe.s.o"
  )
set(CMAKE_ASM_COMPILER_ID "GNU")

# Preprocessor definitions for this target.
set(CMAKE_TARGET_DEFINITIONS_ASM
  "ARM_MATH_CM4"
  "DEBUG"
  "STM32F411xE"
  "USE_HAL_DRIVER"
  )

# The include file search paths:
set(CMAKE_ASM_TARGET_INCLUDE_PATH
  "/root/Controller_M/cmake/stm32cubemx/../../Core/Inc"
  "/root/Controller_M/cmake/stm32cubemx/../../Drivers/STM32F4xx_HAL_Driver/Inc"
  "/root/Controller_M/cmake/stm32cubemx/../../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy"
  "/root/Controller_M/cmake/stm32cubemx/../../Drivers/CMSIS/Device/ST/STM32F4xx/Include"
  "/root/Controller_M/cmake/stm32cubemx/../../Drivers/CMSIS/Include"
  "/root/Controller_M/cmake/stm32cubemx/../../Drivers/CMSIS/DSP/Include"
  )

# The set of dependency files which are needed:
set(CMAKE_DEPENDS_DEPENDENCY_FILES
  "/root/Controller_M/Core/Src/AHRSAlgorithms.c" "CMakeFiles/Controller_M.dir/Core/Src/AHRSAlgorithms.c.o" "gcc" "CMakeFiles/Controller_M.dir/Core/Src/AHRSAlgorithms.c.o.d"
  "/root/Controller_M/Core/Src/bmx055.c" "CMakeFiles/Controller_M.dir/Core/Src/bmx055.c.o" "gcc" "CMakeFiles/Controller_M.dir/Core/Src/bmx055.c.o.d"
  "/root/Controller_M/Core/Src/calculate_notch_coeffs.c" "CMakeFiles/Controller_M.dir/Core/Src/calculate_notch_coeffs.c.o" "gcc" "CMakeFiles/Controller_M.dir/Core/Src/calculate_notch_coeffs.c.o.d"
  "/root/Controller_M/Core/Src/dma.c" "CMakeFiles/Controller_M.dir/Core/Src/dma.c.o" "gcc" "CMakeFiles/Controller_M.dir/Core/Src/dma.c.o.d"
  "/root/Controller_M/Core/Src/globals.c" "CMakeFiles/Controller_M.dir/Core/Src/globals.c.o" "gcc" "CMakeFiles/Controller_M.dir/Core/Src/globals.c.o.d"
  "/root/Controller_M/Core/Src/gpio.c" "CMakeFiles/Controller_M.dir/Core/Src/gpio.c.o" "gcc" "CMakeFiles/Controller_M.dir/Core/Src/gpio.c.o.d"
  "/root/Controller_M/Core/Src/i2c.c" "CMakeFiles/Controller_M.dir/Core/Src/i2c.c.o" "gcc" "CMakeFiles/Controller_M.dir/Core/Src/i2c.c.o.d"
  "/root/Controller_M/Core/Src/main.c" "CMakeFiles/Controller_M.dir/Core/Src/main.c.o" "gcc" "CMakeFiles/Controller_M.dir/Core/Src/main.c.o.d"
  "/root/Controller_M/Core/Src/median_moving_average_filter.c" "CMakeFiles/Controller_M.dir/Core/Src/median_moving_average_filter.c.o" "gcc" "CMakeFiles/Controller_M.dir/Core/Src/median_moving_average_filter.c.o.d"
  "/root/Controller_M/Core/Src/mtf02.c" "CMakeFiles/Controller_M.dir/Core/Src/mtf02.c.o" "gcc" "CMakeFiles/Controller_M.dir/Core/Src/mtf02.c.o.d"
  "/root/Controller_M/Core/Src/optical_flow_compensation.c" "CMakeFiles/Controller_M.dir/Core/Src/optical_flow_compensation.c.o" "gcc" "CMakeFiles/Controller_M.dir/Core/Src/optical_flow_compensation.c.o.d"
  "/root/Controller_M/Core/Src/pid.c" "CMakeFiles/Controller_M.dir/Core/Src/pid.c.o" "gcc" "CMakeFiles/Controller_M.dir/Core/Src/pid.c.o.d"
  "/root/Controller_M/Core/Src/remote_control_mavlink.c" "CMakeFiles/Controller_M.dir/Core/Src/remote_control_mavlink.c.o" "gcc" "CMakeFiles/Controller_M.dir/Core/Src/remote_control_mavlink.c.o.d"
  "/root/Controller_M/Core/Src/stm32f4xx_hal_msp.c" "CMakeFiles/Controller_M.dir/Core/Src/stm32f4xx_hal_msp.c.o" "gcc" "CMakeFiles/Controller_M.dir/Core/Src/stm32f4xx_hal_msp.c.o.d"
  "/root/Controller_M/Core/Src/stm32f4xx_it.c" "CMakeFiles/Controller_M.dir/Core/Src/stm32f4xx_it.c.o" "gcc" "CMakeFiles/Controller_M.dir/Core/Src/stm32f4xx_it.c.o.d"
  "/root/Controller_M/Core/Src/syscalls.c" "CMakeFiles/Controller_M.dir/Core/Src/syscalls.c.o" "gcc" "CMakeFiles/Controller_M.dir/Core/Src/syscalls.c.o.d"
  "/root/Controller_M/Core/Src/sysmem.c" "CMakeFiles/Controller_M.dir/Core/Src/sysmem.c.o" "gcc" "CMakeFiles/Controller_M.dir/Core/Src/sysmem.c.o.d"
  "/root/Controller_M/Core/Src/tim.c" "CMakeFiles/Controller_M.dir/Core/Src/tim.c.o" "gcc" "CMakeFiles/Controller_M.dir/Core/Src/tim.c.o.d"
  "/root/Controller_M/Core/Src/usart.c" "CMakeFiles/Controller_M.dir/Core/Src/usart.c.o" "gcc" "CMakeFiles/Controller_M.dir/Core/Src/usart.c.o.d"
  "/root/Controller_M/Core/Src/vibration_analysis.c" "CMakeFiles/Controller_M.dir/Core/Src/vibration_analysis.c.o" "gcc" "CMakeFiles/Controller_M.dir/Core/Src/vibration_analysis.c.o.d"
  )

# Targets to which this target links which contain Fortran sources.
set(CMAKE_Fortran_TARGET_LINKED_INFO_FILES
  )

# Targets to which this target links which contain Fortran sources.
set(CMAKE_Fortran_TARGET_FORWARD_LINKED_INFO_FILES
  )

# Fortran module output directory.
set(CMAKE_Fortran_TARGET_MODULE_DIR "")
