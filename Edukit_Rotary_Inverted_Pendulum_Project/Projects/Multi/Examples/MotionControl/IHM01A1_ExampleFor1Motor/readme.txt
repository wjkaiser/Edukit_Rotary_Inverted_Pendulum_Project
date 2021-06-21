/**
  @page Motion_Control_IHM01A1_ExampleFor1Motor  
  Example of one motor driving with an L6474 (used via expansion board IHM01A1)
  
  @verbatim
  ******************** (C) COPYRIGHT 2014 STMicroelectronics *******************
  * @file    Multi/Examples/MotionControl/IHM01A1_ExampleFor1Motor/readme.txt  
  * @author  IPC Rennes
  * @version V1.10.0
  * @date    March 16th, 2018
  * @brief   Description of how to read/write L6474 registers via the L6474 FW library
  ******************************************************************************
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  @endverbatim

@par Example Description 

This example describes how to use the L6474 FW library to drive one motor.


@par Directory contents 

  - MotionControl/IHM01A1_ExampleFor1Motor/Inc/stm32f4xx_hal_conf.h    HAL configuration file for STM32F4xx 
  - MotionControl/IHM01A1_ExampleFor1Motor/Inc/stm32f3xx_hal_conf.h    HAL configuration file for STM32F3xx 
  - MotionControl/IHM01A1_ExampleFor1Motor/Inc/stm32f0xx_hal_conf.h    HAL configuration file for STM32F0xx 
  - MotionControl/IHM01A1_ExampleFor1Motor/Inc/stm32l0xx_hal_conf.h    HAL configuration file for STM32L0xx  
  - MotionControl/IHM01A1_ExampleFor1Motor/Inc/stm32f4xx_it.h          Interrupt handlers header file for STM32F4xx 
  - MotionControl/IHM01A1_ExampleFor1Motor/Inc/stm32f3xx_it.h          Interrupt handlers header file for STM32F3xx 
  - MotionControl/IHM01A1_ExampleFor1Motor/Inc/stm32f0xx_it.h          Interrupt handlers header file for STM32F0xx
  - MotionControl/IHM01A1_ExampleFor1Motor/Inc/stm32l0xx_it.h          Interrupt handlers header file for STM32L0xx  
  - MotionControl/IHM01A1_ExampleFor1Motor/Inc/main.h                  Header for main.c module  
  - MotionControl/IHM01A1_ExampleFor1Motor/Src/stm32f4xx_it.c          Interrupt handlers for STM32F4xx
  - MotionControl/IHM01A1_ExampleFor1Motor/Src/stm32f3xx_it.c          Interrupt handlers for STM32F3xx
  - MotionControl/IHM01A1_ExampleFor1Motor/Src/stm32f0xx_it.c          Interrupt handlers for STM32F0xx
  - MotionControl/IHM01A1_ExampleFor1Motor/Src/stm32l0xx_it.c          Interrupt handlers for STM32L0xx  
  - MotionControl/IHM01A1_ExampleFor1Motor/Src/main.c                  Main program
  - MotionControl/IHM01A1_ExampleFor1Motor/Src/system_stm32f4xx.c      system source file for STM32F4xx
  - MotionControl/IHM01A1_ExampleFor1Motor/Src/system_stm32f3xx.c      system source file for STM32F3xx
  - MotionControl/IHM01A1_ExampleFor1Motor/Src/system_stm32f0xx.c      system source file for STM32F0xx
  - MotionControl/IHM01A1_ExampleFor1Motor/Src/system_stm32L0xx.c      system source file for STM32L0xx  
  - MotionControl/IHM01A1_ExampleFor1Motor/Src/stm32f4xx_hal_msp.c     HAL MSP module for STM32F4xx
  - MotionControl/IHM01A1_ExampleFor1Motor/Src/stm32f3xx_hal_msp.c     HAL MSP module for STM32F3xx
  - MotionControl/IHM01A1_ExampleFor1Motor/Src/stm32f0xx_hal_msp.c     HAL MSP module for STM32F0xx
  - MotionControl/IHM01A1_ExampleFor1Motor/Src/stm32L0xx_hal_msp.c     HAL MSP module for STM32L0xx  
  - MotionControl/IHM01A1_ExampleFor1Motor/Src/clock_f4.c              clock configuration for STM32F4xx
  - MotionControl/IHM01A1_ExampleFor1Motor/Src/clock_f3.c              clock configuration for STM32F3xx
  - MotionControl/IHM01A1_ExampleFor1Motor/Src/clock_f0.c              clock configuration for STM32F0xx
  - MotionControl/IHM01A1_ExampleFor1Motor/Src/clock_l0.c              clock configuration for for STM32L0xx    

@par Hardware and Software environment

  This example requires :
    - a NUCLEO-F401RE board : a STM32 Nucleo development board for STM32 F4 series
    - or a NUCLEO-F334 board : a STM32 Nucleo development board for STM32 F3 series
    - or a NUCLEO-F030 board : a STM32 Nucleo development board for STM32 F0 series
    - or a NUCLEO-L053 board : a STM32 Nucleo development board for STM32 L0 series
    - an X-NUCLEO-IHM01A1 board: a stepper motor driver expansion board based on the L6474
    - one stepper motor connected to the X-NUCLEO-IHM01A1 board   

@par How to use it ? 

In order to make the program work, you must do the following :
 - Open your preferred toolchain 
 - Rebuild all files and load your image into target memory
 - Run the example

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
