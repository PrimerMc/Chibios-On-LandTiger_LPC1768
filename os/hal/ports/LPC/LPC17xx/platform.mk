# List of all the LPC17xx platform files.
PLATFORMSRC = ${CHIBIOS}/os/hal/ports/common/ARMCMx/nvic.c \
			  ${CHIBIOS}/os/hal/ports/LPC/LPC17xx/hal_lld.c \
              ${CHIBIOS}/os/hal/ports/LPC/LPC17xx/hal_pal_lld.c \
              ${CHIBIOS}/os/hal/ports/LPC/LPC17xx/hal_serial_lld.c \
              ${CHIBIOS}/os/hal/ports/LPC/LPC17xx/hal_st_lld.c \
              ${CHIBIOS}/os/hal/ports/LPC/LPC17xx/system_LPC17xx.c

# Required include directories
PLATFORMINC = $(CHIBIOS)/os/hal/ports/common/ARMCMx \
			  $(CHIBIOS)/os/hal/ports/LPC/LPC17xx
