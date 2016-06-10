F3_TARGETS  += $(TARGET)
FEATURES    = VCP POLYSTACK SDCARD

TARGET_SRC = \
            drivers/accgyro_mpu.c \
            drivers/accgyro_spi_mpu6000.c \
            drivers/accgyro_mpu6500.c \
            drivers/accgyro_spi_mpu6500.c \
            drivers/accgyro_mpu6500.c \
            drivers/inverter.c
