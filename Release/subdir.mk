################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../PhysicalWorld.cpp \
../scene_drawings.cpp \
../tower_destruction.cpp 

OBJS += \
./PhysicalWorld.o \
./scene_drawings.o \
./tower_destruction.o 

CPP_DEPS += \
./PhysicalWorld.d \
./scene_drawings.d \
./tower_destruction.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/usr/include/bullet/ -I/usr/include/ -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


