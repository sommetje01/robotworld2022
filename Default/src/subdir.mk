################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/AStar.cpp \
../src/AbstractActuator.cpp \
../src/AbstractAgent.cpp \
../src/AbstractSensor.cpp \
../src/BoundedVector.cpp \
../src/CommunicationService.cpp \
../src/DebugTraceFunction.cpp \
../src/Goal.cpp \
../src/GoalShape.cpp \
../src/LaserDistanceSensor.cpp \
../src/LineShape.cpp \
../src/LogTextCtrl.cpp \
../src/Logger.cpp \
../src/Main.cpp \
../src/MainApplication.cpp \
../src/MainFrameWindow.cpp \
../src/MathUtils.cpp \
../src/ModelObject.cpp \
../src/NotificationHandler.cpp \
../src/Notifier.cpp \
../src/ObjectId.cpp \
../src/Observer.cpp \
../src/RectangleShape.cpp \
../src/Robot.cpp \
../src/RobotShape.cpp \
../src/RobotWorld.cpp \
../src/RobotWorldCanvas.cpp \
../src/Shape2DUtils.cpp \
../src/StdOutDebugTraceFunction.cpp \
../src/SteeringActuator.cpp \
../src/ViewObject.cpp \
../src/Wall.cpp \
../src/WallShape.cpp \
../src/WayPoint.cpp \
../src/WayPointShape.cpp \
../src/WidgetDebugTraceFunction.cpp \
../src/Widgets.cpp 

OBJS += \
./src/AStar.o \
./src/AbstractActuator.o \
./src/AbstractAgent.o \
./src/AbstractSensor.o \
./src/BoundedVector.o \
./src/CommunicationService.o \
./src/DebugTraceFunction.o \
./src/Goal.o \
./src/GoalShape.o \
./src/LaserDistanceSensor.o \
./src/LineShape.o \
./src/LogTextCtrl.o \
./src/Logger.o \
./src/Main.o \
./src/MainApplication.o \
./src/MainFrameWindow.o \
./src/MathUtils.o \
./src/ModelObject.o \
./src/NotificationHandler.o \
./src/Notifier.o \
./src/ObjectId.o \
./src/Observer.o \
./src/RectangleShape.o \
./src/Robot.o \
./src/RobotShape.o \
./src/RobotWorld.o \
./src/RobotWorldCanvas.o \
./src/Shape2DUtils.o \
./src/StdOutDebugTraceFunction.o \
./src/SteeringActuator.o \
./src/ViewObject.o \
./src/Wall.o \
./src/WallShape.o \
./src/WayPoint.o \
./src/WayPointShape.o \
./src/WidgetDebugTraceFunction.o \
./src/Widgets.o 

CPP_DEPS += \
./src/AStar.d \
./src/AbstractActuator.d \
./src/AbstractAgent.d \
./src/AbstractSensor.d \
./src/BoundedVector.d \
./src/CommunicationService.d \
./src/DebugTraceFunction.d \
./src/Goal.d \
./src/GoalShape.d \
./src/LaserDistanceSensor.d \
./src/LineShape.d \
./src/LogTextCtrl.d \
./src/Logger.d \
./src/Main.d \
./src/MainApplication.d \
./src/MainFrameWindow.d \
./src/MathUtils.d \
./src/ModelObject.d \
./src/NotificationHandler.d \
./src/Notifier.d \
./src/ObjectId.d \
./src/Observer.d \
./src/RectangleShape.d \
./src/Robot.d \
./src/RobotShape.d \
./src/RobotWorld.d \
./src/RobotWorldCanvas.d \
./src/Shape2DUtils.d \
./src/StdOutDebugTraceFunction.d \
./src/SteeringActuator.d \
./src/ViewObject.d \
./src/Wall.d \
./src/WallShape.d \
./src/WayPoint.d \
./src/WayPointShape.d \
./src/WidgetDebugTraceFunction.d \
./src/Widgets.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Solaris C++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


