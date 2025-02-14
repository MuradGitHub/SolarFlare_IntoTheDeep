package org.firstinspires.ftc.teamcode.base.config;

import java.util.HashMap;
import java.util.logging.Logger;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.base.logging.RobotLogger;

public class HardwareConfig implements Validatable {
    private static HardwareConfig                 hardwareConfig;

    private final  Logger                         logger;
    public  final  RobotConfig                    robotConfig;
    public  final  RobotDimensions                robotDimensions;
    private final  HashMap<MotorEnum,MotorConfig> motorConfigs;
    private final  HashMap<ServoEnum,ServoConfig> servoConfigs;
    public  final  IMUConfig                      imuConfig;
    public  final  PinpointConfig                 pinpointConfig;
    public  final  LimelightConfig                limelightConfig;

    private HardwareConfig(HardwareMap hardwareMap, RobotConfig robotConfig_in) {
        logger            = RobotLogger.getInstance().getConfigLogger();
        robotConfig       = robotConfig_in;
        robotDimensions   = robotConfig.robotDimensions;
        motorConfigs      = robotConfig.motors;
        servoConfigs      = robotConfig.servos;
        imuConfig         = robotConfig.imu;
        pinpointConfig    = robotConfig.pinpoint;
        limelightConfig   = robotConfig.limelight;

        initialize(hardwareMap);
    }

    public void initialize(HardwareMap hardwareMap) {
        for(var motorConfig: motorConfigs.values())
            motorConfig.initialize(hardwareMap);
        for(var servoConfig: servoConfigs.values())
            servoConfig.initialize(hardwareMap);
        imuConfig.initialize(hardwareMap);
        pinpointConfig.initialize(hardwareMap);
        limelightConfig.initialize(hardwareMap);
    }

    public static HardwareConfig createInstance(HardwareMap hardwareMap, RobotConfig robotConfig) {
        hardwareConfig = new HardwareConfig(hardwareMap, robotConfig);
        return hardwareConfig;
    }

    public static HardwareConfig createInstance(HardwareMap hardwareMap, String robotName) {
        return createInstance(hardwareMap, RobotConfig.createInstance(robotName));
    }

    public static HardwareConfig getInstance() {
        if(hardwareConfig == null)
            throw new IllegalStateException("HardwareConfig has not been initialized");
        return hardwareConfig;
    }

    public boolean isValid() {
        return robotConfig.isValid();
    }

    public MotorConfig getMotorConfig(MotorEnum motorEnum) throws IllegalArgumentException {
        logger.entering("HardwareConfig", "getMotorConfig", motorEnum);
        MotorConfig motorConfig = motorConfigs.get(motorEnum);
        if (motorConfig == null){
            throw new IllegalArgumentException("Config is not present for " + motorEnum.name());
        }
        logger.exiting("HardwareConfig", "getMotorConfig", motorConfig);
        return motorConfig;
    }

    public ServoConfig getServoConfig(ServoEnum servoEnum) throws IllegalArgumentException {
        logger.entering("HardwareConfig", "getServoConfig", servoEnum);
        ServoConfig servoConfig = servoConfigs.get(servoEnum);
        if (servoConfig == null){
            throw new IllegalArgumentException("Config is not present for " + servoEnum.name());
        }
        logger.exiting("HardwareConfig", "getServoConfig", servoConfig);
        return servoConfig;
    }

    public PinpointConfig getPinpointConfig() {
        logger.entering("HardwareConfig", "getPinpointConfig");
        if(pinpointConfig == null)
            throw new IllegalStateException("PinpointConfig not initialized");
        logger.exiting("HardwareConfig", "getPinpointConfig", pinpointConfig);
        return pinpointConfig;
    }

    public IMUConfig getImuConfig() {
        logger.entering("HardwareConfig", "getImuConfig");
        if(imuConfig == null)
            throw new IllegalStateException("PinpointConfig not initialized");
        logger.exiting("HardwareConfig", "getImuConfig", imuConfig);
        return imuConfig;
    }

    public LimelightConfig getLimelightConfig() {
        logger.entering("HardwareConfig", "getLimelightConfig");
        if(limelightConfig == null)
            throw new IllegalStateException("Limelight3aConfig not initialized");
        logger.exiting("HardwareConfig", "getLimelightConfig", limelightConfig);
    return limelightConfig;}
}
