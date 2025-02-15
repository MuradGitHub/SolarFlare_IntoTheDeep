package org.firstinspires.ftc.teamcode.base.testing;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.base.config.HardwareConfig;
import org.firstinspires.ftc.teamcode.base.config.MotorEnum;
import org.firstinspires.ftc.teamcode.base.config.RobotConfig;
import org.firstinspires.ftc.teamcode.base.config.RobotState;
import org.firstinspires.ftc.teamcode.base.config.ServoEnum;

import java.util.HashMap;

public class MechanismDiagnosis extends LinearOpMode {
    HardwareConfig hw;
    RobotState state;
    PinpointDrive drive;
    boolean odometryReadingsWorking;
    boolean imuReadingsWorking;
    boolean leftRightOdometryWorking;
    boolean upDownOdometryWorking;
    HashMap<MotorEnum, Boolean> motorMoving = new HashMap<>(10);
    HashMap <MotorEnum, Boolean> motorReadingsWorking = new HashMap<>(10);
    HashMap <ServoEnum, Boolean> servoMoving = new HashMap<>(10);

    @Override
    public void runOpMode() {
        RobotConfig robotConfig = RobotConfig.makeInstance("Rig1Motor");
        hw = HardwareConfig.makeInstance(hardwareMap, robotConfig);
        state = RobotState.getInstance();
        drive = new PinpointDrive(hardwareMap, new Pose2d(0,0,Math.toRadians(0)));

        for (MotorEnum motorEnum: MotorEnum.values()) {
            motorMoving.put(motorEnum, false);

            motorReadingsWorking.put(motorEnum, false);

        }
        for (ServoEnum servoEnum: ServoEnum.values()) {
            servoMoving.put(servoEnum, false);
        }
        waitForStart();

        // Move all motors individually by a small amount and check for movement
        for (MotorEnum motorEnum: MotorEnum.values()){
            hw.getMotorConfig(motorEnum).motor.setPower(0.3);
            sleep(500);
            hw.getMotorConfig(motorEnum).motor.setPower(0);

            while (Boolean.FALSE.equals(motorMoving.get(motorEnum))){
                if (gamepad1.a) {
                    try{
                        motorMoving.put(motorEnum, true);
                    }
                    catch (Exception exception){
                        throw new NullPointerException("Boolean not found for specified motorEnum");
                    }
                }
                else if (gamepad1.b){
                    break;
                }
                telemetry.addLine("Press A if " + motorEnum.toString() + "motor has moved");
                telemetry.update();
            }
        }

        // Check all motor readings individually
        for (MotorEnum motorEnum : MotorEnum.values()){
            if (hw.getMotorConfig(motorEnum).motor.getCurrentPosition() != 0)
                motorReadingsWorking.put(motorEnum, true);
            else telemetry.addData("Motor reading for " + motorEnum.toString() + "is", hw.getMotorConfig(motorEnum).motor.getCurrentPosition());
        }

        // Move all servos individually by a small amount and check for movement
        for (ServoEnum servoEnum: ServoEnum.values()){
            hw.getServoConfig(servoEnum).servo.setPosition(
                    hw.getServoConfig(servoEnum).servo.getPosition() + 15);

            while (Boolean.FALSE.equals(servoMoving.get(servoEnum))){

                if (gamepad1.a) servoMoving.put(servoEnum, true);
                else if (gamepad1.b) break;
                telemetry.addLine("Press A if " + servoEnum.toString() + "has moved");
                telemetry.update();
            }
        }

        // Move drivetrain
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(new Vector2d(5,5),Math.toRadians(90))
                        .build()
        );

        // Check odometry pods output
        while (!odometryReadingsWorking){
            if (drive.pose.position.x != 0) leftRightOdometryWorking = true;
            else telemetry.addData("X axis odometry reading:", drive.pose.position.x);

            if (drive.pose.position.y != 0) upDownOdometryWorking = true;
            else telemetry.addData("Y axis odometry reading:", drive.pose.position.y);
            if (leftRightOdometryWorking && upDownOdometryWorking) odometryReadingsWorking = true;
            if (gamepad1.b) break;
            telemetry.update();
        }

        // Check IMU readings
        while (!imuReadingsWorking){
            if (hw.getImuConfig().imu.getRobotYawPitchRollAngles().getYaw() != 0)
                imuReadingsWorking = true;
            else telemetry.addData("IMU yaw reading: ", hw.getImuConfig().imu.getRobotYawPitchRollAngles().getYaw());
            if (gamepad1.b) break;
            telemetry.update();
        }

        boolean allMotorsMoving = motorMoving.values().stream().allMatch(status -> status);
        boolean allMotorReadingsWorking = motorReadingsWorking.values().stream().allMatch(status -> status);
        boolean allServosMoving = servoMoving.values().stream().allMatch(status -> status);

        if (allMotorsMoving && allMotorReadingsWorking && allServosMoving && odometryReadingsWorking && imuReadingsWorking){
            telemetry.addLine("All systems are functional");
            telemetry.update();
        }
    }
}