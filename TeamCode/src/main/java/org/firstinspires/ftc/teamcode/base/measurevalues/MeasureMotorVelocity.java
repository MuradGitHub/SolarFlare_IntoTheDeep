package org.firstinspires.ftc.teamcode.base.measurevalues;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.base.config.HardwareConfig;
import org.firstinspires.ftc.teamcode.base.config.MotorEnum;
import org.firstinspires.ftc.teamcode.base.config.RobotConfig;
import org.firstinspires.ftc.teamcode.base.config.RobotState;
import org.firstinspires.ftc.teamcode.base.motorcontrol.PID;

import java.util.Objects;

@Config
@TeleOp
public class MeasureMotorVelocity extends LinearOpMode {
    HardwareConfig hw;
    RobotState state;
    PID pid;
    public static String motorType = "extendo";
    MotorEnum motorEnum;
    double greatestSpeed = 0;
    double currentSpeed = 0;
    int lastPosition = 0;
    int currentPosition = 0;
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() {
        RobotConfig robotConfig = RobotConfig.createInstance("Rig1Motor");
        hw = HardwareConfig.createInstance(hardwareMap, robotConfig);
        state = RobotState.getInstance();
        pid = new PID();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry telemetry = dashboard.getTelemetry();

        if (Objects.equals(motorType, "extendo")){
            // motorEnum = MotorEnum.EXTENDO;
        }
        else if (Objects.equals(motorType, "extendoPitch")){
            // motorEnum = MotorEnum.EXTENDO_PITCH;
        }
        else if (Objects.equals(motorType, "bucketSlides")){
            // motorEnum = MotorEnum.BUCKET_SLIDES;
        }
        else if (Objects.equals(motorType, "hang")){
            // motorEnum = MotorEnum.HANG;
        }
        else{
            throw new IllegalArgumentException("motorType input wrong");
        }

        waitForStart();

        while (opModeIsActive()){

            if (gamepad1.a){
                state.setMotorTarget(motorEnum, 0);
                timer.reset();
            }
            else if (gamepad1.b){
                state.setMotorTarget(motorEnum, hw.getMotorConfig(motorEnum).maxTarget);
                timer.reset();
            }

            currentPosition = hw.getMotorConfig(motorEnum).motor.getCurrentPosition();
            currentSpeed = (currentPosition - lastPosition) / timer.seconds();

            if (currentSpeed > greatestSpeed){
                greatestSpeed = currentSpeed;
            }

           //  hw.getMotorConfig(motorEnum).motor.setPower(pid.getPIDOutput(motorEnum, state.getMotorTarget(motorEnum), hw.getMotorConfig(motorEnum).kP,hw.getMotorConfig(motorEnum).kI,hw.getMotorConfig(motorEnum).kD));
            telemetry.addData("greatest speed", greatestSpeed);
            telemetry.addData("position", currentPosition);
            telemetry.update();

            lastPosition = currentPosition;
        }
    }
}
