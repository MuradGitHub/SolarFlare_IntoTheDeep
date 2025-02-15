package org.firstinspires.ftc.teamcode.base.measurevalues;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.base.config.HardwareConfig;
import org.firstinspires.ftc.teamcode.base.config.MotorEnum;
import org.firstinspires.ftc.teamcode.base.config.RobotConfig;
import org.firstinspires.ftc.teamcode.base.config.RobotState;
import org.firstinspires.ftc.teamcode.base.motorcontrol.PID;

import java.util.Objects;

@TeleOp
@Config
public class MeasureMotorAcceleration extends LinearOpMode {
    HardwareConfig hw;
    RobotState state;
    PID pid;
    public static String motorType = "extendo";
    MotorEnum motorEnum;
    double greatestSpeed = 0;
    double currentSpeed = 0;
    double lastSpeed = 0;
    double currentAcceleration = 0;
    double lastAcceleration = 0;
    double greatestAcceleration = 0;
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode(){
        RobotConfig robotConfig = RobotConfig.makeInstance("Rig1Motor");
        hw = HardwareConfig.makeInstance(hardwareMap, robotConfig);
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

        if (gamepad1.a){
            state.setMotorTarget(motorEnum, 0);
            timer.reset();
        }
        else if (gamepad1.b){
            state.setMotorTarget(motorEnum, 3500);
            timer.reset();
        }


        if (currentSpeed > greatestSpeed){
            greatestSpeed = currentSpeed;
        }
        if (currentAcceleration > greatestAcceleration){
            greatestAcceleration = currentAcceleration;
        }

        currentAcceleration = currentSpeed - lastSpeed;

        // hw.getMotorConfig(motorEnum).motor.setPower(pid.getPIDOutput(motorEnum, state.getMotorTarget(motorEnum), hw.getMotorConfig(motorEnum).kP,hw.getMotorConfig(motorEnum).kI,hw.getMotorConfig(motorEnum).kD));
        DcMotorEx dcMotorEx = (DcMotorEx) (hw.getMotorConfig(motorEnum).motor);
        currentSpeed = dcMotorEx.getVelocity();

        telemetry.addData("greatest acceleration", greatestAcceleration);
        telemetry.addData("greatest speed", greatestSpeed);
        telemetry.addData("speed", currentSpeed);
        telemetry.update();

        lastSpeed = currentSpeed;
        lastAcceleration = currentAcceleration;

    }
}
