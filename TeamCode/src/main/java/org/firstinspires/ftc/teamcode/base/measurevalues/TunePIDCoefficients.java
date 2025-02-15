package org.firstinspires.ftc.teamcode.base.measurevalues;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.base.config.HardwareConfig;
import org.firstinspires.ftc.teamcode.base.config.MotorEnum;
import org.firstinspires.ftc.teamcode.base.config.RobotConfig;
import org.firstinspires.ftc.teamcode.base.config.RobotState;
import org.firstinspires.ftc.teamcode.base.motorcontrol.PID;

@Config
@TeleOp
public class TunePIDCoefficients extends LinearOpMode {
    HardwareConfig hw;
    RobotState state;
    public static double kP;
    public static double kI;
    public static double kD;
    MotorEnum testingMotor = MotorEnum.TESTING_MOTOR;

    @Override
    public void runOpMode(){
        RobotConfig robotConfig = RobotConfig.makeInstance("Rig1Motor");
        hw = HardwareConfig.makeInstance(hardwareMap, robotConfig);
        state = RobotState.getInstance();

        hw.getMotorConfig(MotorEnum.TESTING_MOTOR).motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hw.getMotorConfig(MotorEnum.TESTING_MOTOR).motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PID pid = new PID();

        Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();


        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.a){
                state.setMotorTarget(testingMotor, (int) (hw.getMotorConfig(testingMotor).maxTarget * 0.25));
            }
            else if (gamepad1.b){
                state.setMotorTarget(testingMotor, (int) (hw.getMotorConfig(testingMotor).maxTarget * 0.5));
            }
            else if (gamepad1.y){
                state.setMotorTarget(testingMotor, (int) (hw.getMotorConfig(testingMotor).maxTarget * 0.75));
            }
            else if (gamepad1.x){
                state.setMotorTarget(testingMotor, hw.getMotorConfig(testingMotor).maxTarget);
            }

            // hw.getMotorConfig(testingMotor).motor.setPower(pid.getPIDOutput(testingMotor, state.getMotorTarget(testingMotor), kP, kI, kD));

            dashboardTelemetry.addData("target",state.getMotorTarget(testingMotor));
            dashboardTelemetry.addData("current pos",hw.getMotorConfig(testingMotor).motor.getCurrentPosition());
            dashboardTelemetry.update();
        }
    }
}
