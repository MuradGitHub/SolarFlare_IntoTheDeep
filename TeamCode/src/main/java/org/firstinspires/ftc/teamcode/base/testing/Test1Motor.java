package org.firstinspires.ftc.teamcode.base.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.base.config.HardwareConfig;
import org.firstinspires.ftc.teamcode.base.config.MotorEnum;
import org.firstinspires.ftc.teamcode.base.config.RobotConfig;
import org.firstinspires.ftc.teamcode.base.config.RobotState;
import org.firstinspires.ftc.teamcode.base.motorcontrol.MotorControl;

@TeleOp
public class Test1Motor extends LinearOpMode {
    HardwareConfig hw;
    RobotState state;
    MotorControl motorControl;

    public void runOpMode(){
        RobotConfig robotConfig = RobotConfig.makeInstance("Rig1Motor");
        HardwareConfig.makeInstance(hardwareMap, robotConfig);

        hw = HardwareConfig.getInstance();
        state = RobotState.getInstance();
        motorControl = new MotorControl(MotorEnum.TESTING_MOTOR);


        hw.getMotorConfig(MotorEnum.TESTING_MOTOR).motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hw.getMotorConfig(MotorEnum.TESTING_MOTOR).motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.a){
                state.setMotorTarget(MotorEnum.TESTING_MOTOR, 2500);
            }
            else if (gamepad1.b){
                state.setMotorTarget(MotorEnum.TESTING_MOTOR, 5000);
            }
            else if (gamepad1.y){
                state.setMotorTarget(MotorEnum.TESTING_MOTOR, 7500);
            }
            else if (gamepad1.x){
                state.setMotorTarget(MotorEnum.TESTING_MOTOR, 10000);
            }

            motorControl.runTrapezoidalMotionProfile(telemetry, FtcDashboard.getInstance().getTelemetry());
        }
    }
}
