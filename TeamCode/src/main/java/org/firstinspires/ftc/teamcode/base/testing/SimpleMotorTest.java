package org.firstinspires.ftc.teamcode.base.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.base.config.HardwareConfig;
import org.firstinspires.ftc.teamcode.base.config.MotorConfig;
import org.firstinspires.ftc.teamcode.base.config.MotorEnum;
import org.firstinspires.ftc.teamcode.base.config.RobotConfig;

@TeleOp
public class SimpleMotorTest extends LinearOpMode {
    public void runOpMode() {
        RobotConfig robotConfig = null;
        HardwareConfig hw = null;
        robotConfig = RobotConfig.makeInstance("Rig1Motor");
        hw = HardwareConfig.makeInstance(hardwareMap, robotConfig);
        MotorConfig motorConfig = hw.getMotorConfig(MotorEnum.TESTING_MOTOR);
        DcMotorEx      motor    = motorConfig.motor;
        DcMotorEx motor_direct  = hardwareMap.get(DcMotorEx.class, "motor-1");

        telemetry.addData("reference equality", motor == motor_direct);
        telemetry.addData("value equality", motor.equals(motor_direct));
        telemetry.addData("SimpleMotorTest", "Done with initialization");
        telemetry.update();

        waitForStart();

        double motorPower = 0;
        while (opModeIsActive()) {
            if (gamepad1.a){
                motorPower = 0;
                telemetry.addData("setPower=", motorPower);
                motor.setPower(motorPower);
                motor_direct.setPower(motorPower);
            }
            else if (gamepad1.b) {
                motorPower = 0.33;
                telemetry.addData("setPower=", motorPower);
                motor.setPower(motorPower);
                //motor_direct.setPower(motorPower);
            }
            else if (gamepad1.y) {
                motorPower = 0.66;
                telemetry.addData("setPower=", motorPower);
                motor.setPower(motorPower);
                //motor_direct.setPower(motorPower);
            }
            else if (gamepad1.x){
                motorPower = 1.0;
                telemetry.addData("setPower=", motorPower);
                motor.setPower(motorPower);
                //motor_direct.setPower(motorPower);
            } else {
                motorPower = 0;
                telemetry.addData("SimpleMotorTest", "Nothing is pressed. Zero Power");
                motor.setPower(motorPower);
                //motor_direct.setPower(motorPower);
            }
            telemetry.update();
        }
    }
}
