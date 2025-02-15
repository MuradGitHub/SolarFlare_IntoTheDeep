package org.firstinspires.ftc.teamcode.base.measurevalues;

import static java.lang.Math.abs;
import static java.lang.Math.min;
import static java.lang.Math.max;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.base.math.LookupTable1D;
import org.firstinspires.ftc.teamcode.base.config.HardwareConfig;
import org.firstinspires.ftc.teamcode.base.config.MotorConfig;
import org.firstinspires.ftc.teamcode.base.config.MotorEnum;
import org.firstinspires.ftc.teamcode.base.config.RobotConfig;
import org.firstinspires.ftc.teamcode.base.config.RobotState;

import java.util.Locale;

@Config
@TeleOp
public class TunePIDCoefficients1D extends LinearOpMode {
    static final MotorEnum motorEnum = MotorEnum.TESTING_MOTOR;
    static final int       errorTol  = 5;
    RobotConfig            robotConfig;
    HardwareConfig         hardwareConfig;
    RobotState             state;
    MotorConfig            motorConfig;
    DcMotorEx              motor;

    public static double kP          = 0.0335; // original
    public static double kI          = 0.0;
    public static double kD          = 0.0; // original
    public static double errorCap    = 50.0;
    public static double a_x1        = 5.0;
    public static double a_y1        = 0.25;
    public static double a_x2        = 20.0;
    public static double a_y2        = 0.3;
    public static double a_x3        = 50.0;
    public static double a_y3        = 0.5;
    public static double a_x_max     = 60.0;
    // 0.0335

    @Override
    public void runOpMode(){
        robotConfig              = RobotConfig.makeInstance("Rig1Motor");
        hardwareConfig           = HardwareConfig.makeInstance(hardwareMap, robotConfig);
        motorConfig              = hardwareConfig.getMotorConfig(motorEnum);
        motor                    = motorConfig.motor;
        state                    = RobotState.getInstance();

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FtcDashboard dashboard       = FtcDashboard.getInstance();
        Telemetry    telemetry       = dashboard.getTelemetry();
        ElapsedTime  timer           = new ElapsedTime();


        double lastPosition          = 0;
        double lastTarget            = 0;
        double integralSum           = 0;

        waitForStart();

        boolean convFlag             = false;
        double convTime              = Double.POSITIVE_INFINITY;
        ElapsedTime convTimer        = new ElapsedTime();
        convTimer.reset();

        LookupTable1D atten = new LookupTable1D();
        atten.addDataPoint(a_x1, a_y1)
                .addDataPoint(a_x2, a_y2)
                .addDataPoint(a_x3, a_y3)
                .addDataPoint(a_x_max,1.0);

        int iteration                = 0;
        while (opModeIsActive()){
            if (gamepad1.a){
                state.setMotorTarget(motorEnum, (int) (motorConfig.maxTarget * 0.25));
            }
            else if (gamepad1.b){
                state.setMotorTarget(motorEnum, (int) (motorConfig.maxTarget * 0.5));
            }
            else if (gamepad1.y){
                state.setMotorTarget(motorEnum, (int) (motorConfig.maxTarget * 0.75));
            }
            else if (gamepad1.x){
                state.setMotorTarget(motorEnum, motorConfig.maxTarget);
            }

            double currentTarget     = state.getMotorTarget(motorEnum);
            double currentPosition   = motor.getCurrentPosition();
            double dt                = timer.milliseconds();
            double error             = currentTarget - currentPosition;
            double derivative        = (currentPosition - lastPosition) / dt;

            // We get better stability by asking the motor to provide speed
            derivative               = motor.getVelocity() / 1000.0;

            if (currentTarget != lastTarget) {
                convFlag             = false;
                integralSum          = 0;
                convTimer.reset();
            }
            integralSum              = integralSum + (error * dt);

            if(abs(error) < errorTol && !convFlag) {
                convFlag             = true;
                convTime             = convTimer.milliseconds();
            }

            double proportionalPower = max(min(error * kP,1),-1);
            double integralPower     = max(min(integralSum * kI,1),-1);
            double derivativePower   = max(min(derivative * kD,1),-1);
            double outPower          = max(min(proportionalPower+integralPower+derivativePower,1),-1);
            double attenFactor       = atten.apply(abs(error));

            outPower                *= attenFactor;

            motor.setPower(outPower);

            lastTarget               = currentTarget;
            lastPosition             = currentPosition;

            timer.reset();

            double printedError      = max(min(error,errorCap),-errorCap);

            telemetry.addData("Iteration", iteration++);
            telemetry.addData("K- Kp", String.format(Locale.US,"%1$15.8f", kP));
            telemetry.addData("K- Kd", String.format(Locale.US,"%1$15.8f", kD));
            telemetry.addData("K- Ki", String.format(Locale.US,"%1$15.8f", kI));
            telemetry.addData("P- convergence", convFlag);
            telemetry.addData("P- convergence time", convTime);
            telemetry.addData("P- motor max target", motorConfig.maxTarget);
            telemetry.addData("P- motor target", state.getMotorTarget(motorEnum));
            telemetry.addData("P- motor position", hardwareConfig.getMotorConfig(motorEnum).motor.getCurrentPosition());
            telemetry.addData("P- error", printedError);
            telemetry.addData("R- attenuation factor (x10)", 10.0*attenFactor);
            telemetry.addData("R- proportionalPower", proportionalPower);
            telemetry.addData("R- derivative", derivative);
            telemetry.addData("R- derivativePower", derivativePower);
            telemetry.addData("R- integralSum", integralSum);
            telemetry.addData("R- integralPower", integralPower);
            telemetry.addData("R- motor power", outPower);
            telemetry.update();
        }
    }
}
