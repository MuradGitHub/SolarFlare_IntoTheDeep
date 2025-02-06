package org.firstinspires.ftc.teamcode.base.motorcontrol;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.base.config.HardwareConfig;
import org.firstinspires.ftc.teamcode.base.config.MotorConfig;
import org.firstinspires.ftc.teamcode.base.config.MotorEnum;
import org.firstinspires.ftc.teamcode.base.config.RobotState;

public class MotorControl {
    HardwareConfig hw;
    RobotState state;
    PID pid;
    TrapezoidalMotionProfile profile;
    public ElapsedTime timer;
    int currentPosition;
    int currentTarget;
    double motorPower;
    double maxAcceleration;
    double maxVelocity;
    int previousLoopTarget = 0;
    double lastMaxAcceleration = 0;
    double lastMaxVelocity = 0;
    double instantTargetPosition;
    boolean isMaxVelocityChanged = false;
    boolean isMaxAccelerationChanged = false;
    double initialVelocity;
    int distance;
    MotorEnum motorEnum;

    public double kP;
    public double kI;
    public double kD;

    MotorConfig motorConfig;
    public MotorControl(MotorEnum motorEnum){
        this.motorEnum = motorEnum;
        hw = HardwareConfig.getInstance();
        state = RobotState.getInstance();
        pid = new PID();
        profile = new TrapezoidalMotionProfile();
        timer = new ElapsedTime();
        maxAcceleration = hw.getMotorConfig(motorEnum).maxAcceleration;
        maxVelocity = hw.getMotorConfig(motorEnum).maxVelocity;
        motorConfig = hw.getMotorConfig(motorEnum);

        kP = motorConfig.kP;
        kI = motorConfig.kI;
        kD = motorConfig.kD;
    }
    // Run this method in a loop
    public void runTrapezoidalMotionProfile(Telemetry telemetry, Telemetry dashBoardTelemetry){

        int currentTarget = state.getMotorTarget(motorEnum);

        // If the target, maxVelocity, or maxAcceleration changes
        if (currentTarget != previousLoopTarget || isMaxVelocityChanged || isMaxAccelerationChanged){

            currentPosition = motorConfig.motor.getCurrentPosition();
            distance = currentTarget - currentPosition;
            initialVelocity = motorConfig.motor.getVelocity();

            timer.reset();

            profile.resetProfile(
                    maxAcceleration,
                    maxVelocity,
                    initialVelocity,
                    distance,
                    currentPosition);

            isMaxVelocityChanged = false;
            isMaxAccelerationChanged = false;
        }

        double instantTargetPosition = profile.runProfile(timer.seconds());

        // motorPower = pid.getPIDOutput(motor, instantTargetPosition, kP, kI, kD);

        motorConfig.motor.setPower(motorPower);

        previousLoopTarget = currentTarget;
        lastMaxVelocity = maxVelocity;
        lastMaxAcceleration = maxAcceleration;

        telemetry.addData("current phase", profile.currentPhase);
        telemetry.addData("current time", timer.seconds());
        telemetry.addData("motor power", motorPower);
        telemetry.addData("state target position", state.getMotorTarget(motorEnum));
        telemetry.addData("current position", hw.getMotorConfig(motorEnum).motor.getCurrentPosition());
        telemetry.addData("instant target pos", instantTargetPosition);
        telemetry.addData("distance", distance );
        telemetry.addData("initial speed", initialVelocity);
        telemetry.addData("acceleration time", profile.accelerationTime);
        telemetry.addData("acceleration distance", profile.accelerationDistance);
        telemetry.addData("cruise time", profile.cruiseTime);
        telemetry.addData("cruise distance", profile.cruiseDistance);
        telemetry.addData("deceleration time", profile.decelerationTime);
        telemetry.addData("deceleration distance", profile.decelerationDistance);
        telemetry.addData("total time", profile.totalTime);
        telemetry.addData("max acceleration", profile.maxAcceleration);
        telemetry.addData("max deceleration", profile.maxDeceleration);
        telemetry.addData("max velocity", profile.maxVelocity);
        telemetry.addData("current velocity", hw.getMotorConfig(motorEnum).motor.getVelocity());

        dashBoardTelemetry.addData("current position", hw.getMotorConfig(motorEnum).motor.getCurrentPosition());
        dashBoardTelemetry.addData("target position", state.getMotorTarget(motorEnum));
        dashBoardTelemetry.addData("speed", hw.getMotorConfig(motorEnum).motor.getVelocity());

        dashBoardTelemetry.update();
        telemetry.update();
    }

    public void setMaxAcceleration(double maxAcceleration){
        this.maxAcceleration = maxAcceleration;
        isMaxAccelerationChanged = true;
    }
    public void setMaxVelocity(double maxVelocity){
        this.maxVelocity = maxVelocity;
        isMaxVelocityChanged = true;
    }

    // Run this method in a loop
    public void runPIDMotorControl(Telemetry telemetry){

         // motorPower = pid.getPIDOutput(motorEnum, state.getMotorTarget(motorEnum), kP, kI, kD);
        hw.getMotorConfig(motorEnum).motor.setPower(motorPower);

        telemetry.addData("current position", pid.encoderPosition);
        telemetry.addData("state position", state.getMotorTarget(motorEnum));
        telemetry.addData("error", pid.error);
        telemetry.addData("proportional power", pid.proportionalPower);
        telemetry.addData("integral power", pid.integralPower);
        telemetry.addData("derivative power", pid.derivativePower);
        telemetry.addData("derivative", pid.derivativePower);
        telemetry.addData("out power", pid.outPower);
        telemetry.addData("loop time", pid.loopTime);
        telemetry.update();
    }
}
