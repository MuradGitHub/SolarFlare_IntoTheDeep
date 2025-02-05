package org.firstinspires.ftc.teamcode.base.motorcontrol;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    public double error;
    public int encoderPosition;
    public double derivative;
    public double proportionalPower;
    public double integralPower;
    public double derivativePower;
    public double outPower;
    public double lastError = 0;
    public double lastReference = 0;
    public double integralSum = 0;
    public double loopTime;
    ElapsedTime timer = new ElapsedTime();
    boolean isFirstIteration = true;
    public double getPIDOutput(DcMotor motor, double reference, double kP, double kI, double kD) {
        if (isFirstIteration){
            timer.reset();
            isFirstIteration = false;
        }

        encoderPosition = motor.getCurrentPosition();

        error = reference - encoderPosition;

        derivative = (error - lastError) / timer.seconds();

        integralSum = integralSum + (error * timer.seconds());

        if (reference != lastReference){
            integralSum = 0;
        }

        proportionalPower = error * kP;

        integralPower = integralSum * kI;

        derivativePower = derivative * kD;

        outPower = proportionalPower + integralPower + derivativePower;

        lastReference = reference;
        lastError = error;
        loopTime = timer.seconds();
        timer.reset();
        return outPower;
    }
}
