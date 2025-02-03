package org.firstinspires.ftc.teamcode.base.testing;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.base.error.CalculationException;

public class DummyDcMotorEx implements DcMotorEx {
    private boolean          motorEnable;
    private double           angularRate;
    private AngleUnit        angleUnit;
    private PIDFCoefficients pidfCoefficients;
    private RunMode          runMode;
    private int              positionTolerance;
    private int              targetPosition;
    private double           power;

    @Override
    public void setMotorEnable() {
        motorEnable = true;
    }

    @Override
    public void setMotorDisable() {
        motorEnable = false;
    }

    @Override
    public boolean isMotorEnabled() {
        return motorEnable;
    }

    @Override
    public void setVelocity(double angularRate) {
        this.angularRate = angularRate;
    }

    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        this.angularRate = angularRate;
        this.angleUnit   = unit;
    }

    @Override
    public double getVelocity() {
        return angularRate;
    }

    @Override
    public double getVelocity(AngleUnit unit) {
        if(angleUnit == unit)
            return angularRate;
        else
            throw new CalculationException("Wrong angular unit");
    }

    @Override
    public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
        this.runMode            = mode;
        this.pidfCoefficients.p = pidCoefficients.p;
        this.pidfCoefficients.d = pidCoefficients.d;
        this.pidfCoefficients.i = pidCoefficients.i;
    }

    @Override
    public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients)
            throws UnsupportedOperationException {
        this.runMode          = mode;
        this.pidfCoefficients = pidfCoefficients;
    }

    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        pidfCoefficients.i = i;
        pidfCoefficients.p = p;
        pidfCoefficients.d = d;
        pidfCoefficients.f = f;
    }

    @Override
    public void setPositionPIDFCoefficients(double p) {
        pidfCoefficients.p = p;
    }

    @Override
    public PIDCoefficients getPIDCoefficients(RunMode mode) {
        PIDCoefficients pidCoefficients = new PIDCoefficients();
        pidCoefficients.p = pidfCoefficients.p;
        pidCoefficients.i = pidfCoefficients.i;
        pidCoefficients.d = pidfCoefficients.d;

        return pidCoefficients;
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        return pidfCoefficients;
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {
        positionTolerance = tolerance;
    }

    @Override
    public int getTargetPositionTolerance() {
        return positionTolerance;
    }

    @Override
    public double getCurrent(CurrentUnit unit) {
        return 0.0;
    }

    @Override
    public double getCurrentAlert(CurrentUnit unit) {
        return 0;
    }

    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {
    }

    @Override
    public boolean isOverCurrent() {
        return false;
    }

    @Override
    public MotorConfigurationType getMotorType() {
        return null;
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {
    }

    @Override
    public DcMotorController getController() {
        return null;
    }

    @Override
    public int getPortNumber() {
        return 0;
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return null;
    }

    @Override
    public void setPowerFloat() {
    }

    @Override
    public boolean getPowerFloat() {
        return false;
    }

    @Override
    public void setTargetPosition(int position) {
        targetPosition = position;
    }

    @Override
    public int getTargetPosition() {
        return targetPosition;
    }

    @Override
    public boolean isBusy() {
        return false;
    }

    @Override
    public int getCurrentPosition() {
        return 0;
    }

    @Override
    public void setMode(RunMode mode) {
        runMode = mode;
    }

    @Override
    public RunMode getMode() {
        return runMode;
    }

    @Override
    public void setDirection(Direction direction) {
    }

    @Override
    public Direction getDirection() {
        return null;
    }

    @Override
    public void setPower(double power) {
        this.power = power;
    }

    @Override
    public double getPower() {
        return power;
    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return "";
    }

    @Override
    public String getConnectionInfo() {
        return "";
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
    }

    @Override
    public void close() {
    }

    public static void main(String[] args) {
        DcMotorEx motor = new DummyDcMotorEx();
        System.out.println(motor);
    }
}
