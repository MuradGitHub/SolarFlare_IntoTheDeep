/*
 * Copyright (c) 2025 Murad Nayal
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * (subject to the limitations in the disclaimer below) provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions
 * and the following disclaimer in the documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name Murad Nayal nor the names of contributors to this material may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE. THIS
 * SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode.base.calibration;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode;

import static java.lang.Math.abs;
import static java.lang.Math.max;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Locale;
import java.util.logging.Level;
import java.util.logging.Logger;

import androidx.annotation.NonNull;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.base.config.JSONWritable;
import org.firstinspires.ftc.teamcode.base.config.MotorConfig;
import org.firstinspires.ftc.teamcode.base.config.MotorEnum;
import org.firstinspires.ftc.teamcode.base.config.Validatable;
import org.firstinspires.ftc.teamcode.base.error.CalculationException;
import org.firstinspires.ftc.teamcode.base.logging.MetricsFile;
import org.firstinspires.ftc.teamcode.base.logging.MetricsWritable;
import org.firstinspires.ftc.teamcode.base.logging.RobotLogger;
import org.firstinspires.ftc.teamcode.base.logging.RobotMetrics;
import org.firstinspires.ftc.teamcode.base.utils.JSONUtils;
import org.firstinspires.ftc.teamcode.base.validate.Validation;

public class MotorProfileConstP implements MotorProfile, JSONWritable, MetricsWritable, Validatable {
    private transient final Logger      logger;
    private           final MotorEnum   motorEnum;
    private transient final MotorConfig motorConfig;
    private transient final DcMotorEx   motor;
    /**
     * Calibration Direction: FORWARD, REVERSE
     */
    public  transient Direction         calibDirection;

    public            double            minTimeInc;
    /**
     * Encoder resolution of the motor itself at the shaft output (PPR)
     */
    public            double            encoderResolution;
    public            int               timeResolution;
    public            double            power;
    public            double            signedPower;
    /**
     * Starting Position
     */
    public            int               Pi;
    /**
     * Final Position
     */
    public            int               Pf;
    /**
     * Target Position. The motor position should never be driven past this limit
     */
    public            int               Ptarget;
    /**
     * Index where data stops. i.e. if we reach the Pf before we fill out
     * the entire array (before timeResolution)
     */
    public            int               tIdxMax;
    /**
     * The number of periods used to compute Aavg and Vavg
     */
    public            int               averagingPeriods;
    /**
     * Time coordinate
     */
    private           double[]          t;
    /**
     * Time to extract position
     */
    private           double[]          tPextract;
    /**
     * Time to extract velocity
     */
    private           double[]          tVextract;
    /**
     * Time to extract current
     */
    private           double[]          tCextract;
    /**
     * Cycle time
     */
    private           double[]          tCycle;
    /**
     * Position coordinate
     */
    private           int[]             P;
    /**
     * Velocity coordinate
     */
    private           double[]          V;
    /**
     * Moving Average Velocity
     */
    private           double[]          Vavg;
    /**
     * Acceleration coordinate
     */
    private           double[]          A;
    /**
     * Moving Average Acceleration
     */
    private           double[]          Aavg;
    /**
     *
     */
    private           double[]          C;
    /**
     * Power as read from the motor
     */
    private           double[]          motorPower;
    /**
     * Maximum velocity. should be close the stread state velocity
     */
    public            double            Vmax;
    /**
     * Maximum Acceleration
     */
    public            double            Amax;
    /**
     * Maximum Deceleration
     */
    public            double            Dmax;
    /**
     * Has the profile reached the target position Pf
     */
    public            boolean           isTargetReached;
    /**
     * Index of steady state for Vavg
     */
    public            Integer           ssIdxVavg;
    /**
     * steady state Vavg. null if it does not obtain
     */
    public            Double            ssVavg;
    /**
     * Index of steady state of Aavg
     */
    public            Integer           ssIdxAavg;
    /**
     * Steady state Aavg. null if it does not obtain
     */
    public            Double            ssAavg;
    /**
     * Constructor requires information about the motor
     * @param motorConfig_in: The configuration of the motor being calibrated
     */
    public MotorProfileConstP(MotorConfig motorConfig_in) {
        motorConfig       = motorConfig_in;
        logger            = RobotLogger.getInstance().getConfigLogger();
        motor             = motorConfig.motor;
        motorEnum         = motorConfig.motorEnum;
        encoderResolution = motorConfig.getEncoderResolution();
        minTimeInc        = motorConfig.calibParams.minTimeInc;
        timeResolution    = motorConfig.calibParams.timeResolution;
        t                 = new double[timeResolution];
        tPextract         = new double[timeResolution];
        tVextract         = new double[timeResolution];
        tCextract         = new double[timeResolution];
        tCycle            = new double[timeResolution];
        V                 = new double[timeResolution];
        Vavg              = new double[timeResolution];
        P                 = new int   [timeResolution];
        A                 = new double[timeResolution];
        Aavg              = new double[timeResolution];
        C                 = new double[timeResolution];
        motorPower        = new double[timeResolution];
    }

    protected void gotoStart() {
        logger.logp(Level.INFO,
                "MotorProfileConsP",
                "gotoStart",
                "Entering: Motor: " + motorEnum +  " " + calibDirection + " P=" + motor.getCurrentPosition() + " Pi=" + Pi);

        double toStartPower = motor.getCurrentPosition()<Pi?1.0:-1.0;

        motor.setMode(RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(Pi);
        motor.setPower(toStartPower);

        boolean offTarget   = true;
        while(motor.isBusy() ||
                offTarget    ||
                abs(motor.getVelocity()) > motorConfig.calibParams.velocityTolerance) {
            offTarget = abs(Pi - motor.getCurrentPosition()) > 5;

            logger.logp(Level.INFO,
                    "MotorProfileConstP",
                    "TheWhileLoop",
                    calibDirection + " power=" + motor.getPower() + " C=" +
                            motor.getCurrent(CurrentUnit.AMPS) + " P=" +
                            motor.getCurrentPosition() + " V=" + motor.getVelocity() +
                            " - still offTarget");

        }

        motor.setPower(0.0);

        logger.logp(Level.INFO,
                "MotorProfileConsP",
                "gotoStart",
                "Exiting: Motor: " + motorEnum + " " + calibDirection + " appliedPower=" +
                        toStartPower + " motorPower=" + motor.getPower() + " offTarget=" +
                        offTarget + " P=" + motor.getCurrentPosition() + " V=" +
                        motor.getVelocity() + " C=" + motor.getCurrent(CurrentUnit.AMPS));
    }

    public int getPLast() {
        return P[tIdxMax];
    }

    private void calcAveragingPeriods() {
        double tPeriod   = (t[tIdxMax] - t[0])/ (tIdxMax + 1.0);
        averagingPeriods = (int) (motorConfig.calibParams.averagingTime / tPeriod);
    }

    private void calcDerivedData() {
        calcAveragingPeriods();
        Vmax                  = 0.0;
        Amax                  = Double.NEGATIVE_INFINITY;
        Dmax                  = Double.POSITIVE_INFINITY;
        /// tIdx references the original arrays
        for(int tIdx=1; tIdx<=tIdxMax; tIdx++) {
            int tIdx0         = max(tIdx-averagingPeriods, 0);
            double tNow       = t[tIdx];

            Vavg[tIdx]        = (P[tIdx]-P[tIdx0])/(tNow-t[tIdx0]);
            double VNow       = Vavg[tIdx];

            double ANow       = (VNow-Vavg[tIdx-1])/(tNow-t[tIdx-1]);
            Aavg[tIdx]        = (VNow-Vavg[tIdx0])/(tNow-t[tIdx0]);
            A[tIdx]           = ANow;

            if(abs(VNow) > abs(Vmax))
                Vmax          = VNow;

            if(ANow > Amax)
                Amax          = ANow;
            if(ANow < Dmax)
                Dmax          = ANow;
        }

        ssIdxVavg             = Math.getSteadyStateStartPredicate(
                V,
                averagingPeriods,
                (Double v1, Double v2) -> abs(v1-v2) < abs(Vmax)/250.0);
        ssVavg                = ssIdxVavg!=null ? Vavg[ssIdxVavg] : null;

        ssIdxAavg             = Math.getSteadyStateStartPredicate(
                A,
                averagingPeriods,
                (Double a1, Double a2) -> abs(a1-a2) < abs(Amax)/250.0);
        ssAavg                = ssIdxAavg!=null ? Aavg[ssIdxAavg] : null;

        isTargetReached       = Math.approxEquals(P[tIdxMax],Pf,5.0/Pf);
    }

    private void checkCalcInput() {
        if((calibDirection==Direction.FORWARD && (Pf-Pi)<motorConfig.calibParams.minDistance) ||
                (calibDirection==Direction.REVERSE && (Pi-Pf)<motorConfig.calibParams.minDistance)) {
            String errorMsg = "Distance too short for MotorProfile calibration: calibDirection=" +
                    calibDirection + " Pi=" + Pi + " Pf=" + Pf;
            throw new CalculationException(errorMsg);
        }
    }

    public void calcProfile(double power_in, int Pi_in, int Ptarget_in) {
        Pi                      = Pi_in;
        Ptarget                 = Ptarget_in;
        Pf                      = Ptarget - motorConfig.calibParams.targetBuffer;
        power                   = abs(power_in);
        signedPower             = Pf > Pi? power : -power;
        calibDirection          = Pf > Pi? Direction.FORWARD : Direction.REVERSE;

        checkCalcInput();

        ElapsedTime timer       = new ElapsedTime();
        ElapsedTime eTimer      = new ElapsedTime();
        double  dt;
        double  tPrev           = 0;
        int     tIdx            = 0;

        gotoStart();
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(RunMode.RUN_WITHOUT_ENCODER);

        double  tCycleNow       = 0;
        double  tPextractNow    = 0;
        double  tVextractNow    = 0;
        double  tCextractNow    = 0;
        double  tNow;
        int     PNow;
        double  VNow;
        double  CNow;
        double  motorPowerNow;
        boolean targetReached;
        int     endSamples      = motorConfig.calibParams.endSamples;
        timer.reset();
        eTimer.reset();
        motor.setPower(signedPower);
        String format           = "tIdx=%1$d shortOfTarget=%2$b Pi=%3$d Pf=%4$d P=%5$d";
        do {
            tCycleNow          += eTimer.seconds();
            eTimer.reset();
            tNow                = timer.seconds();

            /// Pull current position info
            PNow                 = motor.getCurrentPosition();
            tPextractNow        += eTimer.seconds();

            /// Pull velocity info
            /// With no arguments getVelocity() returns Ticks Per Second
            VNow                 = motor.getVelocity();
            tVextractNow        += eTimer.seconds();

            /// Pull current info
            CNow                 = motor.getCurrent(CurrentUnit.AMPS);
            motorPowerNow        = motor.getPower();
            tCextractNow        += eTimer.seconds();

            dt                   = tNow - tPrev;
            if(dt >= minTimeInc) {
                t         [tIdx] = tNow;
                P         [tIdx] = PNow;
                V         [tIdx] = VNow;
                C         [tIdx] = CNow;
                motorPower[tIdx] = motorPowerNow;
                tCycle    [tIdx] = tCycleNow;
                tPextract [tIdx] = tPextractNow - tCextractNow;
                tVextract [tIdx] = tVextractNow - tPextractNow;
                tCextract [tIdx] = tCextractNow - tVextractNow;
                tPrev            = tNow;
                tCycleNow        = 0;
                tPextractNow     = 0;
                tVextractNow     = 0;
                tCextractNow     = 0;

                tIdx++;
            }
            targetReached        = calibDirection == Direction.FORWARD? PNow>=Pf : PNow<=Pf;

            /*
            logger.logp(Level.SEVERE,
                    "MotorProfileConstP",
                    "calclProfile",
                    String.format(Locale.US, format, tIdx, shortOfTarget, Pi, Pf, PNow));
            */

            if(targetReached) {
                motor.setPower(0.0);
                endSamples--;
            }

        } while((endSamples>=0 || !targetReached) && tIdx<timeResolution);

        motor.setPower(0);
        /// tIdx is one position ahead of the last valid slot in the data arrays
        tIdxMax                  = tIdx - 1;

        calcDerivedData();
        trimArrays();
    }

    private double[] trimArray(double[] data) {
        double[] trimmedArray = new double[tIdxMax+1];
        System.arraycopy(data, 0, trimmedArray, 0, tIdxMax+1);
        return trimmedArray;
    }

    private int[] trimArray(int[] data) {
        int[] trimmedArray = new int[tIdxMax+1];
        System.arraycopy(data, 0, trimmedArray, 0, tIdxMax+1);
        return trimmedArray;
    }

    private void trimArrays() {
        t                 = trimArray(t);
        tPextract         = trimArray(tPextract);
        tVextract         = trimArray(tVextract);
        tCextract         = trimArray(tCextract);
        tCycle            = trimArray(tCycle);
        V                 = trimArray(V);
        Vavg              = trimArray(Vavg);
        P                 = trimArray(P);
        A                 = trimArray(A);
        Aavg              = trimArray(Aavg);
        C                 = trimArray(C);
    }

    public boolean hasSteadyStateV() {
        return ssVavg != null;
    }

    public boolean hasReachedTarget() {
        return isTargetReached;
    }

    /**
     * Returns time to reach steady state in seconds
     * @return time to reach steady state
     */
    public Double getTimeToSteadyState() {
        return hasSteadyStateV() ? t[ssIdxVavg] : null;
    }

    /**
     * Returns time to reach target
     * @return time to reach target
     */
    public Double getTimeToTarget() {
        return hasReachedTarget()? t[tIdxMax] : null;
    }

    public Double getSteadyStateV() {
        return ssVavg;
    }

    public boolean hasSteadyStateA() {
        return ssAavg != null;
    }

    /**
     * Returns time to reach acceleration steady state (should be zero) in seconds
     * @return time to reach steady state
     */
    public Double getTimeToSteadyStateA() {
        return hasSteadyStateA() ? t[ssIdxAavg] : null;
    }

    public Double getSteadyStateA() {
        return ssAavg;
    }

    public String getJSONFileId() {
        return String.format(Locale.US, "%1$s-%2$s-%3$.2f", motorEnum, calibDirection.name(), power);
    }

    public void writeJSON() {
        JSONUtils.writeJSON(this);
    }

    public String getMetricsFileId() {
        return String.format(Locale.US, "%1$s-%2$s-%3$.4f", motorEnum, calibDirection.name(), power);
    }

    public String getMetricsTableType() {
        return "MotorProfileConstP";
    }

    /**
     * The caller needs to close the metrics file
     * @param file: the MetricsFile to write metrics to
     */
    public void writeMetrics(MetricsFile file) {
        for(int tIdx=0; tIdx<=tIdxMax; tIdx++) {
            file.addData(
                    t[tIdx], tPextract[tIdx], tVextract[tIdx], tCextract[tIdx], tCycle[tIdx],
                    P[tIdx], V[tIdx],         Vavg[tIdx],      A[tIdx],         Aavg[tIdx],
                    C[tIdx], motorPower[tIdx]
            );
        }
    }

    public void writeMetrics() {
        MetricsFile metricsFile = RobotMetrics.getInstance().getMetricsFile(this);
        writeMetrics(metricsFile);
        metricsFile.close();
    }

    public MotorProfileDataPoint getDataAtTIdx(int tIdx) {
        if(tIdx<0 || tIdx>=t.length)
            throw new IndexOutOfBoundsException("MotorProfileDataPoint.getDataAtTIdx passed: " + tIdx);

        return new MotorProfileDataPoint(
                calibDirection,
                t[tIdx],
                P[tIdx],
                Vavg[tIdx],
                Aavg[tIdx],
                motorPower[tIdx]);
    }

    public MotorProfileDataPoint getSteadyStateResult() {
        if(!hasSteadyStateV())
            return null;
        return getDataAtTIdx(ssIdxVavg);
    }

    public ArrayList<MotorProfileDataPoint> getProfileData() {
        ArrayList<MotorProfileDataPoint> data = new ArrayList<>();
        if(hasSteadyStateV())
            for(int tIdx=0; tIdx<=ssIdxVavg; tIdx++)
                data.add(getDataAtTIdx(tIdx));
        return data;
    }

    @NonNull
    @Override
    public String toString() {
        var sb = new StringBuilder();

        sb.append("MotorProfileConstP\n");
        sb.append("  JSONFileId=")       .append(getJSONFileId())            .append("\n");
        sb.append("  motorEnum=")        .append(motorEnum)                  .append("\n");
        sb.append("  calibDirection=")   .append(calibDirection.name())      .append("\n");
        sb.append("  minTimeInc=")       .append(minTimeInc)                 .append("\n");
        sb.append("  encoderResolution=").append(encoderResolution)          .append("\n");
        sb.append("  timeResolution=")   .append(timeResolution)             .append("\n");
        sb.append("  averagingPeriods=") .append(averagingPeriods)           .append("\n");
        sb.append("  power=")            .append(power)                      .append("\n");
        sb.append("  signedPower=")      .append(signedPower)                .append("\n");
        sb.append("  Pi=")               .append(Pi)                         .append("\n");
        sb.append("  Pf=")               .append(Pf)                         .append("\n");
        sb.append("  tIdxMax=")          .append(tIdxMax)                    .append("\n");
        sb.append("  t=\n")              .append(Arrays.toString(t))         .append("\n");
        sb.append("  tPextract=\n")      .append(Arrays.toString(tPextract)) .append("\n");
        sb.append("  tVextract=\n")      .append(Arrays.toString(tVextract)) .append("\n");
        sb.append("  tCextract=\n")      .append(Arrays.toString(tCextract)) .append("\n");
        sb.append("  tCycle=\n")         .append(Arrays.toString(tCycle))    .append("\n");
        sb.append("  P=\n")              .append(Arrays.toString(P))         .append("\n");
        sb.append("  V=\n")              .append(Arrays.toString(V))         .append("\n");
        sb.append("  Vavg=\n")           .append(Arrays.toString(Vavg))      .append("\n");
        sb.append("  A=\n")              .append(Arrays.toString(A))         .append("\n");
        sb.append("  Aavg=\n")           .append(Arrays.toString(Aavg))      .append("\n");
        sb.append("  C=\n")              .append(Arrays.toString(C))         .append("\n");
        sb.append("  motorPower=\n")     .append(Arrays.toString(motorPower)).append("\n");
        sb.append("  Vmax=")             .append(Vmax)                       .append("\n");
        sb.append("  Amax=")             .append(Amax)                       .append("\n");
        sb.append("  Dmax=")             .append(Dmax)                       .append("\n");
        sb.append("  isTargetReached=")  .append(isTargetReached)            .append("\n");
        sb.append("  ssIdxVavg=")        .append(ssIdxVavg)                  .append("\n");
        sb.append("  ssVavg=")           .append(ssVavg)                     .append("\n");
        sb.append("  ssIdxAavg=")        .append(ssIdxAavg)                  .append("\n");
        sb.append("  ssAavg=")           .append(ssAavg)                     .append("\n");

        return sb.toString();
    }

    public boolean isValid() {
        return Validation.validate("motorEnum",          motorEnum)                                                              &&
                Validation.validate("motorConfig",       motorConfig)                                                            &&
                Validation.validate("motor",             motor)                                                                  &&
                Validation.validate("minTimeInc",        minTimeInc,        (Double x)  -> x!=null && x>0.0)                     &&
                Validation.validate("encoderResolution", encoderResolution, (Double x)  -> x!=null && x>0)                       &&
                Validation.validate("timeResolution",    timeResolution,    (Integer i) -> i!=null && i>0)                       &&
                Validation.validate("power",             power,             (Double x)  -> x!=null && x>=0  && x<=1)             &&
                Validation.validate("signedPower",       signedPower,       (Double x)  -> x!=null && x>=-1 && x<=1)             &&
                Validation.validate("tIdxMax",           tIdxMax,           (Integer i) -> i!=null && i>=0  && i<timeResolution) &&
                Validation.validate("averagingPeriods",  averagingPeriods,  (Integer i) -> i!=null && i>0   && i<timeResolution) &&
                Validation.validate("t",                 t)                                                                      &&
                Validation.validate("tPextract",         tPextract)                                                              &&
                Validation.validate("tVextract",         tVextract)                                                              &&
                Validation.validate("tCextract",         tCextract)                                                              &&
                Validation.validate("tCycle",            tCycle)                                                                 &&
                Validation.validate("P",                 P)                                                                      &&
                Validation.validate("V",                 V)                                                                      &&
                Validation.validate("Vavg",              Vavg)                                                                   &&
                Validation.validate("A",                 A)                                                                      &&
                Validation.validate("Aavg",              Aavg)                                                                   &&
                Validation.validate("C",                 C)                                                                      &&
                Validation.validate("Vmax",              Vmax)                                                                   &&
                Validation.validate("Amax",              Amax)                                                                   &&
                Validation.validate("Dmax",              Dmax)                                                                   &&
                Validation.validate("ssIdxVavg",         ssIdxVavg,         (Integer i) -> i!=null && i>=0 && i<timeResolution)  &&
                Validation.validate("ssVavg",            ssVavg)                                                                 &&
                Validation.validate("ssIdxAavg",         ssIdxAavg,         (Integer i) -> i!=null && i>=0 && i<timeResolution)  &&
                Validation.validate("ssAavg",            ssAavg);
    }

    public static void main(String[] args) {
    }
}
