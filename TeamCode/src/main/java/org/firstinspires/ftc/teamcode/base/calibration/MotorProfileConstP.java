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
    private transient final Logger                     logger;
    private           final MotorEnum                  motorEnum;
    private transient final MotorConfig                motorConfig;
    private transient final DcMotorEx                  motor;
    /**
     * Calibration Direction: FORWARD, REVERSE
     */
    public  transient Direction                        calibDirection;

    /**
     * Encoder resolution of the motor itself at the shaft output (PPR)
     */
    public            double                           minTimeInc;
    public            double                           encoderResolution;
    public            int                              timeResolution;
    public            double                           maxProfileTime;
    public            double                           power;
    public            double                           signedPower;
    /**
     * Starting Position
     */
    public            int                              Pi;
    /**
     * Final Position
     */
    public            int                              Pf;
    /**
     * Target Position. The motor position should never be driven past this limit
     */
    public            int                              Ptarget;
    /**
     * Data
     */
    public            ArrayList<MotorProfileDataPoint> data;
    /**
     * The number of periods used to compute Aavg and Vavg
     */
    public            int                              averagingPeriods;
    /**
     * Maximum velocity. should be close the stread state velocity
     */
    public            double                           Vmax;
    /**
     * Maximum Acceleration
     */
    public            double                           Amax;
    /**
     * Maximum Deceleration
     */
    public            double                           Dmax;
    /**
     * Has the profile reached the target position Pf
     */
    public            boolean                          isTargetReached;
    /**
     * Index when target has been reached
     */
    public            Integer                          tIdxTarget = null;
    /**
     * Index of steady state for Vavg
     */
    public            Integer                          ssIdxVavg  = null;
    /**
     * Index of steady state of Aavg
     */
    public            Integer                          ssIdxAavg  = null;
    /**
     * Constructor requires information about the motor
     * @param motorConfig_in: The configuration of the motor being calibrated
     */
    public MotorProfileConstP(MotorConfig motorConfig_in) {
        motorConfig                   = motorConfig_in;
        logger                        = RobotLogger.getInstance().getConfigLogger();
        motor                         = motorConfig.motor;
        motorEnum                     = motorConfig.motorEnum;
        encoderResolution             = motorConfig.getEncoderResolution();
        minTimeInc                    = motorConfig.calibParams.minTimeInc;
        timeResolution                = motorConfig.calibParams.timeResolution;
        maxProfileTime                = motorConfig.calibParams.maxProfileTime;
        data                          = new ArrayList<MotorProfileDataPoint>(timeResolution);
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

    private void calcDerivedData() {
        // calculate number of averaging periods
        double tPeriod                = (data.get(data.size()-1).t-data.get(0).t)/data.size();
        averagingPeriods              = (int) (motorConfig.calibParams.averagingTime/tPeriod);
        Vmax                          = 0.0;
        Amax                          = Double.NEGATIVE_INFINITY;
        Dmax                          = Double.POSITIVE_INFINITY;
        /// tIdx references the original arrays
        for(int tIdx=1; tIdx<data.size(); tIdx++) {
            // tIdx0 is the index of the start of the averaging span
            int tIdx0                 = max(tIdx-averagingPeriods, 0);
            MotorProfileDataPoint p   = data.get(tIdx);
            MotorProfileDataPoint p_1 = data.get(tIdx-1);
            MotorProfileDataPoint p0  = data.get(tIdx0);
            double tNow               = p.t;
            double tSpan              = tNow - p0.t;

            double VNow               = (p.P-p0.P)/tSpan;
            p.Vavg                    = VNow;

            double ANow               = (VNow-p_1.Vavg)/(tNow-p_1.t);
            p.Aavg                    = (VNow-p0.Vavg)/tSpan;
            p.A                       = ANow;

            if(abs(VNow) > abs(Vmax))
                Vmax                  = VNow;

            if(ANow > Amax)
                Amax                  = ANow;
            if(ANow < Dmax)
                Dmax                  = ANow;
        }

        double Vtol                   = abs(Vmax)/250.0;
        ssIdxVavg                     = Math.getSteadyStateStartPredicate(
                data,
                averagingPeriods,
                (MotorProfileDataPoint p1, MotorProfileDataPoint p2) -> abs(p1.Vavg-p2.Vavg)<Vtol);

        double Atol                   = max(abs(Amax),abs(Dmax))/250.0;
        ssIdxAavg                     = Math.getSteadyStateStartPredicate(
                data,
                averagingPeriods,
                (MotorProfileDataPoint p1, MotorProfileDataPoint p2) -> abs(p1.Aavg-p2.Aavg)<Atol);
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
        Pi                            = Pi_in;
        Ptarget                       = Ptarget_in;
        power                         = abs(power_in);
        calibDirection                = Ptarget > Pi? Direction.FORWARD : Direction.REVERSE;
        // pull the final position Pf back by targetBuffer
        Pf                            = calibDirection == Direction.FORWARD ?
                Ptarget - motorConfig.calibParams.targetBuffer :
                Ptarget + motorConfig.calibParams.targetBuffer;
        signedPower                   = Pf > Pi? power : -power;

        checkCalcInput();

        ElapsedTime           timer   = new ElapsedTime();
        ElapsedTime           eTimer  = new ElapsedTime();
        MotorProfileDataPoint point;
        double  dt;
        double  tPrev                 = Double.NEGATIVE_INFINITY;
        int     tIdx                  = 0;

        gotoStart();
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(RunMode.RUN_WITHOUT_ENCODER);

        double  tCycleNow             = 0;
        double  tPextractNow          = 0;
        double  tVextractNow          = 0;
        double  tCextractNow          = 0;
        double  tNow;
        int     PNow;
        double  VNow;
        double  CNow;
        double  motorPowerNow;
        int     endSamples            = motorConfig.calibParams.endSamples;
        String  format                = "tIdx=%1$d shortOfTarget=%2$b Pi=%3$d Pf=%4$d P=%5$d";

        // Start the path
        // timer keeps time for the entire path. It will not be reset
        timer.reset();
        // eTimer times each iteration to figure out how long each step takes. It will be reset
        // at the start of each iteration
        eTimer.reset();
        motor.setPower(signedPower);
        do {
            tCycleNow                += eTimer.seconds();
            eTimer.reset();
            tNow                      = timer.seconds();

            /// Because of the time it takes to pull data from the motor, there measurements
            /// unfortunately are not exactly synchronous

            /// Pull current position info
            PNow                      = motor.getCurrentPosition();
            tPextractNow             += eTimer.seconds();

            /// Pull velocity info
            /// With no arguments getVelocity() returns Ticks Per Second
            VNow                      = motor.getVelocity();
            tVextractNow             += eTimer.seconds();

            /// Pull current info
            CNow                      = motor.getCurrent(CurrentUnit.AMPS);
            motorPowerNow             = motor.getPower();
            tCextractNow             += eTimer.seconds();

            dt                        = tNow - tPrev;
            if(dt >= minTimeInc) {
                data.add(new MotorProfileDataPoint(
                        calibDirection,
                        tNow,
                        tPextractNow - tCextractNow,
                        tVextractNow - tPextractNow,
                        tCextractNow - tVextractNow,
                        tCycleNow,
                        PNow,
                        VNow,
                        motorPowerNow,
                        CNow));
                tPrev                 = tNow;
                tCycleNow             = 0;
                tPextractNow          = 0;
                tVextractNow          = 0;
                tCextractNow          = 0;

                tIdx++;
            }

            isTargetReached           = calibDirection == Direction.FORWARD? PNow>=Pf : PNow<=Pf;

            if(isTargetReached && tIdxTarget == null)
                tIdxTarget            = tIdx-1;

            /*
            logger.logp(Level.SEVERE,
                    "MotorProfileConstP",
                    "calclProfile",
                    String.format(Locale.US, format, tIdx, shortOfTarget, Pi, Pf, PNow));
            */

            if(timer.seconds() >= maxProfileTime)
                break;

            /// if the target has been reached, likely exceeded, then set power to zero and
            /// start counting backwards the number of required endSamples
            if(isTargetReached) {
                motor.setPower(0.0);
                endSamples--;
            }

        } while(endSamples>=0 || !isTargetReached);

        /// you get here either because you reached the target AND observed for endSamples
        /// after that. Or, because you simply ran out of space. I.e. you can not perform
        /// any more recordings
        motor.setPower(0);

        data.trimToSize();
        calcDerivedData();
    }

    public MotorProfileDataPoint getLastData() {
        if(data.isEmpty())
            return null;
        return data.get(data.size()-1);
    }

    public boolean hasReachedTarget() {
        return isTargetReached;
    }

    /**
     * Returns time to reach target
     * @return time to reach target
     */
    public Double getTimeToTarget() {
        return hasReachedTarget()? data.get(tIdxTarget).t : null;
    }

    public MotorProfileDataPoint getTargetData() {
        if(!hasReachedTarget())
            return null;
        return data.get(tIdxTarget);
    }

    public boolean hasSteadyStateV() {
        return ssIdxVavg != null;
    }

    public boolean hasSteadyStateA() {
        return ssIdxAavg != null;
    }

    public MotorProfileDataPoint getSteadyStateAData() {
        if(!hasSteadyStateA())
            return null;
        return data.get(ssIdxAavg);
    }

    public MotorProfileDataPoint getSteadyStateVData() {
        if(!hasSteadyStateV())
            return null;
        return data.get(ssIdxVavg);
    }

    public ArrayList<MotorProfileDataPoint> getProfileData() {
        ArrayList<MotorProfileDataPoint> profileData = new ArrayList<>();
        if(hasSteadyStateV())
            for(int tIdx=0; tIdx<=ssIdxVavg; tIdx++)
                profileData.add(data.get(tIdx));
        return profileData;
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
        for(var point: data)
            file.addData(point);
    }

    public void writeMetrics() {
        MetricsFile metricsFile = RobotMetrics.getInstance().getMetricsFile(this);
        writeMetrics(metricsFile);
        metricsFile.close();
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
        sb.append("  maxProfileTime=")   .append(maxProfileTime)             .append("\n");
        sb.append("  averagingPeriods=") .append(averagingPeriods)           .append("\n");
        sb.append("  power=")            .append(power)                      .append("\n");
        sb.append("  signedPower=")      .append(signedPower)                .append("\n");
        sb.append("  Pi=")               .append(Pi)                         .append("\n");
        sb.append("  Pf=")               .append(Pf)                         .append("\n");
        sb.append("  Vmax=")             .append(Vmax)                       .append("\n");
        sb.append("  Amax=")             .append(Amax)                       .append("\n");
        sb.append("  Dmax=")             .append(Dmax)                       .append("\n");
        sb.append("  isTargetReached=")  .append(isTargetReached)            .append("\n");
        sb.append("  ssIdxVavg=")        .append(ssIdxVavg)                  .append("\n");
        sb.append("  ssV=")              .append(getSteadyStateVData())      .append("\n");
        sb.append("  ssIdxAavg=")        .append(ssIdxAavg)                  .append("\n");
        sb.append("  ssA=")              .append(getSteadyStateAData())      .append("\n");
        for(var point: data)
            sb.append(point);

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
                Validation.validate("averagingPeriods",  averagingPeriods,  (Integer i) -> i!=null && i>0   && i<timeResolution) &&
                Validation.validate("data",              data)                                                                   &&
                Validation.validate("Vmax",              Vmax)                                                                   &&
                Validation.validate("Amax",              Amax)                                                                   &&
                Validation.validate("Dmax",              Dmax)                                                                   &&
                Validation.validate("ssIdxVavg",         ssIdxVavg,         (Integer i) -> i!=null && i>=0 && i<timeResolution)  &&
                Validation.validate("ssIdxAavg",         ssIdxAavg,         (Integer i) -> i!=null && i>=0 && i<timeResolution);
    }

    public static void main(String[] args) {
    }
}
