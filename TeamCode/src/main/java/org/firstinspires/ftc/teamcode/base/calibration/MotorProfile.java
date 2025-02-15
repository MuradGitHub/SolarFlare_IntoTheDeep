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
import static java.lang.Thread.sleep;

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
import org.firstinspires.ftc.teamcode.base.logging.MultiMetricsWriter;
import org.firstinspires.ftc.teamcode.base.logging.RobotLogger;
import org.firstinspires.ftc.teamcode.base.logging.RobotMetrics;
import org.firstinspires.ftc.teamcode.base.math.Math;
import org.firstinspires.ftc.teamcode.base.motorcontrol.MotorPowerStrategy;
import org.firstinspires.ftc.teamcode.base.utils.JSONUtils;
import org.firstinspires.ftc.teamcode.base.validate.Validation;

public class MotorProfile extends MultiMetricsWriter implements JSONWritable, Validatable {
    protected transient final Logger                   logger;
    protected           final MotorEnum                motorEnum;
    protected transient final MotorConfig              motorConfig;
    protected transient final DcMotorEx                motor;
    protected           final String                   metricsSpecStartId   = "MotorProfileData-Start";
    protected           final String                   metricsSpecProfileId = "MotorProfileData-Profile";
    /**
     * Calibration Direction: FORWARD, REVERSE
     */
    public    transient       Direction                calibDirection;

    /**
     * Encoder resolution of the motor itself at the shaft output (PPR)
     */
    public            double                           minTimeInc;
    public            double                           encoderResolution;
    public            int                              timeResolution;
    public            double                           maxProfileTime;
    /**
     * Power strategy
     */
    public            MotorPowerStrategy               powerStrategy;
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
     * Goto Start Data
     */
    public            ArrayList<MotorProfileDataPoint> startData;
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
    public            boolean                          isTargetReached = false;
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
    public MotorProfile(MotorConfig motorConfig_in) {
        motorConfig                   = motorConfig_in;
        logger                        = RobotLogger.getInstance().getConfigLogger();
        motor                         = motorConfig.motor;
        motorEnum                     = motorConfig.motorEnum;
        encoderResolution             = motorConfig.getEncoderResolution();
        minTimeInc                    = motorConfig.calibParams.minTimeInc;
        timeResolution                = motorConfig.calibParams.timeResolution;
        maxProfileTime                = motorConfig.calibParams.maxProfileTime;
        startData                     = new ArrayList<>(timeResolution);
        data                          = new ArrayList<>(timeResolution);
    }

    /**
     * Create necessary MetricsSpecs to write out profile metrics
     * Has to be called at the end of calcProfile as the calibDirection is only available
     * after the profile is calculated
     */
    public void initMetricsSpecs() {
        String fileId = String.format(
                Locale.US,
                "%1$s-%2$s-%3$s-%4$s",
                motorEnum.name(),
                calibDirection.name(),
                "Start",
                powerStrategy.getId());
        addMetricsSpec(metricsSpecStartId, MotorProfileDataPoint.makeMetricsSpec(fileId));
        fileId        = String.format(
                Locale.US,
                "%1$s-%2$s-%3$s-%4$s",
                motorEnum.name(),
                calibDirection.name(),
                "Profile",
                powerStrategy.getId());
        addMetricsSpec(metricsSpecProfileId, MotorProfileDataPoint.makeMetricsSpec(fileId));
    }

    protected void gotoStart() {
        String msg;
        String format;
        /*
        format                   = "Entring: %1$s direction=%2$s P=%3$df Pi=%4$d";
        msg                      = String.format(Locale.US,format,
                motorEnum.name(), calibDirection.name(), motor.getCurrentPosition(), Pi);
        logger.logp(Level.INFO, "MotorProfileConsP", "gotoStart", msg);
         */

        double      toStartPower = motor.getCurrentPosition()<Pi?1.0:-1.0;
        ElapsedTime timer        = new ElapsedTime();
        MotorProfileDataPoint pp;

        motor.setMode(RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(Pi);
        motor.setPower(toStartPower);

        boolean     offTarget    = true;
        while(motor.isBusy() ||
                offTarget    ||
                abs(motor.getVelocity()/1000.0) > motorConfig.calibParams.velocityTolerance) {

            try {
                sleep((int) (minTimeInc));
                pp = new MotorProfileDataPoint(motor, "Start", calibDirection, timer);
                startData.add(pp);
            } catch(InterruptedException e) {
                throw new RuntimeException(e);
            }

            offTarget            = abs(Pi - pp.P) > 5;

            /*
            format               = "%1$s power=%2$.3f P=%3$d C=%4$.3f V=%5$.3f - off target";
            msg                  = String.format(Locale.US,format,
                    calibDirection, pp.power, pp.P, pp.C, pp.V);
            logger.logp(Level.INFO,"MotorProfileConstP","gotoStart-TheWhileLoop",msg);
            */
        }

        motor.setPower(0.0);

        format                   = "Exiting: %1$s %2$s appliedPower=%3$.3f motorPower=%4$.3f P=%5$d V=%6$.3f C=%7$.3f";
        msg                      = String.format(
                Locale.US,
                format,
                motorEnum.name(),
                calibDirection.name(),
                toStartPower,
                motor.getPower(),
                motor.getCurrentPosition(),
                motor.getVelocity() / 1000.0,
                motor.getCurrent(CurrentUnit.AMPS));
        logger.logp(Level.INFO,"MotorProfileConsP", "gotoStart", msg);
    }

    protected void calcDerivedData() {
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

    protected void checkCalcInput() {
        if((calibDirection==Direction.FORWARD && (Pf-Pi)<motorConfig.calibParams.minDistance) ||
                (calibDirection==Direction.REVERSE && (Pi-Pf)<motorConfig.calibParams.minDistance)) {
            String errorMsg = "Distance too short for MotorProfile calibration: calibDirection=" +
                    calibDirection + " Pi=" + Pi + " Pf=" + Pf;
            throw new CalculationException(errorMsg);
        }
    }

    protected void preCalcProfile(int Pi_in, int Ptarget_in) {
        Pi                = Pi_in;
        Ptarget           = Ptarget_in;
        calibDirection    = Ptarget > Pi? Direction.FORWARD : Direction.REVERSE;
        // pull the final position Pf back by targetBuffer
        Pf                = calibDirection == Direction.FORWARD ?
                Ptarget - motorConfig.calibParams.targetBuffer :
                Ptarget + motorConfig.calibParams.targetBuffer;

        checkCalcInput();
    }

    public void calcProfile(MotorPowerStrategy powerStrategy_in, int Pi_in, int Ptarget_in) {
        preCalcProfile(Pi_in, Ptarget_in);
        powerStrategy               = powerStrategy_in;

        ElapsedTime  timer          = new ElapsedTime();
        int          tIdx           = 0;

        gotoStart();

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(RunMode.RUN_WITHOUT_ENCODER);

        int     endSamples          = motorConfig.calibParams.endSamples;
        double  motorPower          = 0.0;

        // Start the path
        // timer keeps time for the entire path. It will not be reset
        timer.reset();
        do {
            try {
                if(!isTargetReached) {
                    motorPower      = powerStrategy.calcPower(motor, Pf);
                    motor.setPower(motorPower);
                    if(motorPower == 0.0)
                        isTargetReached = true;
                }

                sleep((int) (minTimeInc));

                MotorProfileDataPoint pp = new MotorProfileDataPoint(
                        motor,
                        "Profile",
                        calibDirection,
                        timer);
                data.add(pp);

                // if the target has been reached, likely exceeded, then set power to zero and
                // start counting backwards the number of required endSamples
                if (isTargetReached) {
                    endSamples--;
                    if (tIdxTarget == null)
                        tIdxTarget = tIdx++ - 1;
                }

            } catch(InterruptedException e) {
                throw new RuntimeException(e);
            }
        } while((endSamples>=0 || !isTargetReached) && timer.milliseconds() < maxProfileTime);

        /// you get here either because you reached the target AND observed for endSamples
        /// after that. Or, because you simply ran out of space. I.e. you can not perform
        /// any more recordings
        motor.setPower(0);

        data.trimToSize();
        startData.trimToSize();

        calcDerivedData();
        initMetricsSpecs();
    }

    public MotorProfileDataPoint getLastDataPoint() {
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
        return hasReachedTarget()? getTargetDataPoint().t : null;
    }

    public MotorProfileDataPoint getTargetDataPoint() {
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

    public MotorProfileDataPoint getSteadyStateADataPoint() {
        if(!hasSteadyStateA())
            return null;
        return data.get(ssIdxAavg);
    }

    public MotorProfileDataPoint getSteadyStateVDataPoint() {
        if(!hasSteadyStateV())
            return null;
        return data.get(ssIdxVavg);
    }

    /**
     * Obtains the profile data up to the point in time where steady state velocity is achieved
     *
     * @return MotorProfileDataPoints up to the point in time were steady state velocity
     *  is achieved
     */
    public ArrayList<MotorProfileDataPoint> getProfileData() {
        ArrayList<MotorProfileDataPoint> profileData = new ArrayList<>();
        if(hasSteadyStateV())
            for(int tIdx=0; tIdx<=ssIdxVavg; tIdx++)
                profileData.add(data.get(tIdx));
        return profileData;
    }

    public String getJSONFileId() {
        return String.format(
                Locale.US,
                "%1$s-%2$s-%3$s",
                motorEnum.name(),
                calibDirection.name(),
                powerStrategy.getId());
    }

    public void writeJSON() {
        JSONUtils.writeJSON(this);
    }

    public void writeMetrics() {
        MetricsFile metricsStartFile   = RobotMetrics.getInstance()
                .getMetricsFile(getMetricsSpec(metricsSpecStartId));
        for(var p: startData)
            metricsStartFile.addData(p);
        metricsStartFile.close();

        MetricsFile metricsProfileFile = RobotMetrics.getInstance()
                .getMetricsFile(getMetricsSpec(metricsSpecProfileId));
        for(var p: data)
            metricsProfileFile.addData(p);
        metricsProfileFile.close();
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
        sb.append("  Pi=")               .append(Pi)                         .append("\n");
        sb.append("  Pf=")               .append(Pf)                         .append("\n");
        sb.append("  Vmax=")             .append(Vmax)                       .append("\n");
        sb.append("  Amax=")             .append(Amax)                       .append("\n");
        sb.append("  Dmax=")             .append(Dmax)                       .append("\n");
        sb.append("  isTargetReached=")  .append(isTargetReached)            .append("\n");
        sb.append("  ssIdxVavg=")        .append(ssIdxVavg)                  .append("\n");
        sb.append("  ssV=")              .append(getSteadyStateVDataPoint()) .append("\n");
        sb.append("  ssIdxAavg=")        .append(ssIdxAavg)                  .append("\n");
        sb.append("  ssA=")              .append(getSteadyStateADataPoint()) .append("\n");
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
