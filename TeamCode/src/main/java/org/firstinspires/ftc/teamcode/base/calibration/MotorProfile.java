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

import static java.lang.Math.abs;
import static java.lang.Math.max;

import java.util.ArrayList;
import java.util.Locale;
import java.util.logging.Level;
import java.util.logging.Logger;

import androidx.annotation.NonNull;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.base.config.Application;
import org.firstinspires.ftc.teamcode.base.config.JSONWritable;
import org.firstinspires.ftc.teamcode.base.config.MotorConfig;
import org.firstinspires.ftc.teamcode.base.config.MotorEnum;
import org.firstinspires.ftc.teamcode.base.config.Validatable;
import org.firstinspires.ftc.teamcode.base.error.CalculationException;
import org.firstinspires.ftc.teamcode.base.logging.MetricsFile;
import org.firstinspires.ftc.teamcode.base.logging.MultiMetricsWriter;
import org.firstinspires.ftc.teamcode.base.logging.DescriptiveIdProvider;
import org.firstinspires.ftc.teamcode.base.logging.RobotLogger;
import org.firstinspires.ftc.teamcode.base.logging.RobotMetrics;
import org.firstinspires.ftc.teamcode.base.math.Math;
import org.firstinspires.ftc.teamcode.base.motorcontrol.MotorPowerStrategy;
import org.firstinspires.ftc.teamcode.base.utils.JSONUtils;
import org.firstinspires.ftc.teamcode.base.validate.Validation;

public class MotorProfile
        extends MultiMetricsWriter
        implements JSONWritable, Validatable, DescriptiveIdProvider {
    protected transient final Logger                           logger;
    protected           final MotorEnum                        motorEnum;
    protected transient final MotorConfig                      motorConfig;
    protected transient final DcMotorEx                        motor;
    protected           final String                           metricsSpecStartId   = "MotorProfileData-Start";
    protected           final String                           metricsSpecProfileId = "MotorProfileData-Profile";
    /**
     * Calibration Direction: FORWARD, REVERSE
     */
    public    transient       Direction                        calibDirection;

    /**
     * Encoder resolution of the motor itself at the shaft output (PPR)
     */
    public                    int                              minTimeInc;
    public                    double                           encoderResolution;
    public                    int                              timeResolution;
    public                    double                           maxProfileTime;
    /**
     * Power strategy
     */
    public                    MotorPowerStrategy               profilePowerStrategy;
    public                    MotorPowerStrategy               startPowerStrategy;
    /**
     * Starting Position
     */
    public                    int                              Pi;
    /**
     * Final Position
     */
    public                    int                              Pf;
    /**
     * Target Position. The motor position should never be driven past this limit
     */
    public                    int                              Ptarget;
    /**
     * Data
     */
    public                    ArrayList<MotorProfileDataPoint> data;
    /**
     * Goto Start Data
     */
    public                    ArrayList<MotorProfileDataPoint> startData;
    /**
     * The number of periods used to compute Aavg and Vavg
     */
    public                    int                              averagingPeriods;
    /**
     * Maximum velocity. should be close the steady state velocity
     */
    public                    double                           Vmax;
    /**
     * Maximum Acceleration
     */
    public                    double                           Amax;
    /**
     * Maximum Deceleration
     */
    public                    double                           Dmax;
    /**
     * Has the profile reached the target position Pf
     */
    public                    boolean                          isTargetReached   = false;
    public                    boolean                          isStrategyStopped = false;
    /**
     * Index when target has been reached
     */
    public                    Integer                          tIdxTarget        = null;
    /**
     * Index of steady state for Vavg
     */
    public                    Integer                          ssIdxVavg         = null;
    /**
     * Index of steady state of Aavg
     */
    public                    Integer                          ssIdxAavg         = null;
    /**
     * minMovingPower fromm MotorCalibResult to be stored in the MotorProfileDataPoint(s)
     */
    public                    Double                           minMovePowerF;
    public                    Double                           minMovePowerR;

    // Construction
    /**
     * Constructor requires information about the motor
     * @param motorConfig_in: The configuration of the motor being calibrated
     */
    public                MotorProfile(MotorConfig        motorConfig_in,
                                       MotorPowerStrategy startPowerStrategy_in,
                                       MotorPowerStrategy profilePowerStrategy_in) {
        motorConfig                   = motorConfig_in;
        logger                        = RobotLogger.getInstance().getConfigLogger();
        motor                         = motorConfig.motor;
        motorEnum                     = motorConfig.motorEnum;
        encoderResolution             = motorConfig.getEncoderResolution();
        minTimeInc                    = motorConfig.calibParams.minTimeInc;
        timeResolution                = motorConfig.calibParams.timeResolution;
        maxProfileTime                = motorConfig.calibParams.maxProfileTime;
        minMovePowerF                 = motorConfig.calibResult.minMovePowerF;
        minMovePowerR                 = motorConfig.calibResult.minMovePowerR;
        startPowerStrategy            = startPowerStrategy_in;
        profilePowerStrategy          = profilePowerStrategy_in;
        startData                     = new ArrayList<>(timeResolution);
        data                          = new ArrayList<>(timeResolution);
    }
    /**
     * Create necessary MetricsSpecs to write out profile metrics
     * Has to be called at the end of calcProfile as the calibDirection is only available
     * after the profile is calculated
     */
    public        void    initMetricsSpecs() {
        addMetricsSpec(metricsSpecStartId, MotorProfileDataPoint.makeMetricsSpec(
                String.format(Locale.US, "%1$s-Start", getDescriptiveId())
        ));
        addMetricsSpec(metricsSpecProfileId, MotorProfileDataPoint.makeMetricsSpec(
                String.format(Locale.US, "%1$s-Profile", getDescriptiveId())
        ));
    }

    // Calculations
    protected     void    gotoStart() {
        /*
        Application.telemetry.addData("Entering gotoStart", startPowerStrategy.toString());
        Application.telemetry.update();
         */

        String msg;
        String format;

        /*
        format                   = "Entering: %1$s direction=%2$s P=%3$df Pi=%4$d";
        msg                      = String.format(Locale.US,format,
                motorEnum.name(), calibDirection.name(), motor.getCurrentPosition(), Pi);
        logger.logp(Level.INFO, "MotorProfileConsP", "gotoStart", msg);
         */

        int         endSamples   = motorConfig.calibParams.endSamples;
        ElapsedTime timer        = new ElapsedTime();
        MotorProfileDataPoint pp;

        startPowerStrategy.init(timer, motor.getCurrentPosition(), Pi);

        do {
            startPowerStrategy.applyPower(Pi);

            Application.sleep(minTimeInc);

            pp               = new MotorProfileDataPoint(
                    motor,
                    "Start-Seeking",
                    calibDirection,
                    timer,
                    calibDirection == Direction.FORWARD ? minMovePowerF : minMovePowerR,
                    true);
            startPowerStrategy.updateProfileDataPoint(pp);
            startData.add(pp);

            if(startPowerStrategy.isStopped) {
                pp.setMotorProfileStage("Start-EndSamples");
                endSamples--;
            }

            /*
            format               = "%1$s power=%2$.3f Pi=%3$d P=%4$d C=%5$.3f V=%6$.3f isBusy=%7$b" +
                    " posTol=%8$d isAtTarget=%9$b velTol=%10$.3f isMoving=%11$b seeking=%12$b" +
                    " endSamples=%13$d";
            msg                  = String.format(Locale.US,format,
                    calibDirection, pp.power, Pi, pp.P, pp.C, pp.V, pp.isBusy, posTol,
                    pp.isAtTarget(Pi, posTol), velTol, pp.isMoving(velTol), seeking, endSamples);
            logger.logp(Level.INFO,"MotorProfileConstP","gotoStart-TheWhileLoop",msg);
             */

        } while((endSamples>=0 || !startPowerStrategy.isStopped) && timer.milliseconds() < maxProfileTime);

        motor.setPower(0.0);

        /*
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
        */
    }
    protected     void    calcDerivedData() {
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

        if(ssIdxVavg != null) {
            MotorProfileDataPoint p   = data.get(ssIdxVavg);
            if(p != null)
                p.appendMotorProfileStage("Vss");
        }

        double Atol                   = max(abs(Amax),abs(Dmax))/250.0;
        ssIdxAavg                     = Math.getSteadyStateStartPredicate(
                data,
                averagingPeriods,
                (MotorProfileDataPoint p1, MotorProfileDataPoint p2) -> abs(p1.Aavg-p2.Aavg)<Atol);

        if(ssIdxAavg != null) {
            MotorProfileDataPoint p   = data.get(ssIdxAavg);
            if(p != null)
                p.appendMotorProfileStage("Ass");
        }
    }
    protected     void    checkCalcInput() {
        if((calibDirection==Direction.FORWARD && (Pf-Pi)<motorConfig.calibParams.minDistance) ||
                (calibDirection==Direction.REVERSE && (Pi-Pf)<motorConfig.calibParams.minDistance)) {
            String errorMsg = "Distance too short for MotorProfile calibration: calibDirection=" +
                    calibDirection + " Pi=" + Pi + " Pf=" + Pf;
            throw new CalculationException(errorMsg);
        }
    }
    protected     void    preCalcProfile(int Pi_in, int Pf_in) {
        Pi                = Pi_in;
        Pf                = Pf_in;
        calibDirection    = Pf > Pi? Direction.FORWARD : Direction.REVERSE;
        // pull the final position Pf back by targetBuffer
        Ptarget           = calibDirection == Direction.FORWARD ?
                Pf - motorConfig.calibParams.targetBuffer :
                Pf + motorConfig.calibParams.targetBuffer;

        checkCalcInput();
    }
    public        void    calcProfile(   int Pi_in, int Pf_in) {
        preCalcProfile(Pi_in, Pf_in);

        ElapsedTime  timer           = new ElapsedTime();
        int          tIdx            = 0;
        int          endSamples      = motorConfig.calibParams.endSamples;

        gotoStart();

        // Start the path
        // timer keeps time for the entire path. It will not be reset
        timer.reset();
        profilePowerStrategy.init(timer, Pi, Ptarget);
        do {
            // Apply power and capture data
            profilePowerStrategy.applyPower(Ptarget);
            MotorProfileDataPoint pp = new MotorProfileDataPoint(
                    motor,
                    "Profile-Seeking",
                    calibDirection,
                    timer,
                    calibDirection == Direction.FORWARD ? minMovePowerF : minMovePowerR,
                    true);
            profilePowerStrategy.updateProfileDataPoint(pp);
            data.add(pp);

            // Wait minTimeInc between iterations
            Application.sleep(minTimeInc);

            // Denote the point in time where the target has been reached
            if(tIdxTarget == null && profilePowerStrategy.isTargetReached) {
                tIdxTarget       = tIdx;
                pp.appendMotorProfileStage("Target");
            }

            // Initialize the attribute flag indicating whether the power strategy has stopped
            // if the power strategy has stopped then start counting backwards the number
            // of required endSamples
            isStrategyStopped        = profilePowerStrategy.isStopped;
            if (isStrategyStopped) {
                pp.setMotorProfileStage("Profile-EndSamples");
                endSamples--;
            }

            // Keep a running count of iterations to use in denoting important milestones
            tIdx++;

        // Conditions for repeating the iterations of the motor profile
        } while((endSamples>0 || !isStrategyStopped) && timer.milliseconds() < maxProfileTime);

        /// you get here either because you reached the target AND observed for endSamples
        /// after that. Or, because you simply ran out of space. I.e. you can not perform
        /// any more recordings
        motor.setPower(0);

        data.trimToSize();
        startData.trimToSize();

        calcDerivedData();
        initMetricsSpecs();
    }

    // Data access
    public        double  getNominalPower() {
        return profilePowerStrategy.getNominalPower();
    }
    public        double  getSignedNominalPower() {
        return profilePowerStrategy.getSignedNominalPower();
    }
    public        boolean hasReachedTarget() {
        return isTargetReached;
    }
    /**
     * Returns time to reach target
     * @return time to reach target
     */
    public        Double  getTimeToTarget() {
        return hasReachedTarget()? getTargetDataPoint().t : null;
    }
    public        boolean hasMoved() {
        return abs(getFirstDataPoint().P-getLastDataPoint().P) > 0.1 * abs(Ptarget-Pi);
    }
    public        boolean hasSteadyStateV() {
        return ssIdxVavg != null;
    }
    public        boolean hasSteadyStateA() {
        return ssIdxAavg != null;
    }

    public MotorProfileDataPoint getFirstDataPoint() {
        if(data.isEmpty())
            return null;
        return data.get(0);
    }
    public MotorProfileDataPoint getLastDataPoint() {
        if(data.isEmpty())
            return null;
        return data.get(data.size()-1);
    }
    public MotorProfileDataPoint getTargetDataPoint() {
        if(!hasReachedTarget())
            return null;
        return data.get(tIdxTarget);
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

    // Data persistence
    /**
     * Obtains the profile data if the profile has achieved steady state velocity
     * otherwise return an empty container
     *
     * @return MotorProfileDataPoints up to the point in time were steady state velocity
     *  is achieved
     */
    public ArrayList<MotorProfileDataPoint> getProfileData() {
        ArrayList<MotorProfileDataPoint> profileData = new ArrayList<>();
        if(hasSteadyStateV())
            profileData.addAll(data);
        return profileData;
    }

    public        String  getDescriptiveId() {
        String descriptiveId = String.format(
                Locale.US,
                "%1$s-%2$s-%3$s",
                motorEnum.name(),
                calibDirection.name(),
                profilePowerStrategy.getDescriptiveId());

        /*
        logger.logp(
                Level.INFO,
                "MotorProfile",
                "getDescriptiveId",
                String.format(Locale.US, "descriptiveId=%1$s", descriptiveId)
        );
        */

        return descriptiveId;
    }
    public        String  getJSONFileId() {
        return getDescriptiveId();
    }
    public        void    writeJSON() {
        JSONUtils.writeJSON(this);
    }
    public        void    writeMetrics() {
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
    public        String  toString() {
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
        sb.append("  isStrategyStopped=").append(isStrategyStopped)          .append("\n");
        sb.append("  ssIdxVavg=")        .append(ssIdxVavg)                  .append("\n");
        sb.append("  ssV=")              .append(getSteadyStateVDataPoint()) .append("\n");
        sb.append("  ssIdxAavg=")        .append(ssIdxAavg)                  .append("\n");
        sb.append("  ssA=")              .append(getSteadyStateADataPoint()) .append("\n");
        for(var point: data)
            sb.append(point);

        return sb.toString();
    }
    public        boolean isValid() {
        return Validation.validate("motorEnum",          motorEnum)                                                              &&
                Validation.validate("motorConfig",       motorConfig)                                                            &&
                Validation.validate("motor",             motor)                                                                  &&
                Validation.validate("minTimeInc",        minTimeInc,        (Integer x) -> x!=null && x>0)                       &&
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

    public static void    main(String[] args) {
    }
}
