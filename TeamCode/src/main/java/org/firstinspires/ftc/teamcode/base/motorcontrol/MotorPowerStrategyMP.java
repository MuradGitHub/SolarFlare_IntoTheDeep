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
package org.firstinspires.ftc.teamcode.base.motorcontrol;

import static java.lang.Math.abs;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.base.calibration.MotorCalibResult;
import org.firstinspires.ftc.teamcode.base.calibration.MotorProfileDataPoint;
import org.firstinspires.ftc.teamcode.base.config.MotorConfig;

import java.util.Locale;
import java.util.logging.Level;

public class MotorPowerStrategyMP extends MotorPowerStrategy {
    public MotionProfile      motionProfile;
    public FeedbackController fbc;


    public        MotorPowerStrategyMP(MotorConfig            motorConfig,
                                       MotionProfileEnum      motionProfileEnum,
                                       FeedbackControllerEnum FBCEnum,
                                       double                 nominalPower_in) {
        super(motorConfig, nominalPower_in);
        motionProfile = MotionProfiles.makeMotionProfile(motionProfileEnum);
        fbc           = FeedbackControllers.makeFeedbackController(FBCEnum, motorConfig.controlParams);
    }

    /**
     * Set up the MotionProfile to get us from Pi to Pf
     * @param Pi: Initial position
     * @param Ptarget: Final position
     */
    @Override
    public void   init(ElapsedTime timer, int Pi, int Ptarget) {
        super.init(timer, Pi, Ptarget);

        motor.setMode(RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        MotorCalibResult calib = motorConfig.calibResult;

        double           Vi    = motor.getVelocity() / 1000;
        double           Vmax;
        double           Amax;

        if(direction == DcMotorSimple.Direction.FORWARD) {
            Vmax               = calib.VssRangeF.getAbsMax();
            Amax               = calib.AssRangeF.getAbsMax();
        } else {
            Vmax               = calib.VssRangeR.getAbsMax();
            Amax               = calib.AssRangeR.getAbsMax();
        }

        // calibResult is returning Vmax with the wrong units and 0 for the given parameters
        // so overriding for now
        //Vmax        = 2.8;
        //Amax        = 0.06;

        logger.logp(
                Level.INFO,
                "MotorPowerStrategyMP",
                "init",
                String.format(Locale.US,"Vi=%1$.3f Vmax=%2$.3f Amax=%3$.3f", Vi, Vmax, Amax));

        motionProfile.calcProfile(Ptarget-Pi, Pi, Vi, Vmax, Amax, Amax);

        fbc.init(timer);
    }

    // Data
    public void   updateProfileDataPoint(MotorProfileDataPoint p) {
        super.updateProfileDataPoint(p);

        fbc.updateProfileDataPoint(p);

        p.tEndMP    = motionProfile.getEndTime();
    }

    @Override
    public String getDescriptiveId() {
        return String.format(
                Locale.US,
                "MotionProfileMP-MP=%1$s,FBC=%2$s",
                motionProfile.motionProfileEnum.name(),
                fbc.FBCEnum.name());
    }
    @Override
    public void   applyPower(int target_in) {
        int    curPosition     = motor.getCurrentPosition();
        double curVelocity     = motor.getVelocity() / 1000.0;
        double curTime         = timer.milliseconds();
        double fbcPower        = 0;

        if(target_in!=ultimateTarget)
            init(timer, curPosition, target_in);

        if(abs(curPosition - ultimateTarget) < posTol)
            isTargetReached    = true;

        if(!isStopped && isTargetReached && abs(curVelocity) <= velTol)
            isStopped          = true;

        if(isStopped) {
            // return exactly 0.0 power
            curPower           = 0.0;
        } else {
            immediateTarget    = motionProfile.getPosition(curTime);
            fbcPower           = fbc.getPower(immediateTarget - curPosition);
            curPower           = limitPower(fbcPower);
        }

        logger.logp(Level.INFO,
                "MotorPowerStrategyMP",
                "applyPower",
                String.format(Locale.US,
                        "t=%1$.3f P=%2$d uTarget=%3$d iTarget=%4$d fbcPower=%5$.3f power=%6$.3f isStopped=%7$b%nfbc=%8$s",
                        curTime, curPosition, ultimateTarget, immediateTarget, fbcPower, curPower, isStopped, fbc.toString()));

        motor.setPower(curPower);
    }

    @NonNull
    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("MotorPowerStrategyMP\n");

        return super.toString() + sb;
    }
}
