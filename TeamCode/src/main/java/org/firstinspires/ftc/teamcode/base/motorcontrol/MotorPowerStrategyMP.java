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
import static java.lang.Math.min;
import static java.lang.Math.signum;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.base.config.MotorConfig;

import java.util.Locale;

public class MotorPowerStrategyMP extends MotorPowerStrategy {
    public MotionProfile      motionProfile;
    public FeedbackController fbc;
    public double             maxPower;

    public        MotorPowerStrategyMP(MotorConfig            motorConfig,
                                       MotionProfileEnum      motionProfileEnum,
                                       FeedbackControllerEnum FBCEnum,
                                       double                 power) {
        super(motorConfig, power);
        motionProfile = MotionProfiles.makeMotionProfile(motionProfileEnum);
        fbc           = FeedbackControllers.makeFeedbackController(FBCEnum, motorConfig.controlParams);
    }

    /**
     * Set up the MotionProfile to get us from Pi to Pf
     * @param Pi: Initial position
     * @param Pf: Final position
     */
    @Override
    public void   init(DcMotorEx motor, ElapsedTime timer, int Pi, int Pf) {
        super.init(motor, timer, Pi, Pf);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double Vi   = motor.getVelocity() / 1000;
        double Vmax = motorConfig.calibResult.getVss(direction, maxPower);
        double Amax = motorConfig.calibResult.getAccelByLut(direction,maxPower, Vi);

        motionProfile.calcProfile(Pf-Pi, Pi, Vi, Vmax, Amax, Amax);
        fbc.init(timer);
    }
    @Override
    public String getId() {
        return String.format(Locale.US, "Power=%1$s", "What is this now");
    }
    @Override
    public void   applyPower(DcMotorEx motor, int target) {
        int currentPosition = motor.getCurrentPosition();

        if(isStopped) {
            // return exactly 0.0 power
            power = 0.0;
        } else {
            target          = motionProfile.getPosition(timer.milliseconds());
            power           = limitPower(fbc.getPower(target - currentPosition));
        }

        if(abs(currentPosition - Pf) < posTol)
            isTargetReached = true;

        motor.setPower(power);
    }
    public double limitPower(double power_in) {
        double absPower = min(abs(power_in),abs(maxPower));
        return signum(power_in) * absPower;
    }

    @NonNull
    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("MotorPowerStrategyMP\n");

        return super.toString() + sb;
    }
}
