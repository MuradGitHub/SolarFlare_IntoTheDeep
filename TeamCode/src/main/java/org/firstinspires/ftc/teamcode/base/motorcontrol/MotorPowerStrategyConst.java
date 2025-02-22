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

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

import static org.firstinspires.ftc.teamcode.base.math.Math.approxEquals;

import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.min;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.base.config.MotorConfig;

import java.util.Locale;
import java.util.logging.Level;

public class MotorPowerStrategyConst extends MotorPowerStrategy {
    public double    brakeInc   = 0.05;

    public        MotorPowerStrategyConst(MotorConfig motorConfig_in,
                                          double      nominalPower_in) {
        super(motorConfig_in, nominalPower_in);
        /*
        logger.logp(
                Level.INFO,
                "MotorPowerStrategyConst",
                "()",
                String.format(Locale.US,"nominalPower=%1$.3f", nominalPower));
        */
    }
    public String getDescriptiveId() {
        return String.format(Locale.US, "Power=%1$.2f", getSignedNominalPower());
    }
    @Override
    public void   setDirection(int Pi, int Pf) {
        super.setDirection(Pi, Pf);
        curPower                = getSignedNominalPower();
    }
    @Override
    public void   init(ElapsedTime timer, int Pi, int Ptarget) {
        super.init(timer, Pi, Ptarget);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    @Override
    public void   applyPower(int target) {
        if(isStopped) {
            // return exactly 0.0 power
            curPower = 0.0;
            motor.setPower(curPower);
        } else {
            int P               = motor.getCurrentPosition();

            // isTargetReached is not revised once true
            if (direction == Direction.FORWARD ? P >= target : P <= target)
                isTargetReached = true;

            if(isTargetReached) {
                // start reducing power
                curPower        = signPower * max(abs(curPower) - abs(brakeInc), 0.0);
                motor.setPower(curPower);
                if (approxEquals(curPower, 0.0)) {
                    isStopped   = true;
                }
            } else {
                motor.setPower(curPower);
            }
        }
    }
    public void   setBreakInc(double brakeInc_in) {
        brakeInc = max(min(brakeInc_in, 1.0), 0.0);
    }
    @NonNull
    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("MotorPowerStrategyConst\n");
        sb.append("  nominalPower=").append(nominalPower).append("\n");
        sb.append("  signPower=")   .append(signPower)   .append("\n");
        sb.append("  curPower=")    .append(curPower)    .append("\n");
        sb.append("  brakeInc=")    .append(brakeInc)    .append("\n");

        return super.toString() + sb;
    }
}
