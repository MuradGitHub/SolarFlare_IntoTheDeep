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

import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.min;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.Locale;

public class MotorPowerStrategyConst extends MotorPowerStrategy {
    public double    nominalPower;
    public double    signPower;
    public double    power;

    public double    brakeInc   = 0.05;

    public        MotorPowerStrategyConst(double power_in, int Pi, int Pf) {
        nominalPower    = power_in;
        setDirection(Pi, Pf);
    }

    public String getId() {
        return String.format(Locale.US, "Power=%1$.2f", getSignedNominalPower());
    }

    public double getSignedNominalPower() {
        return signPower * abs(nominalPower);
    }

    public void   setDirection(int Pi, int Pf) {
        super.setDirection(Pi, Pf);
        signPower       = direction == Direction.FORWARD ? 1.0 : -1.0;
        power           = signPower * abs(nominalPower);
    }

    public double applyPower(DcMotorEx motor, int target) {
        if(stopped) {
            motor.setPower(0.0);
            return 0.0;
        } else {
            int P = motor.getCurrentPosition();

            if (direction == Direction.FORWARD ? P >= target : P <= target)
                isTargetReached = true;

            if(isTargetReached) {
                // start reducing power
                power = signPower * max(abs(power) - brakeInc, 0.0);
                motor.setPower(power);
                if (power == 0.0)
                    stopped = true;
                return power;
            } else {
                motor.setPower(power);
                return power;
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
        sb.append("  power=")       .append(power)       .append("\n");
        sb.append("  brakeInc=")    .append(brakeInc)    .append("\n");

        return super.toString() + sb;
    }
}
