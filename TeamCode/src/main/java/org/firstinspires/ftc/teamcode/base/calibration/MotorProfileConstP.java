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

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.base.config.MotorConfig;
import org.firstinspires.ftc.teamcode.base.motorcontrol.MotorPowerStrategyConst;

public class MotorProfileConstP extends MotorProfile {
    public            double                           power;
    public            double                           signedPower;

    /**
     * Constructor requires information about the motor
     * @param motorConfig: The configuration of the motor being calibrated
     */
    public             MotorProfileConstP(MotorConfig motorConfig) {
        super(motorConfig);
    }

    public void        preCalcProfile(int Pi_in, int Pf_in) {
        super.preCalcProfile(Pi_in, Pf_in);
    }

    public void        calcProfile(double power_in, int Pi_in, int Ptarget_in) {
        // This will set Pi and Pf based on Ptarget_in and other parameters
        preCalcProfile(Pi_in, Ptarget_in);

        MotorPowerStrategyConst startPS   = new MotorPowerStrategyConst(1.0, Pi, Pf);
        MotorPowerStrategyConst profilePS = new MotorPowerStrategyConst(power_in, Pi, Pf);

        profilePS.setBreakInc(1.0);

        super.calcProfile(startPS, profilePS, Pi_in, Ptarget_in);
    }

    @NonNull
    @Override
    public String      toString() {
        var sb = new StringBuilder();
        sb.append(super.toString());
        sb.append("MotorProfileConstP\n");
        sb.append("  power=")            .append(power)                      .append("\n");
        sb.append("  signedPower=")      .append(signedPower)                .append("\n");

        return sb.toString();
    }

    public boolean     isValid() {
        return super.isValid();
    }

    public static void main(String[] args) {
    }
}
