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

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.base.calibration.MotorProfileDataPoint;
import org.firstinspires.ftc.teamcode.base.config.MotorConfig;
import org.firstinspires.ftc.teamcode.base.logging.DescriptiveIdProvider;
import org.firstinspires.ftc.teamcode.base.logging.RobotLogger;

import java.util.Arrays;
import java.util.Locale;
import java.util.logging.Level;
import java.util.logging.Logger;

public abstract class MotorPowerStrategy implements DescriptiveIdProvider {
    public transient Logger      logger = RobotLogger.getInstance().getConfigLogger();
    public transient MotorConfig motorConfig;
    public transient DcMotorEx   motor;
    public transient ElapsedTime timer;
    public           int         Pi;
    /**
     * Final target
     */
    public           int         ultimateTarget;
    /**
     * Possibly intermediate target
     */
    public           int         immediateTarget;
    public           Direction   direction;
    public           double      nominalPower;
    public           double      signPower;
    public           double      curPower;
    public           int         posTol;
    public           double      velTol;
    public           boolean     isTargetReached   = false;
    public           boolean     isStopped         = false;

    public                  MotorPowerStrategy(MotorConfig motorConfig_in,
                                               double      nominalPower_in) {
        motorConfig     = motorConfig_in;
        motor           = motorConfig.motor;
        posTol          = motorConfig.controlParams.tolerances.posTol;
        velTol          = motorConfig.controlParams.tolerances.velTol;
        nominalPower    = nominalPower_in;

        /*
        logger.logp(
                Level.INFO,
                "MotorPowerStrategy",
                "()",
                String.format(
                        Locale.US,
                        "posTol=%1$d velTol=%2$.3f nominalPower=%3$.3f",
                        posTol, velTol, nominalPower));
        */
    }
    public          void    init(ElapsedTime timer_in, int Pi_in, int Ptarget_in) {
        timer           = timer_in;
        Pi              = Pi_in;
        ultimateTarget  = Ptarget_in;
        immediateTarget = Ptarget_in;
        setDirection(Pi, ultimateTarget);
    }
    public          double  limitPower(double power_in) {
        double absPower = min(abs(power_in),abs(motorConfig.maxPower));
        return signum(power_in) * absPower;
    }
    public          double  getNominalPower() {
        return nominalPower;
    }
    public          double  getSignedNominalPower() {
        double signedNominalPower = signPower * abs(nominalPower);

        /*
        logger.logp(
                Level.INFO,
                "MotorPowerStrategy",
                "getSignedNominalPower",
                String.format(
                        Locale.US,
                        "signedNominalPower=%1$.3f%nStackTrace%2$s",
                        signedNominalPower,
                        Arrays.toString(Thread.currentThread().getStackTrace())));
         */

        return signedNominalPower;
    }
    public          double  getCurPower() {
        return curPower;
    }
    public          boolean isTargetReached() {
        return isTargetReached;
    }
    public          boolean isStopped() {
        return isStopped;
    }
    public          void    setDirection(int Pi, int Pf) {
        direction           = Pi < Pf ? Direction.FORWARD : Direction.REVERSE;
        signPower           = direction == Direction.FORWARD ? 1.0 : -1.0;
    }
    public abstract void    applyPower(int target);
    public          void    updateProfileDataPoint(MotorProfileDataPoint p) {
        p.isTargetReached   = isTargetReached;
        p.isStrategyStopped = isStopped;
        p.ultimateTarget    = ultimateTarget;
        p.immediateTarget   = immediateTarget;
        p.posTol            = posTol;
        p.velTol            = velTol;
    }
    @Override
    @NonNull
    public          String  toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("MotorPowerStrategy\n");
        sb.append("  direction=")      .append(direction)      .append("\n");
        sb.append("  isTargetReached=").append(isTargetReached).append("\n");
        sb.append("  isStopped=")      .append(isStopped)        .append("\n");

        return sb.toString();
    }
}
