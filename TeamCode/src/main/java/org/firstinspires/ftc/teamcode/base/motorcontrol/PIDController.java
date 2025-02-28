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
import static java.lang.Math.max;
import static java.lang.Math.min;
import static java.lang.Math.signum;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.base.calibration.MotorProfileDataPoint;
import org.firstinspires.ftc.teamcode.base.math.Math;
import org.firstinspires.ftc.teamcode.base.math.Range;

import java.util.Arrays;

public class PIDController extends FBController {
    public transient ElapsedTime timer;
    public           double      Kp;
    public           double      Ki;
    public           double      Kd;
    public           int         prevError;
    public           double      prevTime;
    public           double      timeToUltimateTarget;
    public           double      dError;
    public           boolean     useDErrorAvg = false;
    public           Range       powerRange;
    public           double[]    ErrHistory;
    public           double[]    DerHistory;
    public           int         maxErrorI;
    public           int         ErrIdx       = 0;
    public           int         DerIdx       = 0;
    public           double      powerP       = 0.0;
    public           double      powerI       = 0.0;
    public           double      powerD       = 0.0;

    public        PIDController(double Kp_in,
                                double Ki_in,
                                double Kd_in,
                                int    maxErrorI_in,
                                int    ErrLookback,
                                int    DerLookback,
                                double timeToBrake_in,
                                int    posTol_in,
                                double velTol_in) {
        super(FBControllerEnum.PID, timeToBrake_in, posTol_in, velTol_in);
        Kp                  = Kp_in;
        Ki                  = Ki_in;
        Kd                  = Kd_in;
        maxErrorI           = maxErrorI_in;
        ErrHistory          = new double[ErrLookback];
        DerHistory          = new double[DerLookback];
        powerRange          = new Range(-1,1);

        reset();
    }
    @Override
    public void   reset() {
        super.reset();

        Arrays.fill(ErrHistory, 0.0);
        Arrays.fill(DerHistory, 0.0);
        useDErrorAvg           = false;

        timeToUltimateTarget   = 0.0;
        dError                 = 0.0;
        prevError              = 0;
        prevTime               = 0;
        ErrIdx                 = 0;
        DerIdx                 = 0;
        powerP                 = 0;
        powerI                 = 0;
        powerD                 = 0;
    }
    public void   init(ElapsedTime timer_in) {
        reset();
        timer          = timer_in;
        prevTime       = timer.milliseconds();
    }
    public double getPower(int    curPosition,
                           int    immediateTarget,
                           int    ultimateTarget,
                           double velocity) {
        int    error           = immediateTarget - curPosition;
        int    ultimateError   = ultimateTarget  - curPosition;
        double time            = timer.milliseconds();

        if(state == FBControllerStateEnum.STARTING) {
            state              = FBControllerStateEnum.CRUISING;
            powerP             = powerRange.constrain(Kp * error);
            powerD             = 0.0;
            powerI             = 0.0;
            dError             = 0.0;
            prevError          = error;
            prevTime           = time;
            return powerP;
        }

        dError                 = (error - prevError) / (time - prevTime);

        ErrHistory[ErrIdx]     = abs(error) < abs(maxErrorI) ? error : signum(error) * abs(maxErrorI);
        DerHistory[DerIdx]     = dError;

        // Idx to be used in the following cycle
        ErrIdx                 = (ErrIdx + 1) % ErrHistory.length;
        DerIdx                 = (DerIdx + 1) % DerHistory.length;

        // Save time and error values for nest cycle differences
        prevTime               = time;
        prevError              = error;

        // This means we fully loaded the DerHistory array with historical dError measurements
        // Change to true only once and don't revise
        if(!useDErrorAvg && DerIdx == 0)
            useDErrorAvg       = true;

        // Once useDErrorAvg is true, always true
        if(useDErrorAvg)
            dError             = Math.sum(DerHistory) / DerHistory.length;

        timeToUltimateTarget   = ultimateError == 0 || dError == 0 ? 0.0 : ultimateError / dError;
        if(startParking || abs(timeToUltimateTarget) < timeToBrake) {
            if(timeToUltimateTarget <= 0) {
                state          = FBControllerStateEnum.BRAKING;
                powerD         = powerRange.constrain(Kd * dError);
            } else {
                state          = FBControllerStateEnum.REVERSING;
                powerD         = powerRange.constrain(-Kd * dError);
            }
            powerI             = powerRange.constrain(Ki * Math.sum(ErrHistory));
        } else {
            state              = FBControllerStateEnum.CRUISING;
            powerD             = 0.0;
            powerI             = 0.0;
        }

        powerP                 = powerRange.constrain(Kp * error);

        if(abs(error) <= posTol && abs(dError) <= velTol && abs(velocity) <= velTol)
            state              = FBControllerStateEnum.STOPPED;

        return powerP + powerI + powerD;
    }
    public void   updateMotorProfileDataPoint(MotorProfileDataPoint p) {
        p.fbControllerState    = state.name();
        p.Kp                   = Kp;
        p.Ki                   = Ki;
        p.Kd                   = Kd;
        p.timeToUltimateTarget = timeToUltimateTarget;
        p.dError               = dError;
        p.powerP               = powerP;
        p.powerI               = powerI;
        p.powerD               = powerD;
    }
    @NonNull
    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("PIDController\n");
        sb.append("  time=")                       .append(timer.milliseconds())       .append("\n");
        sb.append("  Kp=")                         .append(Kp)                         .append("\n");
        sb.append("  Ki=")                         .append(Ki)                         .append("\n");
        sb.append("  Kd=")                         .append(Kd)                         .append("\n");
        sb.append("  prevError=")                  .append(prevError)                  .append("\n");
        sb.append("  prevTime=")                   .append(prevTime)                   .append("\n");
        sb.append("  timeToUltimateTarget=")       .append(timeToUltimateTarget)       .append("\n");
        sb.append("  derErr=")                     .append(dError)                     .append("\n");
        sb.append("  ErrHistory=")                 .append(Arrays.toString(ErrHistory)).append("\n");
        sb.append("  DerHistory=")                 .append(Arrays.toString(DerHistory)).append("\n");
        sb.append("  maxErrorI=")                  .append(maxErrorI)                  .append("\n");
        sb.append("  EIdx=")                       .append(ErrIdx)                     .append("\n");
        sb.append("  DIdx=")                       .append(DerIdx)                     .append("\n");
        sb.append("  powerP=")                     .append(powerP)                     .append("\n");
        sb.append("  powerI=")                     .append(powerI)                     .append("\n");
        sb.append("  powerD=")                     .append(powerD)                     .append("\n");

        return sb.toString();
    }
}
