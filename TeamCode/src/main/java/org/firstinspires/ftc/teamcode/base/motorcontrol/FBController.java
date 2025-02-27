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

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.base.calibration.MotorProfileDataPoint;

public abstract class FBController {
    public FBControllerEnum       FBCEnum;
    public FBControllerStateEnum  state        = FBControllerStateEnum.STARTING;
    public boolean                startParking = false;
    public double                 timeToBrake;
    public int                    posTol;
    public double                 velTol;

    public FBController(
            FBControllerEnum FBCEnum_in,
            double           timeToBrake_in,
            int              posTol_in,
            double           velTol_in) {
        FBCEnum      = FBCEnum_in;
        timeToBrake  = timeToBrake_in;
        startParking = false;
        posTol       = posTol_in;
        velTol       = velTol_in;
    }
    public abstract void    init(ElapsedTime timer);
    public          void    reset() {
        state        = FBControllerStateEnum.STARTING;
        startParking = false;
    }
    public          void    startParking() {
        startParking = true;
    }
    public          boolean isStopped() {
        return state == FBControllerStateEnum.STOPPED;
    }
    public abstract double  getPower(int    curPosition,
                                     int    immediateTarget,
                                     int    ultimateTarget,
                                     double velocity);
    public abstract void    updateMotorProfileDataPoint(MotorProfileDataPoint p);
}
