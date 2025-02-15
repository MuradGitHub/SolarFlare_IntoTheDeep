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
package org.firstinspires.ftc.teamcode.base.config;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.base.calibration.MotorCalibConfig;
import org.firstinspires.ftc.teamcode.base.calibration.MotorCalibResult;
import org.firstinspires.ftc.teamcode.base.logging.RobotLogger;

import java.util.logging.Level;
import java.util.logging.Logger;

public class MotorConfig implements Validatable {
    public MotorEnum                 motorEnum;
    public MotorSpec                 motorSpec;
    public MotorCalibConfig          calibParams;
    public MotorCalibResult          calibResult;
    public String                    partName;
    public String                    deviceName;
    public DcMotorEx                 motor;
    public double                    kP;
    public double                    kI;
    public double                    kD;
    public DcMotor.RunMode           runMode;
    public DcMotorSimple.Direction   direction;
    public DcMotor.ZeroPowerBehavior zeroPowerBehavior;
    public double                    maxPower;
    public int                       minTarget;
    public int                       maxTarget;
    public double                    maxAcceleration;
    public double                    maxVelocity;

    public void initialize(HardwareMap hardwareMap) {
        try {
            if(motorSpec == null)
                throw new MissingDataException("No motorSpec for motor:" + motorEnum);

            motor = hardwareMap.get(DcMotorEx.class, deviceName);

            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(this.runMode);
            motor.setDirection(this.direction);
            motor.setZeroPowerBehavior(this.zeroPowerBehavior);
        } catch (Exception e) {
            Logger logger = RobotLogger.getInstance().getConfigLogger();
            logger.throwing("MotorConfig", "Initialize", e);
            logger.logp(Level.SEVERE, "MotorConfig", "initialize", toString());
        }
    }

    public double getEncoderResolution() {
        return motorSpec.encoderResolution;
    }

    public double getNoLoadVelocity() {
        return motorSpec.getNoLoadSpeedPPS();
    }

    public void setMotorSpec(MotorSpec motorSpec) {
        this.motorSpec = motorSpec;
    }

    public boolean isValid() {
        return true;
    }

    @NonNull
    @Override
    public String toString() {
        var sb = new StringBuilder();

        sb.append("  motorEnum=")             .append(motorEnum)             .append("\n");
        sb.append("  partName=")              .append(partName)              .append("\n");
        sb.append("  motorSpec=\n")           .append(motorSpec)             .append("\n");
        sb.append("  calibParams=\n")         .append(calibParams)           .append("\n");
        sb.append("  calibResult\n")          .append(calibResult)           .append("\n");
        sb.append("  deviceName=")            .append(deviceName)            .append("\n");
        sb.append("  motor=")                 .append(motor)                 .append("\n");
        sb.append("  kP=")                    .append(kP)                    .append("\n");
        sb.append("  kI=")                    .append(kI)                    .append("\n");
        sb.append("  kD=")                    .append(kD)                    .append("\n");
        sb.append("  runMode=")               .append(runMode)               .append("\n");
        sb.append("  direction=")             .append(direction)             .append("\n");
        sb.append("  zeroPowerBehavior=")     .append(zeroPowerBehavior)     .append("\n");
        sb.append("  encoderResolution=")     .append(getEncoderResolution()).append("\n");
        sb.append("  maxPower=")              .append(maxPower)              .append("\n");
        sb.append("  minTarget=")             .append(minTarget)             .append("\n");
        sb.append("  maxTarget=")             .append(maxTarget)             .append("\n");
        sb.append("  maxAcceleration=")       .append(maxAcceleration)       .append("\n");
        sb.append("  maxVelocity=")           .append(maxVelocity)           .append("\n");
        sb.append("  noLoadVelocity=")        .append(getNoLoadVelocity())   .append("\n");

        return sb.toString();
    }
}
