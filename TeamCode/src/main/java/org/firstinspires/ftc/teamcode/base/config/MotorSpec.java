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

import org.firstinspires.ftc.teamcode.base.math.Range;

public class MotorSpec implements Validatable {
    public double nominalVoltage;
    public String gearRatio;
    public String gearRatioFormula;
    /**
     * No-Load Speed at nominal voltage
     */
    public double noLoadSpeed;
    /**
     * No-Load Current (Ampere) at nominal voltage
     */
    public double noLoadCurrent;
    /**
     * Stall Current (Ampere) at nominal voltage
     */
    public double stallCurrent;
    /**
     * Stall Torque (kg.cm) at nominal voltage
     */
    public double stallTorque;
    /**
     * Encoder Type
     */
    public String encoderType;
    /**
     * Encoder Sensor Type
     */
    public String encoderSensorType;
    /**
     * Encoder Voltage Range
     */
    public Range encoderVoltageRange;
    /**
     * Encoder Resolution (PPR)
     */
    public double encoderResolution;
    /**
     * Encoder Resolution Formula
     */
    public String encoderResolutionFormula;

    public double getNoLoadSpeedPPS() {
        return encoderResolution * noLoadSpeed;
    }

    public boolean isValid() {
        return true;
    }

    @NonNull
    @Override
    public String toString() {
        var sb = new StringBuilder();
        sb.append("MotorSpec\n");
        sb.append("  nominalVoltage=")          .append(nominalVoltage)          .append("\n");
        sb.append("  gearRatio=")               .append(gearRatio)               .append("\n");
        sb.append("  gearRatioFormula=")        .append(gearRatioFormula)        .append("\n");
        sb.append("  noLoadSpeed=")             .append(noLoadSpeed)             .append("\n");
        sb.append("  noLoadCurrent=")           .append(noLoadCurrent)           .append("\n");
        sb.append("  stallCurrent=")            .append(stallCurrent)            .append("\n");
        sb.append("  stallTorque=")             .append(stallTorque)             .append("\n");
        sb.append("  encoderType=")             .append(encoderType)             .append("\n");
        sb.append("  encoderSensorType=")       .append(encoderSensorType)       .append("\n");
        sb.append("  encoderVoltageRange=")     .append(encoderVoltageRange)     .append("\n");
        sb.append("  encoderResolution=")       .append(encoderResolution)       .append("\n");
        sb.append("  encoderResolutionFormula=").append(encoderResolutionFormula).append("\n");

        return sb.toString();
    }

    public static void main(String[] args) {
        var motorSpec = new MotorSpec();
        System.out.println("motorSpec\n");
        System.out.println(motorSpec);
    }
}
