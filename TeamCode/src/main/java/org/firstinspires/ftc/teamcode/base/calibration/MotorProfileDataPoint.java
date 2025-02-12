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

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.base.logging.MetricsDataPoint;
import org.firstinspires.ftc.teamcode.base.logging.MetricsFileSpec;

import java.util.Formatter;

public class MotorProfileDataPoint extends MetricsDataPoint {

    static {
        MetricsDataPoint.tableType  = "MotorProfileData";
        MetricsDataPoint.format     =
                "%1$s,%2$.3f,%3$.3f,%4$.3f,%5$.3f,%6$.3f," +
                        "%7$d,%8$.3f,%9$.3f,%10$.3f,%11$.3f,%12$.3f," +
                        "%13$.3f,%14$.3f,%15$.3f%n";

        MetricsDataPoint.fieldNames = new String[] {
                "Direction", "Time",     "TimePExtract", "TimeVextract", "TimeCExtract", "TimeCycle",
                "Position",  "Power",    "Velocity",     "Vavg",         "A",            "Aavg",
                "ApredFun",  "ApredLut", "C"
        };
    }

    public        Direction direction;
    public        double    t;
    public        double    tPextract;
    public        double    tVextract;
    public        double    tCextract;
    public        double    tCycle;
    public        int       P;
    public        double    power;
    public        double    V;
    public        double    Vavg;
    public        double    A;
    public        double    Aavg;
    public        double    ApredFun;
    public        double    ApredLut;
    public        double    C;

    public MotorProfileDataPoint(
            Direction direction_in,
            double    t_in,
            double    tPextract_in,
            double    tVextract_in,
            double    tCextract_in,
            double    tCycle_in,
            int       P_in,
            double    V_in,
            double    power_in,
            double    C_in) {
        direction = direction_in;
        t         = t_in;
        tPextract = tPextract_in;
        tVextract = tVextract_in;
        tCextract = tCextract_in;
        tCycle    = tCycle_in;
        P         = P_in;
        V         = V_in;
        power     = power_in;
        C         = C_in;
    }

    public static MetricsFileSpec getMetricsSpec(String fileId) {
        return MetricsDataPoint.getMetricsSpec(fileId);
    }

    public void writeMetrics(Formatter formatter) {
        formatter.format(format,
                direction, t,    tPextract, tVextract, tCextract, tCycle,   P, power,
                V,         Vavg, A,          Aavg,     ApredFun,  ApredLut, C);
    }

    @NonNull
    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("MotorProfileDataPoint\n");

        sb.append("  direction=").append(direction).append("\n");
        sb.append("  t=")        .append(t)        .append("\n");
        sb.append("  tPextract=").append(tPextract).append("\n");
        sb.append("  tVextract=").append(tVextract).append("\n");
        sb.append("  tCextract=").append(tCextract).append("\n");
        sb.append("  tCycle=")   .append(tCycle)   .append("\n");
        sb.append("  P=")        .append(P)        .append("\n");
        sb.append("  V=")        .append(V)        .append("\n");
        sb.append("  Vavg=")     .append(Vavg)     .append("\n");
        sb.append("  A=")        .append(A)        .append("\n");
        sb.append("  Aavg=")     .append(Aavg)     .append("\n");
        sb.append("  ApredFun=") .append(ApredFun) .append("\n");
        sb.append("  ApredLut=") .append(ApredLut) .append("\n");
        sb.append("  power=")    .append(power)    .append("\n");
        sb.append("  C=")        .append(C)        .append("\n");

        return sb.toString();
    }

    public static void main(String[] args) {
        MetricsFileSpec ms = MotorProfileDataPoint.getMetricsSpec("This File");
        System.out.println(ms);
    }
}
