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

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.base.logging.MetricsDataPoint;
import org.firstinspires.ftc.teamcode.base.logging.RobotMetricsFileSpec;
import org.firstinspires.ftc.teamcode.base.logging.RobotMetricsSpec;


public class MotorProfileDataPoint extends MetricsDataPoint {

    static {
        MetricsDataPoint.tableType  = "MotorProfileData";
        MetricsDataPoint.format     = "%1$s,%2$.3f,%3$d,%4$.3f,%5$.3f,%6$.3f,%7$.3f,%8$.3f%n";
        MetricsDataPoint.fieldNames = new String[] {
                "Direction", "Time", "Position", "Power", "Vavg", "Aavg", "ApredFun", "ApredLut"
        };
    }

    public        Direction direction;
    public        double    t;
    public        int       P;
    public        double    power;
    public        double    Vavg;
    public        double    Aavg;
    public        double    ApredFun;
    public        double    ApredLut;

    public MotorProfileDataPoint() {}

    public MotorProfileDataPoint(DcMotorSimple.Direction direction_in,
                                 double                  t_in,
                                 int                     P_in,
                                 double                  Vavg_in,
                                 double                  Aavg_in,
                                 double                  power_in) {
        direction = direction_in;
        t         = t_in;
        P         = P_in;
        power     = power_in;
        Vavg      = Vavg_in;
        Aavg      = Aavg_in;
    }

    public static RobotMetricsFileSpec getMetricsSpec(String fileId) {
        return MetricsDataPoint.getMetricsSpec(fileId);
    }

    public Object[] getFields() {
        return new Object[] {direction, t, P, power, Vavg, Aavg, ApredFun, ApredLut};
    }

    public static void main(String[] args) {
        RobotMetricsFileSpec ms = MotorProfileDataPoint.getMetricsSpec("This File");
        System.out.println(ms);
    }
}
