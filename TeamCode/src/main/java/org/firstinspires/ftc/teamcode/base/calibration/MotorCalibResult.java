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

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;
import java.util.Locale;

import org.apache.commons.math3.fitting.PolynomialCurveFitter;
import org.apache.commons.math3.fitting.WeightedObservedPoints;
import org.firstinspires.ftc.teamcode.base.config.MotorEnum;
import org.firstinspires.ftc.teamcode.base.logging.MetricsWritable;
import org.firstinspires.ftc.teamcode.base.logging.RobotMetrics;
import org.firstinspires.ftc.teamcode.base.logging.RobotMetricsFile;

public class MotorCalibResult implements MetricsWritable {
    public MotorEnum                        motorEnum;
    public ArrayList<MotorProfileDataPoint> ssDataF = new ArrayList<>();
    public ArrayList<MotorProfileDataPoint> ssDataR = new ArrayList<>();
    public ArrayList<MotorProfileDataPoint> dataF   = new ArrayList<>();
    public ArrayList<MotorProfileDataPoint> dataR   = new ArrayList<>();
    public EmpiricalFunction                VssF    = new EmpiricalFunction();
    public EmpiricalFunction                VssR    = new EmpiricalFunction();
    public double                           k1F;
    public double                           k2F;
    public double                           k3F;
    public double                           kAresF;
    public double                           k1R;
    public double                           k2R;
    public double                           k3R;
    public double                           kAresR;

    /**
     * Constructor
     *
     * @param motorEnum_in motor enum
     * @param ssDataF_in   steady state data for forward profiles
     * @param dataF_in     timed data up to steady state for forward profiles
     * @param ssDataR_in   steady state data for reverse profiles
     * @param dataR_in     timed data up to steady state for reverse profiles
     */
    public MotorCalibResult(MotorEnum motorEnum_in,
                            ArrayList<MotorProfileDataPoint> ssDataF_in,
                            ArrayList<MotorProfileDataPoint> dataF_in,
                            ArrayList<MotorProfileDataPoint> ssDataR_in,
                            ArrayList<MotorProfileDataPoint> dataR_in
                            ) {
        motorEnum             = motorEnum_in;
        ssDataF               = ssDataF_in;
        dataF                 = dataF_in;
        ssDataR               = ssDataR_in;
        dataR                 = dataR_in;
    }

    /**
     *  Motor Dynamic Equation a = k1 * (power - k2*v - k3)
     *      - First fit when a=0
     *        power = k2*v + k3
     *      - Then, k1 is the fit to the equaltion a = k1*(power - k2*v - k3)
     */
    public void fit() {
        WeightedObservedPoints obs     = new WeightedObservedPoints();
        PolynomialCurveFitter  fitter  = PolynomialCurveFitter.create(1);
        double[]               coeff;

        /// First fit k2 and k3
        /// Coefficients are returned in increasing order of the term
        /// Forward data
        for(var dataPoint: ssDataF) {
            obs.add(dataPoint.Vavg, dataPoint.power);
            VssF.addDataPoint(dataPoint.power, dataPoint.Vavg);
        }
        coeff                          = fitter.fit(obs.toList());
        k3F                            = coeff[0];
        k2F                            = coeff[1];
        // Reverse data
        obs.clear();
        for(var dataPoint: ssDataR) {
            obs.add(dataPoint.Vavg, dataPoint.power);
            VssR.addDataPoint(dataPoint.power, dataPoint.Vavg);
        }
        coeff                          = fitter.fit(obs.toList());
        k3R                            = coeff[0];
        k2R                            = coeff[1];

        /// Now fit k1
        /// Forward data
        obs.clear();
        for(var dataPoint: dataF) {
            /// x = (power - k2*v - k3)
            double x = dataPoint.power - k2F * dataPoint.Vavg - k3F;
            obs.add(x, dataPoint.Aavg);
        }
        coeff                          = fitter.fit(obs.toList());
        kAresF                         = coeff[0];
        k1F                            = coeff[1];

        /// Now fit k1
        /// Reverse data
        obs.clear();
        for(var dataPoint: dataR) {
            /// x = (power - k2*v - k3)
            double x = dataPoint.power - k2R * dataPoint.Vavg - k3R;
            obs.add(x, dataPoint.Aavg);
        }
        coeff                          = fitter.fit(obs.toList());
        kAresR                         = coeff[0];
        k1R                            = coeff[1];
    }

    public double getAccel(DcMotorSimple.Direction direction, double power, double velocity) {
        if(direction == DcMotorSimple.Direction.FORWARD)
            return k1F * (power - k2F*velocity - k3F) + kAresF;
        else
            return k1R * (power - k2R*velocity - k3R) + kAresR;
    }

    public double getVss(DcMotorSimple.Direction direction, double power) {
        if(direction == DcMotorSimple.Direction.FORWARD)
            return VssF.apply(power);
        else
            return VssR.apply(power);
    }

    public String getMetricsFileId() {
        return String.format(Locale.US, "%1$s", motorEnum);
    }

    public String getMetricsTableType() {
        return "MotorCalibResult";
    }

    public void writeMetrics() {
        RobotMetricsFile metricsFile = RobotMetrics.getInstance().getMetricsFile(this);



        metricsFile.close();
    }
}
