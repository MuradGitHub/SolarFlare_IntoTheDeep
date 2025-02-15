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

import org.firstinspires.ftc.teamcode.base.config.JSONWritable;
import org.firstinspires.ftc.teamcode.base.config.MotorConfig;
import org.firstinspires.ftc.teamcode.base.config.MotorEnum;
import org.firstinspires.ftc.teamcode.base.config.RobotConfig;
import org.firstinspires.ftc.teamcode.base.logging.MultiMetricsWriter;
import org.firstinspires.ftc.teamcode.base.logging.RobotMetrics;
import org.firstinspires.ftc.teamcode.base.logging.MetricsFile;
import org.firstinspires.ftc.teamcode.base.math.LookupTable1D;
import org.firstinspires.ftc.teamcode.base.math.LookupTable2D;
import org.firstinspires.ftc.teamcode.base.math.Range;
import org.firstinspires.ftc.teamcode.base.utils.JSONUtils;

import static org.firstinspires.ftc.teamcode.base.math.Math.regularizeUp;
import static org.firstinspires.ftc.teamcode.base.math.Math.regularizeDown;


public class MotorCalibResult extends MultiMetricsWriter implements JSONWritable {
    public MotorEnum                            motorEnum;
    public int                                  powerResolution;
    public int                                  velocityResolution;

    public String                               metricsSpecId = "MotorProfileData";
    public ArrayList<MotorProfileDataPoint>     ssDataF;
    public ArrayList<MotorProfileDataPoint>     ssDataR;
    public ArrayList<MotorProfileDataPoint>     dataF;
    public ArrayList<MotorProfileDataPoint>     dataR;
    public LookupTable1D                        Vss             = new LookupTable1D();
    public LookupTable2D                        PVALutF;
    public LookupTable2D                        PVALutR;
    public Range                                VssRangeF       = new Range();
    public Range                                VssRangeR       = new Range();
    public MotorPVAFunction                     PVAFunctionF;
    public MotorPVAFunction                     PVAFunctionR;
    public boolean                              writeMetrics;
    public boolean                              writeMetricsLUT;

    /**
     * Constructor
     *
     * @param motorConfig_in motor config
     * @param ssDataF_in     steady state data for forward profiles
     * @param dataF_in       timed data up to steady state for forward profiles
     * @param ssDataR_in     steady state data for reverse profiles
     * @param dataR_in       timed data up to steady state for reverse profiles
     */
    public MotorCalibResult(MotorConfig                      motorConfig_in,
                            ArrayList<MotorProfileDataPoint> ssDataF_in,
                            ArrayList<MotorProfileDataPoint> dataF_in,
                            ArrayList<MotorProfileDataPoint> ssDataR_in,
                            ArrayList<MotorProfileDataPoint> dataR_in,
                            boolean                          writeMetrics_in,
                            boolean                          writeMetricsLUT_in
                            ) {
        motorEnum             = motorConfig_in.motorEnum;
        powerResolution       = motorConfig_in.calibParams.powerResolution;
        velocityResolution    = motorConfig_in.calibParams.velocityResolution;
        ssDataF               = ssDataF_in;
        dataF                 = dataF_in;
        ssDataR               = ssDataR_in;
        dataR                 = dataR_in;
        writeMetrics          = writeMetrics_in;
        writeMetricsLUT       = writeMetricsLUT_in;

        initMetricsSpecs();
        fitFunctions();
        calcPredictions();
    }

    public void initMetricsSpecs() {
        addMetricsSpec(metricsSpecId,MotorProfileDataPoint.makeMetricsSpec(motorEnum.name()));
    }

    /**
     *  Motor Dynamic Equation a = k1 * (power - k2*v - k3)
     *      - First fit when a=0
     *        power = k2*v + k3
     *      - Then, k1 is the fit to the equation a = k1*(power - k2*v - k3)
     */
    public void fitFunctions() {
        WeightedObservedPoints obs     = new WeightedObservedPoints();
        PolynomialCurveFitter  fitter  = PolynomialCurveFitter.create(1);
        double[]               coeff;

        /// First fit k2 and k3
        /// Coefficients are returned in increasing order of the term
        /// Forward data
        for(var dataPoint: ssDataF) {
            obs.add(dataPoint.Vavg, dataPoint.power);
            Vss.addDataPoint(dataPoint.power, dataPoint.Vavg);
        }
        coeff                          = fitter.fit(obs.toList());
        double k3F                     = coeff[0];
        double k2F                     = coeff[1];
        // Reverse data
        obs.clear();
        for(var dataPoint: ssDataR) {
            obs.add(dataPoint.Vavg, dataPoint.power);
            Vss.addDataPoint(dataPoint.power, dataPoint.Vavg);
        }
        coeff                          = fitter.fit(obs.toList());
        double k3R                     = coeff[0];
        double k2R                     = coeff[1];

        /// Now fit k1
        /// Forward data
        obs.clear();
        for(var dataPoint: dataF) {
            VssRangeF.update(dataPoint.Vavg);
            /// x = (power - k2*v - k3)
            double x = dataPoint.power - k2F * dataPoint.Vavg - k3F;
            obs.add(x, dataPoint.Aavg);
        }
        coeff                          = fitter.fit(obs.toList());
        double kAresF                  = coeff[0];
        double k1F                     = coeff[1];

        /// Now fit k1
        /// Reverse data
        obs.clear();
        for(var dataPoint: dataR) {
            VssRangeR.update(dataPoint.Vavg);
            /// x = (power - k2*v - k3)
            double x = dataPoint.power - k2R * dataPoint.Vavg - k3R;
            obs.add(x, dataPoint.Aavg);
        }
        coeff                          = fitter.fit(obs.toList());
        double kAresR                  = coeff[0];
        double k1R                     = coeff[1];

        PVAFunctionF                   = new MotorPVAFunction(k1F,k2F,k3F,kAresF);
        PVAFunctionR                   = new MotorPVAFunction(k1R,k2R,k3R,kAresR);

        /// Fit the LUTs
        PVALutF                        = new LookupTable2D(
                powerResolution, 0.0, 1.0,
                velocityResolution,
                regularizeDown(VssRangeF.min,2),
                regularizeUp(VssRangeF.max,2));

        PVALutF.setMetricsFileId("MotorCalibResult-PVALutF-" + motorEnum.name());

        PVALutR = new LookupTable2D(
                powerResolution, -1.0, 0.0,
                velocityResolution,
                regularizeDown(VssRangeR.min,2),
                regularizeUp(VssRangeR.max,2));

        PVALutR.setMetricsFileId("MotorCalibResult-PVALutR-" + motorEnum.name());

        /// add points to the LUTs
        for(var dataPoint: dataF)
            PVALutF.addDataPoint(dataPoint.power, dataPoint.Vavg, dataPoint.Aavg);
        if(writeMetricsLUT)
            PVALutF.writeMetricsPrefill();
        PVALutF.update();

        for(var dataPoint: dataR)
            PVALutR.addDataPoint(dataPoint.power, dataPoint.Vavg, dataPoint.Aavg);
        if(writeMetricsLUT)
            PVALutR.writeMetricsPrefill();
        PVALutR.update();

        calcPredictions();
    }

    public void calcPredictions() {
        for(MotorProfileDataPoint Pt: dataF) {
            Pt.ApredFun = getAccelByFunction(Pt.direction, Pt.power, Pt.Vavg);
            Pt.ApredLut = getAccelByLut(Pt.direction, Pt.power, Pt.Vavg);
        }

        for(MotorProfileDataPoint Pt: dataR) {
            Pt.ApredFun = getAccelByFunction(Pt.direction, Pt.power, Pt.Vavg);
            Pt.ApredLut = getAccelByLut(Pt.direction, Pt.power, Pt.Vavg);
        }
    }

    public double getAccelByFunction(DcMotorSimple.Direction direction, double power, double velocity) {
        if(direction == DcMotorSimple.Direction.FORWARD)
            return PVAFunctionF.getAccel(power, velocity);
        else
            return PVAFunctionR.getAccel(power, velocity);
    }

    public double getAccelByLut(DcMotorSimple.Direction direction, double power, double velocity) {
        if(direction == DcMotorSimple.Direction.FORWARD)
            return PVALutF.interpolate(power, velocity);
        else
            return PVALutR.interpolate(power, velocity);
    }

    public double getVss(DcMotorSimple.Direction direction, double power) {
        return Vss.apply(power);
    }

    public void setWriteMetricsLUT(boolean writeMetricsLUT_in) {
        writeMetricsLUT = writeMetricsLUT_in;
    }

    public void writeMetrics() {
        if(writeMetricsLUT) {
            PVALutF.writeMetrics();
            PVALutR.writeMetrics();
        }

        MetricsFile metricsFile = RobotMetrics
                .getInstance()
                .getMetricsFile(getMetricsSpec(metricsSpecId));

        for(MotorProfileDataPoint dataPoint: dataF)
            metricsFile.addData(dataPoint);

        for(MotorProfileDataPoint dataPoint: dataR)
            metricsFile.addData(dataPoint);

        metricsFile.close();
    }

    public static String getJSONFileName(MotorEnum motorEnum) {
        return MotorCalibResult.class.getSimpleName() + "-" + getJSONFileId(motorEnum) + ".json";
    }

    public static String getJSONFileId(MotorEnum motorEnum) {
        return String.format(Locale.US, "%1$s", motorEnum.name());
    }

    public String getJSONFileId() {
        return getJSONFileId(motorEnum);
    }

    public void writeJSON() {
        JSONUtils.writeJSON(this);
    }

    public static void main(String[] args) {
        RobotConfig robotConfig = RobotConfig.makeInstance("Rig1Motor");
        MotorConfig motorConfig = robotConfig.motors.get(MotorEnum.TESTING_MOTOR);
        MotorCalibResult result = new MotorCalibResult(
                motorConfig,
                null,
                null,
                null,
                null,
                false,
                false);

        System.out.println(result.getMetricsSpec("MotorProfileData"));
    }
}
