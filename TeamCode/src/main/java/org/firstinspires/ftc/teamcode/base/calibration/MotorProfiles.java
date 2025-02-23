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

import static java.lang.Math.abs;

import org.firstinspires.ftc.teamcode.base.config.MotorConfig;
import org.firstinspires.ftc.teamcode.base.config.Validatable;
import org.firstinspires.ftc.teamcode.base.logging.RobotLogger;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.logging.Level;
import java.util.logging.Logger;

public class MotorProfiles implements Validatable {
    public  transient final MotorConfig          motorConfig;
    private transient final Logger               logger;
    private           final double[]             powerLevels;
    public                  int                  Pi;
    public                  int                  Ptarget;
    public            final MotorProfileConstP[] motorProfilesF;
    public            final MotorProfileConstP[] motorProfilesR;

    public                  MotorProfiles(MotorConfig motorConfig_in) {
        logger                = RobotLogger.getInstance().getConfigLogger();
        motorConfig           = motorConfig_in;
        powerLevels           = motorConfig.calibParams.powerLevels.clone();

        motorProfilesF        = new MotorProfileConstP[powerLevels.length];
        motorProfilesR        = new MotorProfileConstP[powerLevels.length];

        for(int i=0; i<powerLevels.length; i++) {
            double power      = powerLevels[i];
            motorProfilesF[i] = new MotorProfileConstP(motorConfig,  power);
            motorProfilesR[i] = new MotorProfileConstP(motorConfig, -power);
        }
    }
    public void             calcProfiles(int Pi_in, int Ptarget_in) {
        Pi                   = Pi_in;
        Ptarget              = Ptarget_in;
        for(int i=0; i<powerLevels.length; i++) {
            MotorProfileConstP profile = motorProfilesF[i];
            /*
            String logMsg    = "Profile: nominalPower=" + profile.getNominalPower() + " Pi=" + Pi + " Ptarget=" + Ptarget;
            logger.logp(Level.INFO, "MotorProfiles", "calcProfiles", logMsg);
             */
            profile.calcProfile(Pi, Ptarget);
            profile          = motorProfilesR[i];
            /*
            logMsg           = "Profile: nominalPower=" + profile.getNominalPower() + " Pi=" + Ptarget + " Ptarget=" + Pi;
            logger.logp(Level.INFO, "MotorProfiles", "calcProfiles", logMsg);
             */
            profile.calcProfile(Ptarget, Pi);
        }
    }

    public ArrayList<MotorProfileDataPoint> getDataForward() {
        ArrayList<MotorProfileDataPoint> data = new ArrayList<>();
        for(var profile: motorProfilesF) {
            data.addAll(profile.getProfileData());
        }
        return data;
    }
    public ArrayList<MotorProfileDataPoint> getDataReverse() {
        ArrayList<MotorProfileDataPoint> data = new ArrayList<>();
        for (var profile : motorProfilesR) {
            data.addAll(profile.getProfileData());
        }
        return data;
    }
    public ArrayList<MotorProfileDataPoint> getSteadyStateDataForward() {
        ArrayList<MotorProfileDataPoint> data = new ArrayList<>();
        for (var profile : motorProfilesF) {
            MotorProfileDataPoint ssDataPoint = profile.getSteadyStateVDataPoint();
            if(ssDataPoint == null)
                continue;
            data.add(ssDataPoint);
        }
        return data;
    }
    public ArrayList<MotorProfileDataPoint> getSteadyStateDataReverse() {
        ArrayList<MotorProfileDataPoint> data = new ArrayList<>();
        for (var profile : motorProfilesR) {
            MotorProfileDataPoint ssDataPoint = profile.getSteadyStateVDataPoint();
            if(ssDataPoint == null)
                continue;
            data.add(ssDataPoint);
        }
        return data;
    }

    public Double           getMinMovingPower(MotorProfileConstP[] motorProfiles) {
        MotorProfileConstP[] sortedProfiles = motorProfiles.clone();
        Arrays.sort(sortedProfiles, (MotorProfileConstP o1, MotorProfileConstP o2) ->
                Double.compare(abs(o1.getNominalPower()), abs(o2.getNominalPower())));
        Double minMovePower  = null;
        for(var profile: sortedProfiles) {
            if(profile.hasMoved())
                return minMovePower;
            else
                minMovePower = profile.getSignedNominalPower();
        }
        return minMovePower;
    }
    public MotorCalibResult getCalibResult(boolean writeMetrics, boolean writeMetricsLUT) {
        return new MotorCalibResult(
                motorConfig,
                getSteadyStateDataForward(),
                getDataForward(),
                getSteadyStateDataReverse(),
                getDataReverse(),
                getMinMovingPower(motorProfilesF),
                getMinMovingPower(motorProfilesR),
                writeMetrics,
                writeMetricsLUT
        );
    }
    public void             writeMetrics() {
        for(var profile: motorProfilesF)
            profile.writeMetrics();
        for(var profile: motorProfilesR)
            profile.writeMetrics();
    }
    public void             writeJSONs() {
        for(var profile: motorProfilesF)
            profile.writeJSON();
        for(var profile: motorProfilesR)
            profile.writeJSON();
    }
    public boolean          isValid() {
        return true;
    }
}
