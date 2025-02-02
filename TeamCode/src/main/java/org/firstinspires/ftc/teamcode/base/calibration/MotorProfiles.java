package org.firstinspires.ftc.teamcode.base.calibration;

import static java.lang.Math.floor;

import org.firstinspires.ftc.teamcode.base.config.MotorConfig;
import org.firstinspires.ftc.teamcode.base.config.Validatable;
import org.firstinspires.ftc.teamcode.base.logging.RobotLogger;

import java.util.logging.Level;
import java.util.logging.Logger;

public class MotorProfiles implements Validatable {
    private transient final MotorConfig          motorConfig;
    private transient final Logger               logger;
    private           final int                  powerResolution;
    private           final double               minPower;
    private           final double               maxPower;
    private           final double               dP;
    private                 int                  Pi;
    private                 int                  Ptarget;
    private           final MotorProfileConstP[] motorProfilesF;
    private           final MotorProfileConstP[] motorProfilesR;

    public MotorProfiles(MotorConfig motorConfig_in) {
        logger               = RobotLogger.getInstance().getConfigLogger();
        motorConfig          = motorConfig_in;
        powerResolution      = motorConfig.calibParams.powerResolution;
        minPower             = motorConfig.calibParams.minPower;
        maxPower             = motorConfig.calibParams.maxPower;
        dP                   = (maxPower-minPower)/powerResolution;
        motorProfilesF = new MotorProfileConstP[powerResolution];
        motorProfilesR = new MotorProfileConstP[powerResolution];
        for(int i=0; i<powerResolution; i++) {
            motorProfilesF[i] = new MotorProfileConstP(motorConfig);
            motorProfilesR[i] = new MotorProfileConstP(motorConfig);
        }
    }

    public void calcProfiles(int Pi_in, int Ptarget_in) {
        Pi                   = Pi_in;
        Ptarget              = Ptarget_in;
        for(int pIdx=0; pIdx<powerResolution; pIdx++) {
            double power     = minPower + pIdx*dP;
            String logMsg    = "Profile=" + pIdx + " power=" + power;
            logger.logp(Level.INFO, "MotorProfiles", "calcProfiles", logMsg);
            logMsg           = "Calculating Profile from=" + Pi + " to=" + Ptarget;
            logger.logp(Level.INFO, "MotorProfiles", "calcProfiles", logMsg);
            motorProfilesF[pIdx].calcProfile(power, Pi,      Ptarget);
            logMsg           = "Calculating Profile from=" + Ptarget + " to=" + Pi;
            logger.logp(Level.INFO, "MotorProfiles", "calcProfiles", logMsg);
            motorProfilesR[pIdx].calcProfile(power, Ptarget, Pi);
        }
    }

    public void writeMetrics() {
        for(int pIdx=0; pIdx<powerResolution; pIdx++) {
            motorProfilesF[pIdx].writeMetrics();
            motorProfilesR[pIdx].writeMetrics();
        }
    }

    public void writeJSONs() {
        for(int pIdx=0; pIdx<powerResolution; pIdx++) {
            motorProfilesF[pIdx].writeJSON();
            motorProfilesR[pIdx].writeJSON();
        }
    }

    public boolean isValid() {
        return true;
    }
}
