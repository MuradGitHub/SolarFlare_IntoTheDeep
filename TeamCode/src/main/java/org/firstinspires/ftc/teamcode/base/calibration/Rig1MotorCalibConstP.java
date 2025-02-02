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

import static java.util.logging.Level.INFO;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.base.config.HardwareConfig;
import org.firstinspires.ftc.teamcode.base.config.MotorConfig;
import org.firstinspires.ftc.teamcode.base.config.MotorEnum;
import org.firstinspires.ftc.teamcode.base.logging.RobotLogger;


import java.util.logging.Logger;

@Autonomous
public class Rig1MotorCalibConstP extends LinearOpMode {
    String             className  = "Rig1MotorCalibConstP";
    String             methodName = "runOpMode";
    String             robotName  = "Rig1Motor";
    MotorEnum          motorEnum  = MotorEnum.TESTING_MOTOR;

    Logger             logger;
    HardwareConfig     hardwareConfig;
    MotorConfig        motorConfig;
    DcMotorEx          motor;
    MotorProfileConstP motorProfileF;
    MotorProfileConstP motorProfileR;

    public void runOpMode(){
        sleep(3000);
        try {
            logger         = RobotLogger.getInstance().getConfigLogger();
            logger.logp(INFO, className, methodName, "Created configLogger");
            hardwareConfig = HardwareConfig.createInstance(hardwareMap, robotName);
            logger.logp(INFO, className, methodName, "Created hardwareConfig");
            motorConfig    = hardwareConfig.getMotorConfig(motorEnum);
            logger.logp(INFO, className, methodName, "got motorConfig: " + motorEnum);
            motor          = motorConfig.motor;

            motorProfileF  = new MotorProfileConstP(motorConfig);
            motorProfileR  = new MotorProfileConstP(motorConfig);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Done with initialization", "");


        int    Pi    = 0;
        int    Pf    = 10 * (int) motorConfig.motorSpec.encoderResolution;
        double power = 0.5;

        telemetry.addData("Starting Forward Profile", "");
        telemetry.update();
        motorProfileF.calcProfile(power, Pi, Pf);

        telemetry.addData("Starting to write JSON for Forward Profile", "");
        telemetry.update();
        motorProfileF.writeJSON();


        telemetry.addData("Starting to write Metriccs for Forward Profile", "");
        telemetry.update();
        motorProfileF.writeMetrics();

        sleep(2000);

        telemetry.addData("Starting Reverse Profile", "");
        telemetry.update();
        motorProfileR.calcProfile(power, Pf, Pi);

        telemetry.addData("Starting to write JSON for Reverse Profile", "");
        telemetry.update();
        motorProfileR.writeJSON();

        telemetry.addData("Starting to write Metriccs for Reverse Profile", "");
        telemetry.update();
        motorProfileR.writeMetrics();

        telemetry.addData("Profile Calculations", "");

        telemetry.addData("Power",               power);
        telemetry.addData("Pi",                  Pi);
        telemetry.addData("Pf",                  Pf);

        telemetry.addData("F_isValid",            motorProfileF.isValid());
        telemetry.addData("F_averagingPeriods",   motorProfileF.averagingPeriods);
        telemetry.addData("F_tIdxMax",            motorProfileF.tIdxMax);
        telemetry.addData("F_Plast",              motorProfileF.getPLast());
        telemetry.addData("F_isTargetReached",    motorProfileF.hasReachedTarget());
        telemetry.addData("F_timeToTarget",       motorProfileF.getTimeToTarget());
        telemetry.addData("F_noLoadVelocity",     motorConfig  .getNoLoadVelocity());
        telemetry.addData("F_hasSteadyStateV",    motorProfileF.hasSteadyStateV());
        telemetry.addData("F_timeToSteadyState",  motorProfileF.getTimeToSteadyState());
        telemetry.addData("F_Vss",                motorProfileF.getSteadyStateV());
        telemetry.addData("F_Vmax",               motorProfileF.Vmax);
        telemetry.addData("F_hasSteadyStateA",    motorProfileF.hasSteadyStateA());
        telemetry.addData("F_timeToSteadyStateA", motorProfileF.getTimeToSteadyStateA());
        telemetry.addData("F_Ass",                motorProfileF.getSteadyStateA());
        telemetry.addData("F_Amax",               motorProfileF.Amax);
        telemetry.addData("F_Dmax",               motorProfileF.Dmax);

        telemetry.addData("R_isValid",            motorProfileR.isValid());
        telemetry.addData("R_averagingPeriods",   motorProfileR.averagingPeriods);
        telemetry.addData("R_tIdxMax",            motorProfileR.tIdxMax);
        telemetry.addData("R_Plast",              motorProfileR.getPLast());
        telemetry.addData("R_isTargetReached",    motorProfileR.hasReachedTarget());
        telemetry.addData("R_timeToTarget",       motorProfileR.getTimeToTarget());
        telemetry.addData("R_noLoadVelocity",     motorConfig  .getNoLoadVelocity());
        telemetry.addData("R_hasSteadyStateV",    motorProfileR.hasSteadyStateV());
        telemetry.addData("R_timeToSteadyState",  motorProfileR.getTimeToSteadyState());
        telemetry.addData("R_Vss",                motorProfileR.getSteadyStateV());
        telemetry.addData("R_Vmax",               motorProfileR.Vmax);
        telemetry.addData("R_hasSteadyStateA",    motorProfileR.hasSteadyStateA());
        telemetry.addData("R_timeToSteadyStateA", motorProfileR.getTimeToSteadyStateA());
        telemetry.addData("R_Ass",                motorProfileR.getSteadyStateA());
        telemetry.addData("R_Amax",               motorProfileR.Amax);
        telemetry.addData("R_Dmax",               motorProfileR.Dmax);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            sleep(500);
        }
    }
}
