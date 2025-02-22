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

import java.util.logging.Logger;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.base.config.HardwareConfig;
import org.firstinspires.ftc.teamcode.base.config.MotorConfig;
import org.firstinspires.ftc.teamcode.base.config.MotorEnum;
import org.firstinspires.ftc.teamcode.base.logging.RobotLogger;

@Autonomous
public class Rig1MotorCalib extends LinearOpMode {
    String             className  = "Rig1MotorCalib";
    String             methodName = "runOpMode";
    String             robotName  = "Rig1Motor";
    MotorEnum          motorEnum  = MotorEnum.TESTING_MOTOR;

    Logger             logger;
    HardwareConfig     hardwareConfig;
    MotorConfig        motorConfig;
    DcMotorEx          motor;
    MotorProfiles      motorProfiles;

    public void runOpMode(){
        sleep(3000);
        try {
            logger         = RobotLogger.getInstance().getConfigLogger();
            logger.logp(INFO, className, methodName, "Created configLogger");
            hardwareConfig = HardwareConfig.makeInstance(hardwareMap, robotName);
            logger.logp(INFO, className, methodName, "Created hardwareConfig");
            motorConfig    = hardwareConfig.getMotorConfig(motorEnum);
            logger.logp(INFO, className, methodName, "got motorConfig: " + motorEnum);
            motor          = motorConfig.motor;

            motorProfiles  = new MotorProfiles(motorConfig);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Done with initialization", "");

        int    Pi    = 0;
        int    Pf    = 5 * (int) motorConfig.motorSpec.encoderResolution;

        telemetry.addData("Starting Profile Calculations", "");
        telemetry.addData("Pi", Pi);
        telemetry.addData("Pf", Pf);
        telemetry.update();
        motorProfiles.calcProfiles(Pi, Pf);

        telemetry.addData("Starting to Generate MotorCalibResult", "");
        telemetry.update();

        MotorCalibResult result = motorProfiles.getCalibResult(true, true);
        telemetry.addData("Finished calibration", "");
        telemetry.addData("Starting to write calibration result json", "");
        telemetry.update();

        result.writeJSON();

        telemetry.addData("Finished finished writing calibration result jsons", "");
        telemetry.addData("PVAFunctionF",     result.PVAFunctionF);
        telemetry.addData("PVAFunctionR",     result.PVAFunctionR);
        telemetry.addData("FORWARD Vss(0.4)", result.getVss(  0.4));
        telemetry.addData("FORWARD Vss(0.5)", result.getVss(  0.5));
        telemetry.addData("FORWARD Vss(0.6)", result.getVss(  0.6));
        telemetry.addData("FORWARD Vss(0.7)", result.getVss(  0.7));
        telemetry.addData("FORWARD Vss(0.8)", result.getVss(  0.8));
        telemetry.addData("FORWARD Vss(0.9)", result.getVss(  0.9));
        telemetry.addData("FORWARD Vss(1.0)", result.getVss(  1.0));
        telemetry.addData("REVERSE Vss(-0.4)", result.getVss(-0.4));
        telemetry.addData("REVERSE Vss(-0.5)", result.getVss(-0.5));
        telemetry.addData("REVERSE Vss(-0.6)", result.getVss(-0.6));
        telemetry.addData("REVERSE Vss(-0.7)", result.getVss(-0.7));
        telemetry.addData("REVERSE Vss(-0.8)", result.getVss(-0.8));
        telemetry.addData("REVERSE Vss(-0.9)", result.getVss(-0.9));
        telemetry.addData("REVERSE Vss(-1.0)", result.getVss(-1.0));
        telemetry.addData("Starting to write metrics for the calibration result", "");
        telemetry.update();

        result.writeMetrics();

        sleep(60000);

        telemetry.addData("Finished writing metrics for the calibration result", "");
        telemetry.addData("Starting to write JSONs for Profiles", "");
        telemetry.update();
        motorProfiles.writeJSONs();

        telemetry.addData("Finished writing JSONs for Profiles", "");
        telemetry.addData("Starting to write Metrics for Profiles", "");
        telemetry.update();
        motorProfiles.writeMetrics();

        telemetry.addData("Completed writing metrics for profiles", "");
        telemetry.addData("Waiting for start", "");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            sleep(500);
        }
    }
}
