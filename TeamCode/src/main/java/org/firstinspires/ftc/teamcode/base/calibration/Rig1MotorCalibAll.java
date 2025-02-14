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

import java.util.Objects;
import java.util.logging.Logger;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.base.config.HardwareConfig;
import org.firstinspires.ftc.teamcode.base.config.MotorConfig;
import org.firstinspires.ftc.teamcode.base.logging.RobotLogger;

@Autonomous
public class Rig1MotorCalibAll extends LinearOpMode {
    String          className  = "Rig1MotorCalibAll";
    String          methodName = "runOpMode";
    String          robotName  = "Rig1Motor";

    Logger          logger;
    HardwareConfig  hardwareConfig;


    public void runOpMode(){
        sleep(3000);
        try {
            logger                      = RobotLogger.getInstance().getConfigLogger();
            hardwareConfig              = HardwareConfig.createInstance(hardwareMap, robotName);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        for(var motorEnum: Objects.requireNonNull(hardwareConfig.robotConfig.calibration.get("motors"))) {
            logger.logp(INFO, className, methodName, "processing: " + motorEnum);
            MotorConfig   motorConfig   = hardwareConfig.getMotorConfig(motorEnum);
            DcMotorEx     motor         = motorConfig.motor;
            MotorProfiles motorProfiles = new MotorProfiles(motorConfig);
            int           Pi            = motorConfig.minTarget;
            int           Pf            = motorConfig.maxTarget;
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            telemetry.addData("Starting Profiles Calculations", motorEnum);
            telemetry.addData("Pi", Pi);
            telemetry.addData("Pf", Pf);
            telemetry.update();
            motorProfiles.calcProfiles(Pi, Pf);

            telemetry.addData("Starting to Generate CalibResult", motorEnum);
            telemetry.update();

            MotorCalibResult result = motorProfiles.getCalibResult(true, true);
            telemetry.addData("Finished calibration", motorEnum);
            telemetry.addData("Starting to write CalibResult json", motorEnum);
            telemetry.update();

            result.writeJSON();

            telemetry.addData("Finished finished writing CalibResult json", motorEnum);
            telemetry.addData("PVAFunctionF", result.PVAFunctionF);
            telemetry.addData("PVAFunctionR", result.PVAFunctionR);
            telemetry.addData("FORWARD Vss(0.4)", result.getVss(Direction.FORWARD, 0.4));
            telemetry.addData("FORWARD Vss(0.5)", result.getVss(Direction.FORWARD, 0.5));
            telemetry.addData("FORWARD Vss(0.6)", result.getVss(Direction.FORWARD, 0.6));
            telemetry.addData("FORWARD Vss(0.7)", result.getVss(Direction.FORWARD, 0.7));
            telemetry.addData("FORWARD Vss(0.8)", result.getVss(Direction.FORWARD, 0.8));
            telemetry.addData("FORWARD Vss(0.9)", result.getVss(Direction.FORWARD, 0.9));
            telemetry.addData("FORWARD Vss(1.0)", result.getVss(Direction.FORWARD, 1.0));
            telemetry.addData("REVERSE Vss(-0.4)", result.getVss(Direction.REVERSE, -0.4));
            telemetry.addData("REVERSE Vss(-0.5)", result.getVss(Direction.REVERSE, -0.5));
            telemetry.addData("REVERSE Vss(-0.6)", result.getVss(Direction.REVERSE, -0.6));
            telemetry.addData("REVERSE Vss(-0.7)", result.getVss(Direction.REVERSE, -0.7));
            telemetry.addData("REVERSE Vss(-0.8)", result.getVss(Direction.REVERSE, -0.8));
            telemetry.addData("REVERSE Vss(-0.9)", result.getVss(Direction.REVERSE, -0.9));
            telemetry.addData("REVERSE Vss(-1.0)", result.getVss(Direction.REVERSE, -1.0));
            telemetry.addData("Starting to write metrics for CalibResult", motorEnum);
            telemetry.update();

            result.writeMetrics();

            sleep(60000);

            telemetry.addData("Finished writing metrics for CalibResult", motorEnum);
            telemetry.addData("Starting to write JSONs for Profiles", motorEnum);
            telemetry.update();
            motorProfiles.writeJSONs();

            telemetry.addData("Finished writing JSONs for Profiles", "");
            telemetry.addData("Starting to write Metrics for Profiles", "");
            telemetry.update();
            motorProfiles.writeMetrics();

            telemetry.addData("Completed writing metrics for profiles", "");
            telemetry.addData("Waiting for start", "");
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            sleep(500);
        }
    }
}
