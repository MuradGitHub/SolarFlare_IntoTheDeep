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
package org.firstinspires.ftc.teamcode.base.testing;

import java.util.logging.Logger;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static java.util.logging.Level.INFO;

import org.firstinspires.ftc.teamcode.base.config.HardwareConfig;
import org.firstinspires.ftc.teamcode.base.config.MotorEnum;
import org.firstinspires.ftc.teamcode.base.config.RobotConfig;
import org.firstinspires.ftc.teamcode.base.logging.RobotLogger;
import org.firstinspires.ftc.teamcode.base.config.RobotState;
import org.firstinspires.ftc.teamcode.base.motorcontrol.MotorControl1D;

@Autonomous
public class Rig1Motor extends LinearOpMode {
    Logger         logger;
    RobotConfig    robotConfig;
    HardwareConfig hardwareConfig;
    RobotState     robotState;
    MotorControl1D motorControl;

    public void runOpMode(){
        sleep(3000);
        try {
            logger         = RobotLogger.getInstance().getConfigLogger();
            logger.logp(INFO, "Rig1Motor", "runOpMode", "Created configLogger");
            robotConfig    = RobotConfig.makeInstance("Rig1Motor");
            logger.logp(INFO, "Rig1Motor", "runOpMode", "Created robotConfig");
            hardwareConfig = HardwareConfig.makeInstance(hardwareMap, robotConfig);
            logger.logp(INFO, "Rig1Motor", "runOpMode", "Created hardwareConfig");
            motorControl   = new MotorControl1D(MotorEnum.TESTING_MOTOR);
            logger.logp(INFO, "Rig1Motor", "runOpMode", "Created motorControl");
            robotState     = RobotState.getInstance();
            logger.logp(INFO, "Rig1Motor", "runOpMode", "Created robotState");
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        telemetry.addData("Done with initialization", "");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a){
                robotState.setMotorTarget(MotorEnum.TESTING_MOTOR, 0);
            }
            else if (gamepad1.b){
                robotState.setMotorTarget(MotorEnum.TESTING_MOTOR, 500);
            }
            else if (gamepad1.y){
                robotState.setMotorTarget(MotorEnum.TESTING_MOTOR, 1500);
            }
            else if (gamepad1.x){
                robotState.setMotorTarget(MotorEnum.TESTING_MOTOR, 2000);
            }

            motorControl.runTrapezoidalMotionProfile(telemetry);
        }
    }
}
