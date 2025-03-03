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
package org.firstinspires.ftc.teamcode.base.autonomous;


// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PinpointDrive;

@Config
@Autonomous(name = "BearOneWander", group = "Autonomous")
public class BearOneWander extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-41, -62.5, Math.toRadians(90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);

        Action onePlusThreeBucket1 = drive.actionBuilder(new Pose2d(-41,-62.5, Math.toRadians(90)))
                // Score preload bucket
                .strafeToLinearHeading(new Vector2d(-64,-54), Math.toRadians(45))
                .build();
        Action onePlusThreeBucket4 = drive.actionBuilder(new Pose2d(-64,-54, Math.toRadians(45)))
                // Sample zone 1
                .strafeToLinearHeading(new Vector2d(-52.5,-55.5), Math.toRadians(90))
                .build();
        Action onePlusThreeBucket5 = drive.actionBuilder(new Pose2d(-52.5,-55.5, Math.toRadians(90)))
                // Score bucket
                .strafeToLinearHeading(new Vector2d(-58.3,-54.3), Math.toRadians(37))
                .build();
        Action onePlusThreeBucket6 = drive.actionBuilder(new Pose2d(-58.3,-54.3, Math.toRadians(37)))
                // sample zone 2
                .strafeToLinearHeading(new Vector2d(-67.5,-53), Math.toRadians(95))
                .build();
        Action onePlusThreeBucket7 = drive.actionBuilder(new Pose2d(-67.5,-53, Math.toRadians(95)))
                // turn and score bucket
                .strafeToLinearHeading(new Vector2d(-56.5,-53.5), Math.toRadians(49))
                .build();
        Action onePlusThreeBucket8 = drive.actionBuilder(new Pose2d(-56.5,-53.5, Math.toRadians(49)))
                // sample 3
                .strafeToLinearHeading(new Vector2d(-67 ,-54), Math.toRadians(113))
                .build();
        Action onePlusThreeBucket9 = drive.actionBuilder(new Pose2d(-67,-54, Math.toRadians(113)))
                // obs zone
                .strafeToLinearHeading(new Vector2d(-56,-56), Math.toRadians(45))
                .build();

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                onePlusThreeBucket1,
                                onePlusThreeBucket5,
                                new SleepAction(0.4),
                                onePlusThreeBucket6,
                                new SleepAction(0.2),
                                onePlusThreeBucket7,
                                onePlusThreeBucket8,
                                onePlusThreeBucket9
                        )
                )
        );
    }
}
