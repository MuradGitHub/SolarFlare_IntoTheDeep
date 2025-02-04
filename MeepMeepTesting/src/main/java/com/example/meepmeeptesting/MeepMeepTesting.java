package com.example.meepmeeptesting;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.MinMax;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


import org.jetbrains.annotations.NotNull;

public class MeepMeepTesting {
    static MeepMeep meepMeep = new MeepMeep(500);
    static RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
            .setDimensions(12.4375,15.0625)
            .build();

    public static void main(String[] args) {


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();




        Action onePlusThreeBucket = myBot.getDrive().actionBuilder(new Pose2d(-42,-62.5,Math.toRadians(90)))
                // Score preload
                .strafeToLinearHeading(new Vector2d(-9,-58), Math.toRadians(90))
                .waitSeconds(1.5)
                // Go to sample zone 1
                .strafeToLinearHeading(new Vector2d(-48,-53), Math.toRadians(90))
                .waitSeconds(1.5)
                // Score bucket
                .strafeToLinearHeading(new Vector2d(-54,-54), Math.toRadians(45))
                .waitSeconds(2)
                // Go to sample zone 2
                .strafeToLinearHeading(new Vector2d(-57,-50), Math.toRadians(90))
                .waitSeconds(1.5)
                // Score bucket
                .strafeToLinearHeading(new Vector2d(-54,-54), Math.toRadians(45))
                .waitSeconds(2)
                // Turn and score bucket
                .strafeToLinearHeading(new Vector2d(-61,-50), Math.toRadians(105))
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(-54,-54), Math.toRadians(45))
                .waitSeconds(2)
                // Park
                .strafeToLinearHeading(new Vector2d(-30,-6), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-23.4,-6), Math.toRadians(0))
                .waitSeconds(1)
                .build();
        Action onePlusThreeBucket1 = myBot.getDrive().actionBuilder(new Pose2d(-42,-62.5,Math.toRadians(270)))
                // Score preload
                .strafeToLinearHeading(new Vector2d(-9,-58), Math.toRadians(270))
                .build();
        Action onePlusThreeBucket2 = myBot.getDrive().actionBuilder(new Pose2d(-9,-58, Math.toRadians(270)))
                // Go to sample zone 1
                .strafeToLinearHeading(new Vector2d(-48,-53), Math.toRadians(270))
                .build();
        Action onePlusThreeBucket3 = myBot.getDrive().actionBuilder(new Pose2d(-48,-53, Math.toRadians(270)))
                // Score bucket
                .strafeToLinearHeading(new Vector2d(-54,-54), Math.toRadians(225))
                .build();
        Action onePlusThreeBucket4 = myBot.getDrive().actionBuilder(new Pose2d(-54,-54, Math.toRadians(225)))
                // Sample zone 2
                .strafeToLinearHeading(new Vector2d(-57,-50), Math.toRadians(270))
                .build();
        Action onePlusThreeBucket5 = myBot.getDrive().actionBuilder(new Pose2d(-57,-50, Math.toRadians(270)))
                // Score bucket
                .strafeToLinearHeading(new Vector2d(-54,-54), Math.toRadians(225))
                .build();
        Action onePlusThreeBucket6 = myBot.getDrive().actionBuilder(new Pose2d(-54,-54, Math.toRadians(225)))
                // turn and score bucket
                .strafeToLinearHeading(new Vector2d(-61,-50), Math.toRadians(285))
                .build();
        Action onePlusThreeBucket7 = myBot.getDrive().actionBuilder(new Pose2d(-61,-50, Math.toRadians(285)))
                // turn and score bucket
                .strafeToLinearHeading(new Vector2d(-54,-54), Math.toRadians(225))
                .build();
        Action onePlusThreeBucket8 = myBot.getDrive().actionBuilder(new Pose2d(-54,-54, Math.toRadians(225)))
                // park
                .strafeToLinearHeading(new Vector2d(-35,-6), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-23.4,-6), Math.toRadians(180))
                .build();










        Action fiveSampleAuto1= myBot.getDrive().actionBuilder(new Pose2d(-42,-62.5,Math.toRadians(270)))
                // Score sample preload
                .strafeToLinearHeading(new Vector2d(-54,-54), Math.toRadians(225))
                .build();
        Action fiveSampleAuto2= myBot.getDrive().actionBuilder(new Pose2d(-54,-54, Math.toRadians(225)))
                // Go to sample zone 1
                .strafeToLinearHeading(new Vector2d(-55,-52.5), Math.toRadians(270))
                .build();
        Action fiveSampleAuto3 = myBot.getDrive().actionBuilder(new Pose2d(-55,-52.5, Math.toRadians(270)))
                // Score bucket
                .strafeToLinearHeading(new Vector2d(-55,-55), Math.toRadians(225))
                .build();
        Action fiveSampleAuto4 = myBot.getDrive().actionBuilder(new Pose2d(-55,-55, Math.toRadians(225)))
                // Sample zone 2
                .strafeToLinearHeading(new Vector2d(-63,-52), Math.toRadians(273))
                .build();
        Action fiveSampleAuto5 = myBot.getDrive().actionBuilder(new Pose2d(-63,-52, Math.toRadians(273)))
                // Score bucket
                .strafeToLinearHeading(new Vector2d(-54,-54), Math.toRadians(225))
                .build();
        Action fiveSampleAuto6 = myBot.getDrive().actionBuilder(new Pose2d(-54,-54, Math.toRadians(225)))
                // sample zone 3
                .strafeToLinearHeading(new Vector2d(-67,-52), Math.toRadians(285))
                .build();
        Action fiveSampleAuto7 = myBot.getDrive().actionBuilder(new Pose2d(-67,-52, Math.toRadians(285)))
                // turn and score bucket
                .strafeToLinearHeading(new Vector2d(-54,-54), Math.toRadians(225))
                .build();
        Action fiveSampleAuto8 = myBot.getDrive().actionBuilder(new Pose2d(-54,-54, Math.toRadians(225)))
                .strafeToLinearHeading(new Vector2d(0,-52), Math.toRadians(180))
                .build();
        Action fiveSampleAuto9 = myBot.getDrive().actionBuilder(new Pose2d(0,-52, Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(-55,-55), Math.toRadians(225))
                .build();
        Action fiveSampleAuto10 = myBot.getDrive().actionBuilder(new Pose2d(-55,-55, Math.toRadians(225)))
                // park
                .strafeToLinearHeading(new Vector2d(-44,-6), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-24.2,-6), Math.toRadians(0))
                .build();
        Action oneSpecimenPlusThreeBucket1 = myBot.getDrive().actionBuilder(new Pose2d(-42,-62.5,Math.toRadians(270)))
                // Score preload
                .strafeToLinearHeading(new Vector2d(-3,-47), Math.toRadians(270))
                .build();
        Action oneSpecimenPlusThreeBucket2 = myBot.getDrive().actionBuilder(new Pose2d(-3,-47, Math.toRadians(270)))
                // Go to sample zone 1
                .strafeToLinearHeading(new Vector2d(-55,-52.5), Math.toRadians(270))
                .build();
        Action oneSpecimenPlusThreeBucket3 = myBot.getDrive().actionBuilder(new Pose2d(-55,-52.5, Math.toRadians(270)))
                // Score bucket
                .strafeToLinearHeading(new Vector2d(-55,-55), Math.toRadians(225))
                .build();
        Action oneSpecimenPlusThreeBucket4 = myBot.getDrive().actionBuilder(new Pose2d(-55,-55, Math.toRadians(225)))
                // Sample zone 2
                .strafeToLinearHeading(new Vector2d(-63,-52), Math.toRadians(273))
                .build();
        Action oneSpecimenPlusThreeBucket5 = myBot.getDrive().actionBuilder(new Pose2d(-63,-52, Math.toRadians(273)))
                // Score bucket
                .strafeToLinearHeading(new Vector2d(-54,-54), Math.toRadians(225))
                .build();
        Action oneSpecimenPlusThreeBucket6 = myBot.getDrive().actionBuilder(new Pose2d(-54,-54, Math.toRadians(225)))
                // sample zone 3
                .strafeToLinearHeading(new Vector2d(-67,-52), Math.toRadians(285))
                .build();
        Action oneSpecimenPlusThreeBucket7 = myBot.getDrive().actionBuilder(new Pose2d(-67,-52, Math.toRadians(285)))
                // turn and score bucket
                .strafeToLinearHeading(new Vector2d(-54,-54), Math.toRadians(225))
                .build();
        Action oneSpecimenPlusThreeBucket8 = myBot.getDrive().actionBuilder(new Pose2d(-54,-54, Math.toRadians(225)))
                // park
                .strafeToLinearHeading(new Vector2d(-44,-6), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-24.2,-6), Math.toRadians(0))

                .build();


        Action oneSpecimenPlusThreeSpecimen1 = myBot.getDrive().actionBuilder(new Pose2d(12.4375/2,-70+15.0625/2,Math.toRadians(90)))
                .waitSeconds(0.2)
                .strafeToLinearHeading(new Vector2d(12.4375/2,-37), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(38,-38),Math.toRadians(45))
                .waitSeconds(0.001)
                .splineToLinearHeading(new Pose2d(43,-41,Math.toRadians(-50)),Math.toRadians(-45))
                .splineToSplineHeading(new Pose2d(46,-38,Math.toRadians(37)),Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(52,-41,Math.toRadians(-55)),Math.toRadians(-45))
                .splineToSplineHeading(new Pose2d(57,-38,Math.toRadians(37)),Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(58,-41,Math.toRadians(-10)),Math.toRadians(-80))
                .splineToSplineHeading(new Pose2d(50,-45,Math.toRadians(-110)),Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(36,-54,Math.toRadians(-90)),Math.toRadians(225))
                .strafeToLinearHeading(new Vector2d(12.4375/2,-37),Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(36,-54),Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(12.4375/2,-37),Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(36,-54),Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(12.4375/2,-37),Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(36,-54),Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(12.4375/2,-37),Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(39,-56),Math.toRadians(-45))
                .build();

        myBot.runAction(new SequentialAction(
                oneSpecimenPlusThreeSpecimen1
        ));
    }
}