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


        double robotLength = 15.364;
        double robotWidth  = 14.375;


        Action fourSpecimenPathing1 = myBot.getDrive().actionBuilder(new Pose2d(-(robotWidth / 2), -70 + (robotLength / 2), 90))
                // Go to scoring zone first time
                .strafeToLinearHeading(new Vector2d(8,-46), Math.toRadians(90))
                .build();
        Action fourSpecimenPathing2 = myBot.getDrive().actionBuilder(new Pose2d(8, -46, Math.toRadians(90)))
                // Go to sample 1 sample zone
                .strafeToLinearHeading(new Vector2d(30.7,-45.1), Math.toRadians(50))
                .build();
        Action fourSpecimenPathing3 = myBot.getDrive().actionBuilder(new Pose2d(30.7, -45.1, Math.toRadians(50)))
                // Rotate towards observation zone 1st time
                .turnTo(Math.toRadians(-45))
                .build();
        Action fourSpecimenPathing4 = myBot.getDrive().actionBuilder(new Pose2d(30.7, -45.1, Math.toRadians(-45)))
                // Rotate to sample 2 in sample zone
                .strafeToLinearHeading(new Vector2d(40,-41), Math.toRadians(40))
                .build();
        Action fourSpecimenPathing5 = myBot.getDrive().actionBuilder(new Pose2d(40, -41, Math.toRadians(40)))
                // Rotate towards observation zone 2nd time
                .turnTo(Math.toRadians(-70))
                .build();
        Action fourSpecimenPathing6 = myBot.getDrive().actionBuilder(new Pose2d(40, -41, Math.toRadians(-70)))
                // Rotate towards sample 3 in sample zone
                .strafeToLinearHeading(new Vector2d(51,-41), Math.toRadians(40))
                .build();
        Action fourSpecimenPathing7 = myBot.getDrive().actionBuilder(new Pose2d(51, -41, Math.toRadians(40)))
                // Rotate towards observation zone 3rd time
                .turnTo(Math.toRadians(-100))
                .build();
        Action fourSpecimenPathing8 = myBot.getDrive().actionBuilder(new Pose2d(51, -41, Math.toRadians(-100)))
                // Go to pickup zone
                .strafeToLinearHeading(new Vector2d(34,-54), Math.toRadians(-90))
                .build();
        Action fourSpecimenPathing9 = myBot.getDrive().actionBuilder(new Pose2d(34, -54, Math.toRadians(-90)))
                // Score second specimen
                .strafeToLinearHeading(new Vector2d(4,-46), Math.toRadians(90))
                .build();
        Action fourSpecimenPathing10 = myBot.getDrive().actionBuilder(new Pose2d(4, -46, Math.toRadians(90)))
                // Go to pickup zone
                .strafeToLinearHeading(new Vector2d(34,-54), Math.toRadians(-90))
                .build();
        Action fourSpecimenPathing11 = myBot.getDrive().actionBuilder(new Pose2d(34, -54, Math.toRadians(-90)))
                // Score third specimen
                .strafeToLinearHeading(new Vector2d(0,-46), Math.toRadians(90))
                .build();
        Action fourSpecimenPathing12 = myBot.getDrive().actionBuilder(new Pose2d(0, -46, Math.toRadians(90)))
                // Go to pickup zone
                .strafeToLinearHeading(new Vector2d(34,-54), Math.toRadians(-90))
                .build();
        Action fourSpecimenPathing13 = myBot.getDrive().actionBuilder(new Pose2d(34, -54, Math.toRadians(-90)))
                // Score fourth specimen
                .strafeToLinearHeading(new Vector2d(-4,-46), Math.toRadians(90))
                .build();
        Action fourSpecimenPathing14 = myBot.getDrive().actionBuilder(new Pose2d(-4, -46, Math.toRadians(90)))
                // Park
                .strafeToLinearHeading(new Vector2d(34,-62), Math.toRadians(90))
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
        

        Action onePlusThreeSpecimen1 = myBot.getDrive().actionBuilder(new Pose2d(-(robotWidth / 2), -70 + (robotLength / 2), 90))
                // Go to scoring zone first time
                .strafeToLinearHeading(new Vector2d(8,-46), Math.toRadians(90))
                .build();
        Action onePlusThreeSpecimen2 = myBot.getDrive().actionBuilder(new Pose2d(8, -46, Math.toRadians(90)))
                // Go to sample 1 sample zone
                .strafeToLinearHeading(new Vector2d(30.7,-45.1), Math.toRadians(50))
                .build();
        Action onePlusThreeSpecimen3 = myBot.getDrive().actionBuilder(new Pose2d(30.7, -45.1, Math.toRadians(50)))
                // Rotate towards observation zone 1st time
                .turnTo(Math.toRadians(-45))
                .build();
        Action onePlusThreeSpecimen4 = myBot.getDrive().actionBuilder(new Pose2d(30.7, -45.1, Math.toRadians(-45)))
                // Rotate to sample 2 in sample zone
                .strafeToLinearHeading(new Vector2d(40,-41), Math.toRadians(40))
                .build();
        Action onePlusThreeSpecimen5 = myBot.getDrive().actionBuilder(new Pose2d(40, -41, Math.toRadians(40)))
                // Rotate towards observation zone 2nd time
                .turnTo(Math.toRadians(-70))
                .build();
        Action onePlusThreeSpecimen6 = myBot.getDrive().actionBuilder(new Pose2d(40, -41, Math.toRadians(-70)))
                // Rotate towards sample 3 in sample zone
                .strafeToLinearHeading(new Vector2d(51,-41), Math.toRadians(40))
                .build();
        Action onePlusThreeSpecimen7 = myBot.getDrive().actionBuilder(new Pose2d(51, -41, Math.toRadians(40)))
                // Rotate towards observation zone 3rd time
                .turnTo(Math.toRadians(-100))
                .build();
        Action onePlusThreeSpecimen8 = myBot.getDrive().actionBuilder(new Pose2d(51, -41, Math.toRadians(-100)))
                // Go to pickup zone
                .strafeToLinearHeading(new Vector2d(34,-54), Math.toRadians(-90))
                .build();
        Action onePlusThreeSpecimen9 = myBot.getDrive().actionBuilder(new Pose2d(34, -54, Math.toRadians(-90)))
                // Score second specimen
                .strafeToLinearHeading(new Vector2d(4,-46), Math.toRadians(90))
                .build();
        Action onePlusThreeSpecimen10 = myBot.getDrive().actionBuilder(new Pose2d(4, -46, Math.toRadians(90)))
                // Go to pickup zone
                .strafeToLinearHeading(new Vector2d(34,-54), Math.toRadians(-90))
                .build();
        Action onePlusThreeSpecimen11 = myBot.getDrive().actionBuilder(new Pose2d(34, -54, Math.toRadians(-90)))
                // Score third specimen
                .strafeToLinearHeading(new Vector2d(0,-46), Math.toRadians(90))
                .build();
        Action onePlusThreeSpecimen12 = myBot.getDrive().actionBuilder(new Pose2d(0, -46, Math.toRadians(90)))
                // Go to pickup zone
                .strafeToLinearHeading(new Vector2d(34,-54), Math.toRadians(-90))
                .build();
        Action onePlusThreeSpecimen13 = myBot.getDrive().actionBuilder(new Pose2d(34, -54, Math.toRadians(-90)))
                // Score fourth specimen
                .strafeToLinearHeading(new Vector2d(-4,-46), Math.toRadians(90))
                .build();
        Action onePlusThreeSpecimen14 = myBot.getDrive().actionBuilder(new Pose2d(-4, -46, Math.toRadians(90)))
                // Park
                .strafeToLinearHeading(new Vector2d(34,-62), Math.toRadians(90))
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
                onePlusThreeSpecimen1,
                onePlusThreeSpecimen2,
                onePlusThreeSpecimen3,
                onePlusThreeSpecimen4,
                onePlusThreeSpecimen5,
                onePlusThreeSpecimen6,
                onePlusThreeSpecimen7,
                onePlusThreeSpecimen8,
                onePlusThreeSpecimen9,
                onePlusThreeSpecimen10,
                onePlusThreeSpecimen11,
                onePlusThreeSpecimen12,
                onePlusThreeSpecimen13,
                onePlusThreeSpecimen14
        ));
    }
}