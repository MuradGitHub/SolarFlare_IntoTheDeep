package org.firstinspires.ftc.teamcode.base.testing;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.base.autonomous.CustomActions;
import org.firstinspires.ftc.teamcode.base.config.HardwareConfig;
import org.firstinspires.ftc.teamcode.base.config.RobotConfig;
import org.firstinspires.ftc.teamcode.base.config.RobotState;

@Autonomous
public class TestPathing extends LinearOpMode {
    RobotConfig robotConfig;
    HardwareConfig hw;
    RobotState state;
    CustomActions actions;
    @Override
    public void runOpMode(){
        robotConfig = RobotConfig.makeInstance("IntoTheDeep-V2");

        try {
            hw = HardwareConfig.makeInstance(hardwareMap, robotConfig);
        }
        catch (Exception e){
            throw new RuntimeException(e);
        }

        state = RobotState.getInstance();

        actions = new CustomActions(telemetry);
        actions.setInitialDrivePosition("specimen", "sample");

        Action oneSpecimenPlusThreeBucket1 = actions.getDrive().actionBuilder(new Pose2d(-42,-62.5,Math.toRadians(270)))
                // Score preload
                .strafeToLinearHeading(new Vector2d(-3,-47), Math.toRadians(270))
                .build();
        Action oneSpecimenPlusThreeBucket2 = actions.getDrive().actionBuilder(new Pose2d(-3,-47, Math.toRadians(270)))
                // Go to sample zone 1
                .strafeToLinearHeading(new Vector2d(-55,-52.5), Math.toRadians(270))
                .build();
        Action oneSpecimenPlusThreeBucket3 = actions.getDrive().actionBuilder(new Pose2d(-55,-52.5, Math.toRadians(270)))
                // Score bucket
                .strafeToLinearHeading(new Vector2d(-55,-55), Math.toRadians(225))
                .build();
        Action oneSpecimenPlusThreeBucket4 = actions.getDrive().actionBuilder(new Pose2d(-55,-55, Math.toRadians(225)))
                // Sample zone 2
                .strafeToLinearHeading(new Vector2d(-63,-52), Math.toRadians(273))
                .build();
        Action oneSpecimenPlusThreeBucket5 = actions.getDrive().actionBuilder(new Pose2d(-63,-52, Math.toRadians(273)))
                // Score bucket
                .strafeToLinearHeading(new Vector2d(-54,-54), Math.toRadians(225))
                .build();
        Action oneSpecimenPlusThreeBucket6 = actions.getDrive().actionBuilder(new Pose2d(-54,-54, Math.toRadians(225)))
                // sample zone 3
                .strafeToLinearHeading(new Vector2d(-67,-52), Math.toRadians(285))
                .build();
        Action oneSpecimenPlusThreeBucket7 = actions.getDrive().actionBuilder(new Pose2d(-67,-52, Math.toRadians(285)))
                // turn and score bucket
                .strafeToLinearHeading(new Vector2d(-54,-54), Math.toRadians(225))
                .build();
        Action oneSpecimenPlusThreeBucket8 = actions.getDrive().actionBuilder(new Pose2d(-54,-54, Math.toRadians(225)))
                // park
                .strafeToLinearHeading(new Vector2d(-44,-6), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-24.2,-6), Math.toRadians(0))

                .build();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        oneSpecimenPlusThreeBucket1,
                        oneSpecimenPlusThreeBucket2,
                        oneSpecimenPlusThreeBucket3,
                        oneSpecimenPlusThreeBucket4,
                        oneSpecimenPlusThreeBucket5,
                        oneSpecimenPlusThreeBucket6,
                        oneSpecimenPlusThreeBucket7,
                        oneSpecimenPlusThreeBucket8
                        )
        );
    }
}
