package org.firstinspires.ftc.teamcode.base.teleop;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.bucketSlides;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.clawFingers;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.clawPitch;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.clawPitchRight;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.extendo;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.extendoPitch;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.innerClawPitch;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.ryanNemesis;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.UpdateTelemetryAction;
import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.TeleOpSequentialAction;
import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.TeleOpParallelAction;
import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.UninterruptiblePressTrigger;
import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.TeleOpAction;
import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.TeleOpSleepAction;
import org.firstinspires.ftc.teamcode.base.teleop.LambdaInterfaces.Condition;

@TeleOp
public class JustTetsing extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        TeleOpComponents.initializeMechanisms(hardwareMap,telemetry,new Pose2d(0,0,Math.toRadians(90)));
        waitForStart();
            TeleOpActions.runLoop(
                    this::opModeIsActive,
                    bucketSlides.triggeredToggleAction(()->(gamepad1.a),0,1055),
                    extendo.triggeredToggleAction(()->(gamepad1.b),0,600),
                    new TeleOpActions.PressTrigger(new Condition[]{()->(gamepad1.x)},new TeleOpAction[]{
                            new TeleOpActions.SemiUninterruptibleConditionalAction(new Condition[]{()->(extendoPitch.target==0),()->(extendoPitch.target==-1020)},
                                    new TeleOpAction[]{
                                        new TeleOpSequentialAction(
                                            new TeleOpActions.ShortAction(()->{extendoPitch.setMovementMode("PID");}),
                                            extendoPitch.setTargetAction(-1020)
                                        ),
                                        new TeleOpSequentialAction(
                                                new TeleOpActions.ShortAction(()->{extendoPitch.setMovementMode("MOTION_PROFILE");}),
                                                extendoPitch.setTargetAction(0)
                                        ),
                                    }
                            )

                    }),
                    new UpdateTelemetryAction()
            );
    }
}
