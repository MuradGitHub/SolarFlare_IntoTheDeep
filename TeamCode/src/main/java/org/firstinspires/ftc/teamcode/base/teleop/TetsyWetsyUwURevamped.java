package org.firstinspires.ftc.teamcode.base.teleop;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.bucket;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.bucketSlides;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.clawFingers;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.clawPitch;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.clawWrist;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.extendo;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.extendoPitch;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.innerClawPitch;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.UpdateTelemetryAction;

@TeleOp
public class TetsyWetsyUwURevamped extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        TeleOpComponents.initializeMechanisms(hardwareMap,telemetry,new Pose2d(0,0,Math.toRadians(90)));
        waitForStart();
        /*
        bucket.setPosition(0);
        clawFingers.setPosition(92);
        clawPitch.setPosition(73);
        clawWrist.setPosition(95);
        innerClawPitch.setPosition(0);
        */
        extendo.setMotorDisable();
        extendoPitch.setMotorDisable();
        bucketSlides.setMotorDisable();
            TeleOpActions.runLoop(
                    this::opModeIsActive,
                    bucket.triggeredDynamicAction(()->(gamepad1.left_bumper),()->(gamepad1.right_bumper),0.25),
                    clawPitch.triggeredDynamicAction(()->(gamepad1.left_trigger>0),()->(gamepad1.right_trigger>0),0.25),
                    clawFingers.triggeredDynamicAction(()->(gamepad1.dpad_left),()->(gamepad1.dpad_right),0.25),
                    clawWrist.triggeredDynamicAction(()->(gamepad1.dpad_up),()->(gamepad1.dpad_down),0.25),
                    innerClawPitch.triggeredDynamicAction(()->(gamepad1.right_stick_y>0),()->(gamepad1.right_stick_y<0),0.25),
                    new UpdateTelemetryAction()
            );
    }
}
