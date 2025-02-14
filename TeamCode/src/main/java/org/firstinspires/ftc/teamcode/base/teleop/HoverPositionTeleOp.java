package org.firstinspires.ftc.teamcode.base.teleop;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.isRRActive;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.bucket;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.bucketSlides;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.clawFingers;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.clawPitch;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.clawWrist;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.extendo;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.extendoPitch;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.hang;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.innerClawPitch;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.BotMotor;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.leftBack;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.leftFront;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.rightBack;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.rightFront;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.UpdateTelemetryAction;
import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.TeleOpSequentialAction;
import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.TeleOpParallelAction;
import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.PressTrigger;
import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.ConditionalAction;
import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.RobotCentricMecanumAction;
import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.TeleOpAction;
import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.TeleOpSleepAction;
import org.firstinspires.ftc.teamcode.base.teleop.LambdaInterfaces.Condition;

@TeleOp
public class HoverPositionTeleOp extends LinearOpMode {
    public boolean isBucketSlidesMaxLowered = false;
    @Override
    public void runOpMode() throws InterruptedException {
        TeleOpComponents.initializeMechanisms(hardwareMap,telemetry,new Pose2d(0,0,Math.toRadians(90)));
        waitForStart();
        if (gamepad1.options){
            bucketSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extendoPitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bucketSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            extendoPitch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        clawFingers.setPosition(clawFingers.getPos("openPosition"));
        clawWrist.setPosition(clawWrist.getPos("normalPosition"));
        clawPitch.setPosition(clawPitch.getPos("hoverPosition"));
        innerClawPitch.setPosition(innerClawPitch.getPos("hoverPosition"));
        bucket.setPosition(bucket.getPos("transferPosition"));
        PressTrigger sequences = new PressTrigger(new Condition[]{
                () -> (gamepad1.b),
                () -> (gamepad2.x),
                () -> (gamepad1.a),
                () -> (gamepad2.options),
                ()->(gamepad2.b),
                ()->(gamepad2.y),
                ()->(gamepad2.dpad_up),
                ()->(gamepad2.dpad_down)
        }, new TeleOpAction[]{
                new TeleOpParallelAction(
                        bucketSlides.setTargetAction(bucketSlides.getPos("transferPosition")),
                        new TeleOpSequentialAction(
                                new TeleOpParallelAction(
                                        new TeleOpActions.ShortAction(()->{extendoPitch.setMovementMode("PID");}),
                                        extendoPitch.setTargetAction(extendoPitch.getPos("pickUpPosition")),
                                        clawFingers.setPositionAction(clawFingers.getPos("openPosition")),
                                        clawPitch.setPositionAction(clawPitch.getPos("hoverPosition")),
                                        innerClawPitch.setPositionAction(innerClawPitch.getPos("hoverPosition"))
                                ),
                                new TeleOpParallelAction(
                                        extendo.setTargetAction(428),
                                        clawFingers.setPositionAction(clawFingers.getPos("openPosition"))
                                )
                        )
                ),
                new TeleOpSequentialAction(
                        clawFingers.setPositionAction(clawFingers.getPos("openPosition")),
                        new TeleOpParallelAction(
                                innerClawPitch.setPositionAction(innerClawPitch.getPos("pickUpPosition")),
                                clawPitch.setPositionAction(clawPitch.getPos("pickUpPosition"))
                        ),
                        clawFingers.setPositionAction(clawFingers.getPos("closedPosition")),
                        new TeleOpParallelAction(
                                innerClawPitch.setPositionAction(innerClawPitch.getPos("hoverPosition")),
                                clawPitch.setPositionAction(clawPitch.getPos("hoverPosition"))
                        )
                ),
                new TeleOpSequentialAction(
                        new TeleOpParallelAction(
                                bucketSlides.setTargetAction(bucketSlides.getPos("transferPosition")),
                                bucket.setPositionAction(bucket.getPos("transferPosition")),
                                clawWrist.setPositionAction(clawWrist.getPos("normalPosition")),
                                extendo.setTargetAction(extendo.MIN_POSITION),
                                clawPitch.setPositionAction(clawPitch.getPos("transferPosition")),
                                innerClawPitch.setPositionAction(innerClawPitch.getPos("transferPosition")),
                                new TeleOpSequentialAction(
                                        new TeleOpActions.SleepWhileTrue(()->(extendo.instantTargetPosition<160)),
                                        new TeleOpParallelAction(
                                                new TeleOpActions.ShortAction(()->{extendoPitch.setMovementMode("MOTION_PROFILE");}),
                                                extendoPitch.setTargetAction(extendoPitch.getPos("transferPosition"))
                                        )
                                )
                        ),
                        clawFingers.setPositionAction(clawFingers.getPos("openPosition")),
                        new TeleOpSleepAction(0.1),
                        new TeleOpParallelAction(
                                clawPitch.setPositionAction(clawPitch.getPos("backOffPosition")),
                                innerClawPitch.setPositionAction(innerClawPitch.getPos("backOffPosition"))
                        ),
                        bucketSlides.setTargetAction(()->{if (!isBucketSlidesMaxLowered) return bucketSlides.getPos("depositPosition"); else return bucketSlides.getPos("lowDepositPosition");})
                ),
                new TeleOpParallelAction(
                        bucket.setPositionAction(bucket.getPos("depositPosition")),
                        new TeleOpActions.ShortAction(()->{extendoPitch.setMovementMode("PID");}),
                        extendoPitch.setTargetAction(extendoPitch.getPos("specimenGrabPosition")),
                        clawWrist.setPositionAction(clawWrist.getPos("normalPosition")),
                        extendo.setTargetAction(extendo.MIN_POSITION),
                        clawPitch.setPositionAction(clawPitch.getPos("specimenGrabPosition")),
                        innerClawPitch.setPositionAction(innerClawPitch.getPos("specimenGrabPosition"))
                ),
                new TeleOpSequentialAction(
                        new TeleOpParallelAction(
                                clawFingers.setPositionAction(clawFingers.getPos("closedPosition")),
                                innerClawPitch.setPositionAction(innerClawPitch.getPos("transferPosition")),
                                bucket.setPositionAction(bucket.getPos("depositPosition")),
                                bucketSlides.setTargetAction(200)
                        ),
                        new TeleOpActions.ShortAction(()->{extendoPitch.setMovementMode("MOTION_PROFILE");}),
                        new TeleOpParallelAction(
                                extendoPitch.setTargetAction(extendoPitch.getPos("specimenDepositPosition")),
                                new TeleOpSequentialAction(
                                        new TeleOpActions.SleepWhileTrue(()->(extendoPitch.instantTargetPosition<-800)),
                                        new TeleOpParallelAction(
                                                clawPitch.setPositionAction(clawPitch.getPos("specimenDepositPosition")),
                                                innerClawPitch.setPositionAction(innerClawPitch.getPos("specimenDepositPosition"))
                                        )
                                ),
                                new TeleOpSequentialAction(
                                        new TeleOpActions.SleepWhileTrue(()->(extendoPitch.instantTargetPosition<-100)),
                                        extendo.setTargetAction(extendo.MAX_POSITION)
                                )
                        )
                ),
                new TeleOpSequentialAction(
                        extendo.setTargetAction(250),
                        clawFingers.setPositionAction(clawFingers.getPos("openPosition"))
                ),
                new TeleOpParallelAction(
                        extendo.setTargetAction(428),
                        clawPitch.setPositionAction(clawPitch.getPos("hoverPosition")),
                        innerClawPitch.setPositionAction(innerClawPitch.getPos("hoverPosition")),
                        bucket.setPositionAction(bucket.getPos("transferPosition"))
                ),
                new TeleOpSequentialAction(
                        new TeleOpParallelAction(
                                clawWrist.setPositionAction(clawWrist.getPos("normalPosition")),
                                clawPitch.setPositionAction(clawPitch.getPos("transferPosition")),
                                innerClawPitch.setPositionAction(innerClawPitch.getPos("transferPosition")),
                                bucket.setPositionAction(bucket.getPos("transferPosition"))
                        ),
                        extendo.setTargetAction(extendo.MIN_POSITION)
                ),
        });
        TeleOpActions.runLoop(
                this::opModeIsActive,
                new PressTrigger(new Condition[]{()->(gamepad1.dpad_down)}, new TeleOpAction[]{
                        new TeleOpParallelAction(
                                hang.setPowerAction(()->{if (hang.getPower()!=0) return 0; else return 1;}),
                                clawPitch.setPositionAction(clawPitch.getPos("pickUpPosition")),
                                innerClawPitch.setPositionAction(innerClawPitch.getPos("pickUpPosition"))
                        )
                }),
                new PressTrigger(new Condition[]{()->(gamepad1.dpad_up)}, new TeleOpAction[]{hang.setPowerAction(()->{if (hang.getPower()!=0) return 0; else return -1;})}),
                clawFingers.triggeredToggleAction(()->(gamepad2.left_bumper||gamepad2.right_bumper),clawFingers.getPos("openPosition"),clawFingers.getPos("closedPosition")),
                new PressTrigger(new Condition[]{()->(gamepad2.back)},new TeleOpAction[]{extendoPitch.stallResetAction(-1020)}),
                new PressTrigger(new Condition[]{()->(gamepad1.back)},new TeleOpAction[]{new TeleOpActions.ShortAction(()->{bucketSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);bucketSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);bucketSlides.offset=1065;})}),
                bucketSlides.triggeredDynamicAction(()->(gamepad1.dpad_left),()->(gamepad1.dpad_right),15),
                clawWrist.triggeredDynamicAction(()->(gamepad2.left_trigger>0),()->(gamepad2.right_trigger>0),8),
                bucket.triggeredToggleAction(()->(gamepad2.a),bucket.getPos("transferPosition"),bucket.getPos("depositPosition")),
                new ConditionalAction(
                        new Condition[]{()->(!isBucketSlidesMaxLowered),()->(isBucketSlidesMaxLowered)},
                        new TeleOpAction[]{
                                bucketSlides.triggeredToggleAction(()->(gamepad1.y),bucketSlides.getPos("depositPosition"),bucketSlides.getPos("transferPosition"),new double[]{},new double[]{200}),
                                bucketSlides.triggeredToggleAction(()->(gamepad1.y),bucketSlides.getPos("lowDepositPosition"),bucketSlides.getPos("transferPosition"),new double[]{},new double[]{bucketSlides.getPos("depositPosition"),200}),
                        }
                ),
                new PressTrigger(new Condition[]{()->(gamepad2.left_stick_x<0 && gamepad2.right_stick_x>0)},new TeleOpAction[]{new TeleOpActions.ShortAction(()->{isBucketSlidesMaxLowered=true;})}),
                sequences,
                new ConditionalAction(new Condition[]{()->(!isRRActive)}, new TeleOpAction[]{
                        new RobotCentricMecanumAction(new BotMotor[]{leftFront,leftBack,rightFront,rightBack},()->(gamepad1.left_stick_x),()->(gamepad1.left_stick_y),()->(gamepad1.right_stick_x),()->(gamepad1.left_trigger>0.2))
                }),
                extendo.triggeredDynamicAction(()->(gamepad1.right_bumper),()->(gamepad1.left_bumper),15),
                new PressTrigger(new Condition[]{()->(gamepad1.options)}, new TeleOpAction[]{new TeleOpActions.ShortAction(()->{
                    extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                })}),
                new TeleOpActions.ShortAction(()->{clawWrist.setPosition(clawWrist.getPosition());}),
                new UpdateTelemetryAction()
        );
    }
}
