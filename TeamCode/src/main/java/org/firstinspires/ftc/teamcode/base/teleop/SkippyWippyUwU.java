package org.firstinspires.ftc.teamcode.base.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class SkippyWippyUwU extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo clawPitchLeft = hardwareMap.servo.get("clawPitchLeft");
        Servo clawPitchRight = hardwareMap.servo.get("clawPitchRight");
        Servo innerClawPitch = hardwareMap.servo.get("innerClawPitch");

        Servo clawFingers = hardwareMap.servo.get("clawFingers");
        Servo clawWrist = hardwareMap.servo.get("clawWrist");
        Servo bucket = hardwareMap.servo.get("bucket");
        clawPitchRight.setDirection(Servo.Direction.REVERSE);
        innerClawPitch.setDirection(Servo.Direction.REVERSE);
        waitForStart();
        clawWrist.setPosition((double) 95/270);
        clawFingers.setPosition((double) 92/180);
        clawPitchLeft.setPosition((double) 68/270);
        clawPitchRight.setPosition((double) 68/270);
        innerClawPitch.setPosition((double) 20/270);
        bucket.setPosition((double) 36/270);
        while (opModeIsActive()) {
            if (gamepad1.right_bumper){
                clawFingers.setPosition(clawFingers.getPosition()+((double) 1 /180));
            }
            else if (gamepad1.left_bumper){
                clawFingers.setPosition(clawFingers.getPosition()-((double) 1 /180));
            }
            telemetry.addData("pos",clawFingers.getPosition()*180);
            telemetry.update();
        }
    }
}
