package org.firstinspires.ftc.teamcode.base.autonomous;

import androidx.annotation.NonNull;

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

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PinpointDrive;

@Config
@Autonomous(name = "TristanAuto", group = "Autonomous")
public class TristanBackupAuto extends LinearOpMode {
    double kP = 0.015;
    DcMotor extendo;
    DcMotor extendoPitch;
    DcMotor hang;
    DcMotor bucketSlides;
    Servo clawPitchLeft;
    Servo clawPitchRight;
    Servo innerClawPitch;
    Servo clawFingers;
    Servo clawWrist;
    Servo bucket;
    public double extendoTarget = 0;
    public double extendoPitchTarget = 0;
    public double clawPitchPosition = 77.4;

    public double innerClawPitchPosition = 170.5;
    public double clawFingerPosition = 92;
    public double clawWristPosition = 95;
    public double bucketSlidesTarget = 0;
    public double bucketPosition = 36;
    public double hangTarget = 0;
    public class MotionProfile{
        public double kP; public double kI; public double kD;
        public double MAX_ACCELERATION; public double MAX_VELOCITY;
        public double currentMaxAcceleration = 0; public double currentMaxDeceleration = 0; public double currentMaxVelocity = 0;
        public double accelDT = 0; public double decelDT = 0; public double cruiseDT = 0;
        public double accelDistance = 0; public double decelDistance = 0; public double cruiseDistance = 0;
        public double profileStartPos = 0;
        public double startVelocity = 0;
        public ElapsedTime MOVEMENT_TIMER = new ElapsedTime(); public ElapsedTime LOOP_TIMER = new ElapsedTime();
        public DcMotorEx[] motor;
        public double target = 0;
        public double instantTargetPosition = 0;
        public double integralSum;
        public double previousError;
        public MotionProfile(double kP, double kI, double kD,double maxVelocity, double maxAcceleration, DcMotorEx[] motor){
            this.kP=kP;this.kD=kD;this.kI=kI;
            this.MAX_VELOCITY=maxVelocity;
            this.MAX_ACCELERATION=maxAcceleration;
            this.motor=motor;
        }
        public void createMotionProfile(double max_velocity, double max_acceleration) {
            profileStartPos=motor[0].getCurrentPosition();
            double distance=target-profileStartPos;
            if (distance!=0) {
                startVelocity = motor[0].getVelocity();
                currentMaxVelocity = max_velocity * Math.signum(distance);
                currentMaxAcceleration = max_acceleration * Math.signum(currentMaxVelocity - startVelocity);
                currentMaxDeceleration = -max_acceleration * Math.signum(distance);

                accelDT = (currentMaxVelocity - startVelocity) / currentMaxAcceleration;
                decelDT = (0 - currentMaxVelocity) / currentMaxDeceleration;
                accelDistance = startVelocity * accelDT + 0.5 * currentMaxAcceleration * Math.pow(accelDT, 2);
                decelDistance = currentMaxVelocity * decelDT + 0.5 * currentMaxDeceleration * Math.pow(decelDT, 2);

                if (Math.abs(accelDistance + decelDistance) > Math.abs(distance)) {
                    double halfExceededDistance = (distance - accelDistance - decelDistance) / 2;
                    accelDistance = accelDistance + halfExceededDistance;
                    accelDT = Math.max(
                            (-startVelocity + Math.sqrt(Math.abs(Math.pow(startVelocity, 2) + 2 * currentMaxAcceleration * accelDistance))) / (currentMaxAcceleration),
                            (-startVelocity - Math.sqrt(Math.abs(Math.pow(startVelocity, 2) + 2 * currentMaxAcceleration * accelDistance))) / (currentMaxAcceleration)
                    );
                    currentMaxVelocity = currentMaxAcceleration * accelDT + startVelocity;
                    decelDistance = decelDistance + halfExceededDistance;
                    decelDT = Math.max(
                            (-currentMaxVelocity + Math.sqrt(Math.abs(Math.pow(currentMaxVelocity, 2) + 2 * currentMaxDeceleration * decelDistance))) / (currentMaxDeceleration),
                            (-currentMaxVelocity - Math.sqrt(Math.abs(Math.pow(currentMaxVelocity, 2) + 2 * currentMaxDeceleration * decelDistance))) / (currentMaxDeceleration)
                    );
                }
                cruiseDistance = distance - accelDistance - decelDistance;
                cruiseDT = cruiseDistance / currentMaxVelocity;
            }
            else{
                accelDT=0;
                cruiseDT=0;
                decelDT=0;
                accelDistance=0;
                cruiseDistance=0;
                decelDistance=0;
            }
            MOVEMENT_TIMER.reset();
        }
        public void runMotionProfileOnce(){
            double elapsedTime = MOVEMENT_TIMER.time();
            if (elapsedTime > accelDT+decelDT+cruiseDT){
                instantTargetPosition=target;
            }

            else if (elapsedTime < accelDT){
                instantTargetPosition=profileStartPos + startVelocity * elapsedTime + 0.5 * currentMaxAcceleration * Math.pow(elapsedTime, 2);
            }
            else if (elapsedTime < accelDT+cruiseDT){
                double cruiseCurrentDT = elapsedTime - accelDT;
                instantTargetPosition=profileStartPos + accelDistance + currentMaxVelocity * cruiseCurrentDT;
            }

            else if (elapsedTime < accelDT+cruiseDT+decelDT){
                double decelCurrentDT = elapsedTime - accelDT - cruiseDT;
                instantTargetPosition = profileStartPos + accelDistance + cruiseDistance + currentMaxVelocity * decelCurrentDT + 0.5 * currentMaxDeceleration * Math.pow(decelCurrentDT, 2);
            }
            double error=instantTargetPosition-motor[0].getCurrentPosition();
            double kpPower = kP*error;
            integralSum += LOOP_TIMER.time()*error;
            double kiPower = kI*integralSum;
            double kdPower = kD*(error-previousError)/LOOP_TIMER.time();
            motor[0].setPower(Math.min(1,Math.max(-1,kpPower+kiPower+kdPower)));
            previousError=error;
            LOOP_TIMER.reset();
        }
        public void createAndRunProfileOnce(double target){
            if (target!=this.target) {
                this.target = target;
                integralSum=0;
                previousError = 0;
                createMotionProfile(MAX_VELOCITY,MAX_ACCELERATION);
                LOOP_TIMER.reset();
            }
            runMotionProfileOnce();
        }
    }

    public class GlobalPID implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            extendo.setPower((extendoTarget - extendo.getCurrentPosition()) * kP);
            extendoPitch.setPower((extendoPitchTarget - extendoPitch.getCurrentPosition()) * 0.005);
            //hang.setPower((hangTarget - hang.getCurrentPosition()) * kP);
            bucketSlides.setPower((bucketSlidesTarget - bucketSlides.getCurrentPosition()) * kP);

            clawPitchLeft.setPosition(clawPitchPosition/270);
            clawPitchRight.setPosition(clawPitchPosition/270);
            innerClawPitch.setPosition(innerClawPitchPosition/270);
            clawFingers.setPosition(clawFingerPosition/180);
            clawWrist.setPosition(clawWristPosition/270);
            bucket.setPosition(bucketPosition/270);
            return true;
        }
    }

    public Action GlobalPID() {
        return new GlobalPID();
    }

    public class SetExtendoTarget implements Action {
        private double target;

        public SetExtendoTarget(double target) {
            this.target = target;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            extendoTarget = target;
            return false;
        }
    }

    public Action setExtendoTarget(double target) {
        return new SetExtendoTarget(target);
    }

    public class SetExtendoPitchTarget implements Action {
        private double target;

        public SetExtendoPitchTarget(double target) {
            this.target = target;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            extendoPitchTarget = target;
            return false;
        }
    }

    public Action setExtendoPitchTarget(double target) {
        return new SetExtendoPitchTarget(target);
    }

    public class SetBucketSlidesTarget implements Action {
        private double target;

        public SetBucketSlidesTarget(double target) {
            this.target = target;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            bucketSlidesTarget = target;
            return false;
        }
    }

    public Action setBucketSlidesTarget(double target) {
        return new SetBucketSlidesTarget(target);
    }

    public class SetHangTarget implements Action {
        private double target;

        public SetHangTarget(double target) {
            this.target = target;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            hangTarget = target;
            return false;
        }
    }

    public Action setHangTarget(double target) {
        return new SetHangTarget(target);
    }

    public class SetClawPitchPosition implements Action {
        private double position;

        public SetClawPitchPosition(double position) {
            this.position = position;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            clawPitchPosition = position;
            return false;
        }
    }

    public Action setClawPitchPosition(double position) {
        return new SetClawPitchPosition(position);
    }

    public class SetClawFingerPosition implements Action {
        private double position;

        public SetClawFingerPosition(double position) {
            this.position = position;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            clawFingerPosition = position;
            return false;
        }
    }

    public Action setClawFingerPosition(double position) {
        return new SetClawFingerPosition(position);
    }

    public class SetClawWristPosition implements Action {
        private double position;

        public SetClawWristPosition(double position) {
            this.position = position;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            clawWristPosition = position;
            return false;
        }
    }

    public Action setInnerClawPitchPosition(double position) {
        return new setInnerClawPitchPosition(position);
    }

    public class setInnerClawPitchPosition implements Action {
        private double position;

        public setInnerClawPitchPosition(double position) {
            this.position = position;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            innerClawPitchPosition = position;
            return false;
        }
    }

    public Action setClawWristPosition(double position) {
        return new SetClawWristPosition(position);
    }

    public class SetBucketPosition implements Action {
        private double position;

        public SetBucketPosition(double position) {
            this.position = position;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            bucketPosition = position;
            return false;
        }
    }

    public Action setBucketPosition(double position) {
        return new SetBucketPosition(position);
    }

    @Override
    public void runOpMode() {


        Pose2d initialPose = new Pose2d(-41, -62.5, Math.toRadians(90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);

        Action onePlusThreeBucket1 = drive.actionBuilder(new Pose2d(-41,-62.5, Math.toRadians(90)))
                // Score preload bucket
                .strafeToLinearHeading(new Vector2d(-62,-53), Math.toRadians(45))
                .build();
        Action onePlusThreeBucket2 = drive.actionBuilder(new Pose2d(-62,-53, Math.toRadians(45)))
                // Go to ob zone
                .strafeToLinearHeading(new Vector2d(5,-64), Math.toRadians(3))
                .build();
        Action onePlusThreeBucket3 = drive.actionBuilder(new Pose2d(5,-64, Math.toRadians(3)))
                // Score bucket
                .strafeToLinearHeading(new Vector2d(-60,-52), Math.toRadians(45))
                .build();
        Action onePlusThreeBucket4 = drive.actionBuilder(new Pose2d(-60,-52, Math.toRadians(45)))
                // Sample zone 1
                .strafeToLinearHeading(new Vector2d(-48,-54), Math.toRadians(90))
                .build();
        Action onePlusThreeBucket5 = drive.actionBuilder(new Pose2d(-48,-54, Math.toRadians(90)))
                // Score bucket
                .strafeToLinearHeading(new Vector2d(-58,-54), Math.toRadians(45))
                .build();
        Action onePlusThreeBucket6 = drive.actionBuilder(new Pose2d(-58,-54, Math.toRadians(45)))
                // sample zone 2
                .strafeToLinearHeading(new Vector2d(-64.5,-54), Math.toRadians(93))
                .build();
        Action onePlusThreeBucket7 = drive.actionBuilder(new Pose2d(-64.5,-54, Math.toRadians(93)))
                // turn and score bucket
                .strafeToLinearHeading(new Vector2d(-58,-55), Math.toRadians(45))
                .build();
        Action onePlusThreeBucket8 = drive.actionBuilder(new Pose2d(-58,-55, Math.toRadians(45)))
                // sample 3
                .strafeToLinearHeading(new Vector2d(-64.5,-54), Math.toRadians(108))
                .build();
        Action onePlusThreeBucket9 = drive.actionBuilder(new Pose2d(-64.5,-54, Math.toRadians(108                     )))
                // obs zone
                .strafeToLinearHeading(new Vector2d(-59,-55), Math.toRadians(45))
                .build();

        extendo = hardwareMap.dcMotor.get("extendo");
        extendoPitch = hardwareMap.dcMotor.get("extendoPitch");
        bucketSlides = hardwareMap.dcMotor.get("bucketSlides");
        clawPitchLeft = hardwareMap.servo.get("clawPitchLeft");
        clawPitchRight = hardwareMap.servo.get("clawPitchRight");
        innerClawPitch = hardwareMap.servo.get("innerClawPitch");

        clawFingers = hardwareMap.servo.get("clawFingers");
        clawWrist = hardwareMap.servo.get("clawWrist");
        bucket = hardwareMap.servo.get("bucket");

        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendo.setDirection(DcMotorSimple.Direction.REVERSE);

        extendoPitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendoPitch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendoPitch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bucketSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bucketSlides.setDirection(DcMotor.Direction.REVERSE);
        bucketSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawPitchRight.setDirection(Servo.Direction.REVERSE);
        innerClawPitch.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        GlobalPID(),
                        new SequentialAction(
                                new ParallelAction(
                                        // score preload bucket
                                        onePlusThreeBucket1,
                                        //close claw
                                        new SequentialAction(
                                                new SleepAction(0.5),
                                                setBucketSlidesTarget(1070)
                                        )

                                ),
                                // Move bucketSlides up to scoring position
                                new ParallelAction(
                                        new SequentialAction(

                                                //setBucketSlidesTarget(0),
                                                setInnerClawPitchPosition(5),
                                                setClawPitchPosition(68),
                                                // Extendo pitch pickup position
                                                setExtendoPitchTarget(-960)
                                        ),
                                        new SequentialAction(
                                            new SleepAction(0.8),
                                            setBucketPosition(158),
                                            new SleepAction(0.7),
                                            setBucketPosition(36),
                                            new SleepAction(0.4)
                                        )
                                ),



                                // Drive to sample zone 1, lower extendo, retract extendo pitch when driving
                                new ParallelAction(
                                        // Drive to sample zone 1
                                        setBucketSlidesTarget(0),
                                        onePlusThreeBucket2,
                                        setExtendoTarget(780)
                                ),

                                // Claw pitch picking up pos
                                new SleepAction(0.2),

                                new ParallelAction(
                                        setInnerClawPitchPosition(78),
                                        setClawPitchPosition(13)
                                ),
                                // Close Claw
                                new SleepAction(0.25),

                                setClawFingerPosition(0),
                                new SleepAction(0.3),

                                new ParallelAction(
                                        setClawWristPosition(95),
                                        setInnerClawPitchPosition(200),
                                        setExtendoTarget(0),
                                        setClawPitchPosition(110)
                                ),

                                // Retract extendo, transfer and move to scoring pos
                                new ParallelAction(
                                        // Move to scoring position
                                        onePlusThreeBucket3,
                                        new SequentialAction(
                                                // Retract extendo
                                                // Claw pitch transfer position
                                                new SleepAction(0.4),
                                                // Extendo pitch transfer position
                                                setExtendoPitchTarget(0),
                                                new SleepAction(0.7),
                                                setClawFingerPosition(86),
                                                new SleepAction(0.4),
                                                setClawPitchPosition(72.4),
                                                setInnerClawPitchPosition(160)
                                        )
                                ),
                                // Wait for sample to settle in bucket
                                new SleepAction(0.4),
                                // Move bucketSlides up to scoring position
                                setBucketSlidesTarget(1070),
                                new ParallelAction(
                                        new SequentialAction(

                                                //setBucketSlidesTarget(0),
                                                setInnerClawPitchPosition(5),
                                                setClawPitchPosition(68),
                                                // Extendo pitch pickup position
                                                setExtendoPitchTarget(-960)
                                        ),
                                        new SequentialAction(
                                                new SleepAction(0.8),
                                                setBucketPosition(158),
                                                new SleepAction(0.7),
                                                setBucketPosition(36),
                                                new SleepAction(0.4)
                                        )
                                ),
                                new ParallelAction(
                                        // Drive to sample zone 2
                                        setBucketSlidesTarget(0),
                                        onePlusThreeBucket4,
                                        setExtendoTarget(780)
                                ),
                                // Claw pitch picking up pos
                                new SleepAction(0.2),

                                new ParallelAction(
                                        setInnerClawPitchPosition(78),
                                        setClawPitchPosition(13)
                                ),
                                // Close Claw
                                new SleepAction(0.25),

                                setClawFingerPosition(0),
                                new SleepAction(0.3),

                                new ParallelAction(
                                        setClawWristPosition(95),
                                        setInnerClawPitchPosition(200),
                                        setExtendoTarget(0),
                                        setClawPitchPosition(110)
                                ),

                                // Retract extendo, transfer and move to scoring pos
                                new ParallelAction(
                                        // Move to scoring position
                                        onePlusThreeBucket5,
                                        new SequentialAction(
                                                // Retract extendo
                                                // Claw pitch transfer position
                                                new SleepAction(0.4),
                                                // Extendo pitch transfer position
                                                setExtendoPitchTarget(0),
                                                new SleepAction(0.7),
                                                setClawFingerPosition(86),
                                                new SleepAction(0.4),
                                                setClawPitchPosition(72.4),
                                                setInnerClawPitchPosition(160)
                                        )
                                ),
                                // Wait for sample to settle in bucket
                                new SleepAction(0.4),
                                // Move bucketSlides up to scoring position
                                setBucketSlidesTarget(1070),
                                new ParallelAction(
                                        new SequentialAction(

                                                //setBucketSlidesTarget(0),
                                                setInnerClawPitchPosition(5),
                                                setClawPitchPosition(68),
                                                // Extendo pitch pickup position
                                                setExtendoPitchTarget(-960)
                                        ),
                                        new SequentialAction(
                                                new SleepAction(0.8),
                                                setBucketPosition(158),
                                                new SleepAction(0.7),
                                                setBucketPosition(36),
                                                new SleepAction(0.4)
                                        )
                                ),
                                new ParallelAction(
                                        // Drive to sample zone 3
                                        setBucketSlidesTarget(0),
                                        onePlusThreeBucket6,
                                        setExtendoTarget(780)
                                ),

                                // Claw pitch picking up pos
                                new SleepAction(0.2),

                                new ParallelAction(
                                        setInnerClawPitchPosition(78),
                                        setClawPitchPosition(13)
                                ),
                                // Close Claw
                                new SleepAction(0.25),

                                setClawFingerPosition(0),
                                new SleepAction(0.3),

                                new ParallelAction(
                                        setClawWristPosition(95),
                                        setInnerClawPitchPosition(200),
                                        setExtendoTarget(0),
                                        setClawPitchPosition(110)
                                ),

                                // Retract extendo, transfer and move to scoring pos
                                new ParallelAction(
                                        // Move to scoring position
                                        onePlusThreeBucket7,
                                        new SequentialAction(
                                                // Retract extendo
                                                // Claw pitch transfer position
                                                new SleepAction(0.4),
                                                // Extendo pitch transfer position
                                                setExtendoPitchTarget(0),
                                                new SleepAction(0.7),
                                                setClawFingerPosition(86),
                                                new SleepAction(0.4),
                                                setClawPitchPosition(72.4),
                                                setInnerClawPitchPosition(160)
                                        )
                                ),
                                // Wait for sample to settle in bucket
                                new SleepAction(0.4),
                                // Move bucketSlides up to scoring position
                                setBucketSlidesTarget(1070),
                                new ParallelAction(
                                        new SequentialAction(

                                                //setBucketSlidesTarget(0),
                                                setInnerClawPitchPosition(5),
                                                setClawPitchPosition(68),
                                                // Extendo pitch pickup position
                                                setExtendoPitchTarget(-960)
                                        ),
                                        new SequentialAction(
                                                new SleepAction(0.8),
                                                setBucketPosition(158),
                                                new SleepAction(0.7),
                                                setBucketPosition(36),
                                                new SleepAction(0.4)
                                        )
                                ),
                                new ParallelAction(
                                        // Drive to sample zone 2
                                        setBucketSlidesTarget(0),
                                        onePlusThreeBucket8,
                                        setExtendoTarget(780)
                                ),
                                // Claw pitch picking up pos
                                new SleepAction(0.2),

                                new ParallelAction(
                                        setInnerClawPitchPosition(78),
                                        setClawPitchPosition(13)
                                ),
                                // Close Claw
                                new SleepAction(0.25),

                                setClawFingerPosition(0),
                                new SleepAction(0.3),

                                new ParallelAction(
                                        setClawWristPosition(95),
                                        setInnerClawPitchPosition(200),
                                        setExtendoTarget(0),
                                        setClawPitchPosition(110)
                                ),

                                // Retract extendo, transfer and move to scoring pos
                                new ParallelAction(
                                        // Move to scoring position
                                        onePlusThreeBucket9,
                                        new SequentialAction(
                                                // Retract extendo
                                                // Claw pitch transfer position
                                                new SleepAction(0.4),
                                                // Extendo pitch transfer position
                                                setExtendoPitchTarget(0),
                                                new SleepAction(0.7),
                                                setClawFingerPosition(86),
                                                new SleepAction(0.4),
                                                setClawPitchPosition(72.4),
                                                setInnerClawPitchPosition(160)
                                        )
                                ),
                                // Wait for sample to settle in bucket
                                new SleepAction(0.4),
                                // Move bucketSlides up to scoring position
                                setBucketSlidesTarget(1070),
                                new  SleepAction(0.8),
                                setBucketPosition(158),
                                new SleepAction(0.7),
                                setBucketPosition(36),
                                new SleepAction(0.4),
                                setBucketSlidesTarget(0)

                    )
                )
        );
    }
}