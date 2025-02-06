
package org.firstinspires.ftc.teamcode.base.teleop;

import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class BackUpTeleOp extends LinearOpMode {
    double extendoPitchTarget = 0;
    double extendoTarget = 0;
    double bucketSlidesTarget = 0;
    double clawWristPosition = 95;
    double clawFingerPosition = 86;
    double clawPitchPosition = 100;
    double innerClawPitchPosition = 200;
    double bucketPosition = 36;

    double bucketSlidesMax = 1070;
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
    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();

        //double hangTarget = 0;
        double maxExtendoPosition = 793;

        boolean isXSequenceActive = false;
        boolean isASequenceActive = false;
        boolean isBSequenceActive = false;
        boolean isB2SequenceActive = false;
        boolean isOp2SequenceActive = false;
        boolean isPressingY = false;
        boolean isPressingY2 = false;
        boolean isBucketMaxSet = false;

        boolean isPressingA2 = false;
        boolean isX2SequenceActive = false;
        boolean isPressingBumper2 = false;
        boolean isPressingTrigger1 = false;
        boolean isPressingDpad = false;
        boolean isDownSequenceActive=false;


        double kP = 0.015;
        double kPpitch = 0.005;

        ElapsedTime Xtimer = new ElapsedTime();
        ElapsedTime Btimer = new ElapsedTime();
        ElapsedTime B2timer = new ElapsedTime();
        ElapsedTime X2timer = new ElapsedTime();
        ElapsedTime downTimer = new ElapsedTime();
        ElapsedTime Atimer = new ElapsedTime();
        ElapsedTime Op2timer = new ElapsedTime();

        DcMotor extendo = hardwareMap.dcMotor.get("extendo");
        DcMotor extendoPitch = hardwareMap.dcMotor.get("extendoPitch");
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");
        //DcMotor hang = hardwareMap.dcMotor.get("hang");
        DcMotor bucketSlides = hardwareMap.dcMotor.get("bucketSlides");

        Servo clawPitchLeft = hardwareMap.servo.get("clawPitchLeft");
        Servo clawPitchRight = hardwareMap.servo.get("clawPitchRight");
        Servo innerClawPitch = hardwareMap.servo.get("innerClawPitch");

        Servo clawFingers = hardwareMap.servo.get("clawFingers");
        Servo clawWrist = hardwareMap.servo.get("clawWrist");
        Servo bucket = hardwareMap.servo.get("bucket");

        IMU imu = hardwareMap.get(IMU.class, "imu");
        GoBildaPinpointDriverRR pinpoint = hardwareMap.get(GoBildaPinpointDriverRR.class, "pinpoint");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendo.setDirection(DcMotorSimple.Direction.REVERSE);

        extendoPitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendoPitch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendoPitch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        bucketSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bucketSlides.setDirection(DcMotor.Direction.REVERSE);
        bucketSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawPitchRight.setDirection(Servo.Direction.REVERSE);
        innerClawPitch.setDirection(Servo.Direction.REVERSE);


        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        pinpoint.recalibrateIMU();

        pinpoint.resetPosAndIMU();


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.back && bucketSlidesTarget>=15){
                bucketSlidesTarget-=15;
            }
            else if (gamepad1.back && bucketSlidesTarget<15){
                bucketSlidesTarget=0;
            }
            if ((gamepad2.back) && !(gamepad2.a)){
                bucketSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                extendoPitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                bucketSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                extendoPitch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            pinpoint.update();

            // bucket slides retraction sequence
            if (gamepad1.x){
                isXSequenceActive=true;
                Xtimer.reset();
            }
            if (isXSequenceActive) {
                bucketPosition=36;

                if (Xtimer.seconds()>0.3){
                    bucketSlidesTarget=0;
                    isXSequenceActive=false;
                }
            }
            // Intake sequence picking up sample in submersible (claw pitch needs to fit over sub)
            if (gamepad1.b){
                isBSequenceActive=true;
                isASequenceActive=false;
                Btimer.reset();

            }
            if (isBSequenceActive) {
                extendoPitchTarget = -960;
                clawWristPosition = 95;
                clawFingerPosition = 86;
                clawPitchPosition = 68;
                innerClawPitchPosition = 20;
                bucketSlidesTarget=0;
                if (Btimer.seconds() > 0.6) {
                    extendoTarget = maxExtendoPosition;

                    isBSequenceActive=false;
                }
            }
            // Transfer sample
            if (gamepad1.a){
                isASequenceActive=true;
                isX2SequenceActive=false;
                Atimer.reset();

            }
            if (isASequenceActive) {

                clawWristPosition = 95;
                clawPitchPosition = 100;
                innerClawPitchPosition = 200;
                bucketPosition=36;

                if (Atimer.seconds() > 0.3){
                    extendoTarget=0;
                }

                if (extendo.getCurrentPosition() < 100) {
                    extendoPitchTarget=0;
                }

                if (Atimer.seconds() > 1.4) {
                    clawFingerPosition = 86;
                }

                if (Atimer.seconds() > 1.7) {
                    clawPitchPosition = 72.4;
                    innerClawPitchPosition = 160;

                }
                if (Atimer.seconds() > 2) {
                    bucketSlidesTarget=bucketSlidesMax;
                    isASequenceActive=false;

                }


            }

            //dropdown intake
            if (gamepad2.x){
                isX2SequenceActive=true;
                X2timer.reset();
            }
            if (isX2SequenceActive) {

                //clawWristPosition = 95;
                clawPitchPosition = 13;
                innerClawPitchPosition = 82;
                bucketPosition=36;
                clawFingerPosition=86;

                if (X2timer.seconds() > 0.3){
                    clawFingerPosition = 0;
                }

                if (X2timer.seconds() > 1) {
                    //clawWristPosition = 95;
                    clawPitchPosition = 68;
                    innerClawPitchPosition = 20;
                    bucketPosition=36;
                    isX2SequenceActive=false;
                }


            }

            if (gamepad2.dpad_down){
                isDownSequenceActive=true;
                downTimer.reset();
            }
            if (isDownSequenceActive){
                clawWristPosition = 95;
                clawPitchPosition = 100;
                innerClawPitchPosition = 200;
                bucketPosition=36;
                if (downTimer.seconds()>0.4){
                    extendoTarget=0;
                    isDownSequenceActive=false;
                }
            }

            if(gamepad2.dpad_up){
                extendoTarget=maxExtendoPosition;
                clawPitchPosition = 68;
                innerClawPitchPosition = 20;
                bucketPosition=36;

            }
            
            //specimen setup sequence (including retraction of extendo after specimen pickup
            if (gamepad2.b){
                isB2SequenceActive=true;
                B2timer.reset();
            }
            if (isB2SequenceActive) {
                clawFingerPosition = 0;
                innerClawPitchPosition = 180;

                if (B2timer.seconds()>0.3){
                    bucketPosition=158;
                    extendoTarget=0;
                    bucketSlidesTarget=200;
                }

                if (B2timer.seconds() > 0.6) {
                    clawPitchPosition = 13;
                    innerClawPitchPosition = 140;
                    extendoPitchTarget=-200;
                }
                if (B2timer.seconds() > 1) {
                    extendoTarget=700;
                    isB2SequenceActive=false;
                }
            }
            //wall specimen pickup sequence

            if (gamepad2.options){
                isOp2SequenceActive=true;
                Op2timer.reset();
            }
            if (isOp2SequenceActive){
                clawWristPosition = 95;

                if (B2timer.seconds()>0.3){
                    clawPitchPosition = 145;
                    innerClawPitchPosition = 78;
                    bucketPosition=168;
                    extendoTarget=0;
                }

                if (B2timer.seconds() > 0.6) {
                    extendoPitchTarget = -960;
                }
                if (B2timer.seconds() > 1) {
                    extendoTarget=0;
                    clawFingerPosition=86;
                    isOp2SequenceActive=false;
                }
            }


            // Extendo retracted 0 ticks
            // Extendo fully extending 36050
            // Dynamic extendo control

            if ((gamepad1.right_bumper) && (extendoTarget <= maxExtendoPosition-30)){
                extendoTarget += 30;
            }
            else if ((gamepad1.right_bumper) && (extendoTarget >= maxExtendoPosition-30)){
                extendoTarget = maxExtendoPosition;
            }
            else if ((gamepad1.left_bumper) && (extendoTarget>=30)){
                extendoTarget -= 30;
            }
            else if ((gamepad1.left_bumper) && (extendoTarget <= 30)){
                extendoTarget = 0;
            }


            // Extendo pitch transfer / default pos 0 ticks
            // Extendo pitch pickup 1425
            /*
            if (gamepad1.dpad_down && !(extendo.getCurrentPosition()>150)){
                if (!isPressingDpad) {
                    if (extendoPitchTarget >= 0 && extendoPitchTarget<450){
                        extendoPitchTarget = 450;
                    }
                    else if (extendoPitchTarget >= 450 && extendoPitchTarget<1421){
                        extendoPitchTarget = 1421;
                    }
                }
                isPressingDpad=true;
            }
            else if (gamepad1.dpad_up && !(extendo.getCurrentPosition()>150)){
                if (!isPressingDpad) {
                    if (extendoPitchTarget <= 1421 && extendoPitchTarget > 450) {
                        extendoPitchTarget = 450;
                    }
                    else if (extendoPitchTarget <= 450 && extendoPitchTarget>0) {
                        extendoPitchTarget = 0;
                    }
                }
                isPressingDpad=true;
            }
            else{
                isPressingDpad=false;
            }

            //dynamic extendo pitch movement - for specimen scoring
            if (-gamepad2.right_stick_y>0&&extendoPitchTarget>=50&&!(extendo.getCurrentPosition()>150)){
                extendoPitchTarget-=20;
            }
            else if (-gamepad2.right_stick_y<0&&extendoPitchTarget<=1300&&!(extendo.getCurrentPosition()>150)){
                extendoPitchTarget+=20;
            }
            if (gamepad2.x&&extendoPitchTarget>=215){
                if (!isPressingX2) {
                    extendoPitchTarget=760;
                    isPressingX2=true;
                }
            }
            else isPressingX2=false;

            // Hang toggle between min and max positions
            if (gamepad2.y){
                if (!isPressingY2) {
                    if (hangTarget == 0) {
                        isPressingY2 = true;
                        hangTarget = 9000;}
                    else if (hangTarget == 9000) {
                        hangTarget = 5400;
                        isPressingY2 = true;
                    }
                    else if (hangTarget == 5400){
                        hangTarget = 0;
                        isPressingY2 = true;
                    }
                }
            }
            else isPressingY2=false;

             */

            // Bucket Slides toggle between min and max positions
            if (gamepad1.y){
                if (!isPressingY) {
                    if (bucketSlidesTarget == 0) {
                        bucketSlidesTarget = bucketSlidesMax;}
                    else {
                        bucketSlidesTarget = 0;
                    }
                    isPressingY=true;
                }
            }
            else isPressingY=false;

            // Claw pitch picking up pos 30.5 degrees
            // Claw transfer pos 215 degrees
            // Claw pitch going into sub 104 degrees
            // Claw pitch to position 0 to 1


            // Claw finger close 0 degrees
            // Claw finger open 50 degrees
            // Claw fingers toggle between open and closed
            // Claw fingers fully open 80 degrees


            // Default perpendicular claw pos 79.5 degrees
            if (gamepad2.left_trigger>0 && clawWristPosition >= 15) {
                clawWristPosition -= 6;
            }
            else if (gamepad2.right_trigger>0 && clawWristPosition <= 175) {
                clawWristPosition += 6;
            }


            else if (gamepad1.right_trigger>0) {
                clawFingerPosition += 1;
            }
            else if (gamepad1.left_trigger>0) {
                clawFingerPosition -= 1;
            }

            // BucketTransfer / default pos 85 degrees
            // Bucket Deposit pos 205 degrees
            if ((gamepad2.a)){
                if (!isPressingA2){
                    isPressingA2=true;
                    if (bucketPosition==36) {bucketPosition=158;} else {bucketPosition=36;}
                }
            }
            else isPressingA2=false;

            if ((gamepad2.left_stick_x<0.2) && (gamepad2.right_stick_x>0.2)){
                if (!isBucketMaxSet){
                    isBucketMaxSet=true;
                    if (bucketSlidesMax==570) {bucketSlidesMax=1070;} else {bucketSlidesMax=570;}
                }
            }
            else isBucketMaxSet=false;

            if(gamepad2.right_bumper){
                clawFingerPosition=86;
            }
            if(gamepad2.left_bumper){
                clawFingerPosition=0;
            }

            if (gamepad2.y){
                if (!isPressingY2){
                    isPressingY2=true;
                    if (extendoPitchTarget==-200) {extendoPitchTarget=-450;} else {extendoPitchTarget=-200;}
                }
            }
            else isPressingY2=false;

            if (gamepad1.options) {
                imu.resetYaw();
                pinpoint.resetPosAndIMU();
            }

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            //double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double botHeading = 0;

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            if (gamepad1.left_trigger>0.5) { // Checks for left trigger input, slows all motors by 50%
                frontLeftPower = 0.5 * (rotY + rotX + rx) / denominator;
                backLeftPower = 0.5 * (rotY - rotX + rx) / denominator;
                frontRightPower = 0.5 * (rotY - rotX - rx) / denominator;
                backRightPower = 0.5 * (rotY + rotX - rx) / denominator;
            }

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            extendo.setPower((extendoTarget - extendo.getCurrentPosition()) * kP);
            extendoPitch.setPower((extendoPitchTarget - extendoPitch.getCurrentPosition()) * kPpitch);
            //hang.setPower((hangTarget - hang.getCurrentPosition()) * kP);
            bucketSlides.setPower((bucketSlidesTarget - bucketSlides.getCurrentPosition()) * kP);

            clawPitchLeft.setPosition(clawPitchPosition/270);
            clawPitchRight.setPosition(clawPitchPosition/270);
            clawFingers.setPosition(clawFingerPosition/180);
            clawWrist.setPosition(clawWristPosition/270);
            innerClawPitch.setPosition(innerClawPitchPosition/270);
            bucket.setPosition(bucketPosition/270);

            telemetry.addData("extendo position", extendo.getCurrentPosition());
            telemetry.addData("extendo target", extendoTarget);
            telemetry.addData("extendo pitch position", extendoPitch.getCurrentPosition());
            telemetry.addData("extendo pitch target", extendoPitchTarget);
            //telemetry.addData("hang pos", hang.getCurrentPosition());
            //telemetry.addData("hang target", hangTarget);
            telemetry.addData("bucketSlides pos", bucketSlides.getCurrentPosition());
            telemetry.addData("bucketSlides target", bucketSlidesTarget);

            telemetry.addData("left claw pitch position", clawPitchLeft.getPosition());
            telemetry.addData("right claw pitch position", clawPitchRight.getPosition());
            telemetry.addData("claw finger position", clawFingers.getPosition());
            telemetry.addData("claw finger position in degree", clawFingerPosition);
            telemetry.addData("claw wrist position", clawWrist.getPosition());
            telemetry.addData("bucket pos", bucket.getPosition());
            telemetry.addData("bucket target", bucketPosition);
            telemetry.addData("inc pos", innerClawPitchPosition);

            telemetry.addData("b seq", isBSequenceActive);
            telemetry.addData("aseq", isASequenceActive);
            telemetry.addData("x2seq", isX2SequenceActive);


            telemetry.addData("bot heading", botHeading);
            telemetry.addData("pinpoint heading", pinpoint.getHeading());
            telemetry.addData("Control hub IMU heading", imu.getRobotYawPitchRollAngles().getYaw());
            telemetry.addData("pinpoint x",pinpoint.getPosX());
            telemetry.addData("pinpoint y", pinpoint.getPosY());
            telemetry.addData("looptime", timer.time());

            telemetry.update();
            timer.reset();
        }
    }
}