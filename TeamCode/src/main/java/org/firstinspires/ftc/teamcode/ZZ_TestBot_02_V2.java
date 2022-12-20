package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.ButtonMgr;

//import com.qualcomm.robotcore.util.Range;
//import java.util.Locale;

@TeleOp (name="ZZ_TestBot_02", group="Test")
//@Disabled
public class ZZ_TestBot_02_V2 extends LinearOpMode {

//    Robot robot   = new Robot(this);
//    ButtonMgr buttonMgr = new ButtonMgr(this);
//    Orientation angles;

    Robot robot;
    ButtonMgr buttonMgr;

    @Override
    public void runOpMode() {
        robot = new Robot(this);
        buttonMgr = new ButtonMgr(this);
        robot.init();

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        robot.motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && !robot.sensorIMU.isGyroCalibrated())  {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        robot.motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            //angles = robot.sensorIMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData(">", "Robot Heading = %.1f", robot.returnImuHeading(true));
            telemetry.update();
            sleep(100);
        }

        double tgtPower;
        double tgtPosition = 0.5;

        //double counter = 0;

        ElapsedTime loopElapsedTime = new ElapsedTime();

        int tgtMotor = 0;
        int tgtServo = 0;

        boolean toggleLB = false;
        boolean toggleRB = false;
        boolean toggleA = false;
        boolean tgtServoLive = false;

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};
        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;
        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            robot.loop();  // this will take care of clearing out the bulk reads
            buttonMgr.updateAll();

            //counter++;

            /* Color Sensor Section */
//            Color.RGBToHSV((int) (robot.sensorColor.red() * SCALE_FACTOR),
//                    (int) (robot.sensorColor.green() * SCALE_FACTOR),
//                    (int) (robot.sensorColor.blue() * SCALE_FACTOR),
//                    hsvValues);

            /* IMU */
            //angles = robot.sensorIMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            /* Check for button presses to switch test motor & servo */
            // Left bumper switches motors
            //if (gamepad1.left_bumper && !toggleLB) {
            if (buttonMgr.wasTapped(1, ButtonMgr.Buttons.leftBUMPER)) {
                stopAllMotors();
                tgtMotor++;
                if (tgtMotor>3) tgtMotor=0;
                //toggleLB=true;
            }
            // Right bumper switches servos
            //if (gamepad1.right_bumper && !toggleRB) {
            if (buttonMgr.wasTapped(1, ButtonMgr.Buttons.rightBUMPER)) {
                    tgtServo++;
                if (tgtServo>5) tgtServo=0;
                //toggleRB=true;
                /* Get the position of the target servo.
                 *  (This would be a lot nicer in an array) */
                switch (tgtServo) {
                    case 0: tgtPosition=robot.servo0.getPosition(); break;
                    case 1: tgtPosition=robot.servo1.getPosition(); break;
                    case 2: tgtPosition=robot.servo2.getPosition(); break;
                    case 3: tgtPosition=robot.servo3.getPosition(); break;
                    case 4: tgtPosition=robot.servo4.getPosition(); break;
                    case 5: tgtPosition=robot.servo5.getPosition(); break;
                    default: break;
                }
            }
            // Y button resets the encoder for the current target motor
            if (gamepad1.y) {
                switch (tgtMotor) {
                    case 0: robot.motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); robot.motor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); break;
                    case 1: robot.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); robot.motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); break;
                    case 2: robot.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); robot.motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); break;
                    case 3: robot.motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); robot.motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); break;
                    default: break;
                }
            }
            // A button toggles whether the servo position will be applied "Live"
            //if (gamepad1.a & !toggleA) {
            if (buttonMgr.wasTapped(1, ButtonMgr.Buttons.A)) {
                tgtServoLive=!tgtServoLive;
            //    toggleA = true;
            }
            // Toggles for certain buttons to avoid repeating actions
            //if (!gamepad1.left_bumper) toggleLB=false;
            //if (!gamepad1.right_bumper) toggleRB=false;
            //if (!gamepad1.a) toggleA=false;

            /* Check controls for changes to target motor power and target servo position */
            // Left Stick controls motor power
            // X button will make the motor full speed, otherwise only 25%
            // Right Stick adjusts target servo position
            // B button panic stops all motors (should no longer be necessary)
            // Start button sets servo to center position (to stop if in continuous mode)
            // !!! Can't seem to back the "back" button work ???
            tgtPower = -gamepad1.left_stick_y * (gamepad1.x ? 1 : 0.25);
            if (!gamepad1.start) {
                tgtPosition = Math.max(0, Math.min(1, tgtPosition - gamepad1.right_stick_y * .025));
            } else {
                // added "stop" function in case the servo is in continuous mode
                tgtPosition = 0.5;
            }
            if (gamepad1.b) {
                stopAllMotors();
                tgtPower=0;
            }

            /* Update the servo position */
            if (tgtServoLive) {
                switch (tgtServo) {
                    case 0: robot.servo0.setPosition(tgtPosition); break;
                    case 1: robot.servo1.setPosition(tgtPosition); break;
                    case 2: robot.servo2.setPosition(tgtPosition); break;
                    case 3: robot.servo3.setPosition(tgtPosition); break;
                    case 4: robot.servo4.setPosition(tgtPosition); break;
                    case 5: robot.servo5.setPosition(tgtPosition); break;
                    default: break;
                }
            }

            /* Set the motor power */
            switch (tgtMotor) {
                case 0: robot.motor0.setPower(tgtPower); break;
                case 1: robot.motor1.setPower(tgtPower); break;
                case 2: robot.motor2.setPower(tgtPower); break;
                case 3: robot.motor3.setPower(tgtPower); break;
                default: break;
            }

            // If needed, could disable the servo signal with Servo.getController().pwmDisable()

            /* Add extensive telemetry for debugging */
            telemetry.addLine()
                .addData("Mtr",tgtMotor)
                .addData("Pwr", "%.2f", tgtPower)
                .addData("Srv", tgtServo)
                .addData("Pos","%.3f", tgtPosition)
                .addData("Live?", tgtServoLive);
            telemetry.addLine()
                .addData("Encoder 0", robot.motor0.getCurrentPosition())
                .addData("1", robot.motor1.getCurrentPosition())
                .addData("2", robot.motor2.getCurrentPosition())
                .addData("3", robot.motor3.getCurrentPosition());
            telemetry.addLine()
                .addData("Motor 0", "%.2f", robot.motor0.getPower())
                .addData("1", "%.2f", robot.motor1.getPower())
                .addData("2", "%.2f", robot.motor2.getPower())
                .addData("3", "%.2f", robot.motor3.getPower());
            telemetry.addLine()
                .addData("S 0", "%.2f", robot.servo0.getPosition())
                .addData("1","%.2f", robot.servo1.getPosition())
                .addData("2","%.2f", robot.servo2.getPosition())
                .addData("3","%.2f",  robot.servo3.getPosition())
                .addData("4","%.2f", robot.servo4.getPosition())
                .addData("5","%.2f", robot.servo5.getPosition());
//            telemetry.addLine()
//                .addData("Color A", robot.sensorColor.alpha())
//                .addData("R", robot.sensorColor.red())
//                .addData("G", robot.sensorColor.green())
//                .addData("B", robot.sensorColor.blue());
//            telemetry.addLine()
//                .addData("Hue", "%.1f", hsvValues[0])
//                .addData("Distance (cm)", "%.01f", robot.sensorDistance.getDistance(DistanceUnit.CM));
            telemetry.addLine()
                .addData("D 0", (robot.digital0.getState() ? "T" : "F") +
                        " | 1 : " + (robot.digital1.getState() ? "T" : "F") +
                        " | 2 : " + (robot.digital2.getState() ? "T" : "F")  +
                        " | 3 : " + (robot.digital3.getState() ? "T" : "F")+
                        " | 4 : " + (robot.digital4.getState() ? "T" : "F") +
                        " | 5 : " + (robot.digital5.getState() ? "T" : "F") +
                        " | 6 : " + (robot.digital6.getState() ? "T" : "F") +
                        " | 7 : " + (robot.digital7.getState() ? "T" : "F"));
            telemetry.addData("Heading", "%.1f", robot.returnImuHeading());
            //telemetry.addData("Counter", counter);
            //telemetry.addData("LoopSpeed","%.1f",calcLoopSpeed());
            telemetry.addData("LoopTime(ms)","%.1f",loopElapsedTime.milliseconds());
            telemetry.addData("LoopSpeed(lps)","%.1f",1/(loopElapsedTime.milliseconds()/1000));
            loopElapsedTime.reset();
            telemetry.update();
        }
    }

    public void stopAllMotors () {
        robot.motor0.setPower(0);
        robot.motor1.setPower(0);
        robot.motor2.setPower(0);
        robot.motor3.setPower(0);
    }

//    static int loopCounter = 0;
//    static double loopSpeed = 0;
//    static ElapsedTime loopElapsedTime = new ElapsedTime();
//
//    private double calcLoopSpeed() {
//
//        loopCounter += 1;
//        if (loopElapsedTime.milliseconds() > 1000) {
//            //loopSpeedText = JavaUtil.formatNumber(loopCounter / (elapsedTime.milliseconds() / 1000), 1);
//            loopSpeed = loopCounter / (loopElapsedTime.milliseconds() / 1000);
//            loopCounter = 0;
//            loopElapsedTime.reset();
//        }
//        return loopSpeed;
//    }

}