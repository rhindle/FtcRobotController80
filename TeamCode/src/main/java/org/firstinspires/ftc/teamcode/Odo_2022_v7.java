package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.robot.ButtonMgr;
import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.Localizer;
import org.firstinspires.ftc.teamcode.robot.Navigator;
import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name = "3Odo_2022_v7", group = "")
//@Disabled
public class Odo_2022_v7 extends LinearOpMode {

    private BNO055IMU imu;

    private DcMotorEx motor0, motor2, motor1, motor3, odoXL, odoY, odoXR;
//    private long encoder0, encoder2, encoder1, encoder3, encoderY, encoderXL, encoderXR;
//    private long encoderY0, encoderXL0, encoderXR0;

//    private static double eTicksPerInch = 82300 / 48;
//    private static double eTicksPerRotate = 171500; //171738.8; //170000;

//    private double yPos, xPos;
//    private double odoHeading, odoHeading0;

//    private double targetX, targetY, targetRot;
//    private int navigate = 0;

    private ElapsedTime elapsedTime; // = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private ElapsedTime timerLoop = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private double timeLoop;

    final double tileSize = 23.5;  //in inches

//    private double globalHeading;
//    private double globalHeading0;
    private double storedHeading = 0;
    private double deltaHeading = 0;
    private int toggleRotate;

    private final double maxSpeed = 1;//0.2;

    double DriveSpeed, DriveAngle, Rotate;
    Double v0, v2, v1, v3;
//    double averageValue;
//    double highValue;
    double currentError = 0;

//    boolean accurate = true;
//    int navStep = 0;
//    private ElapsedTime navTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
//    int lastStep = 0;

    public ButtonMgr buttonMgr;
    public Robot robot;
    public Localizer localizer;
    public Drivetrain drivetrain;
    public Navigator navigator;

    //DELETE THIS CRAP
    private int countTap = 0;
    private int countRelease = 0;

    // Hypotenuse function I used in Blocky?
    private double math_hypot(float arg0, float arg1) {
        return Math.sqrt(Math.pow(arg0, 2) + Math.pow(arg1, 2));
    }

    @Override
    public void runOpMode() {
        //imu = hardwareMap.get(BNO055IMU.class, "imu");

        robot = new Robot(this);
        buttonMgr = new ButtonMgr(this);
        localizer = new Localizer(this, robot);
        drivetrain = new Drivetrain(this, robot);
        navigator = new Navigator(this, robot, localizer, drivetrain);

        elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        robot.init();
        drivetrain.init();
        navigator.init();

        motor0 = robot.motor0;
        motor1 = robot.motor1;
        motor2 = robot.motor2;
        motor3 = robot.motor3;
//        odoY = robot.motor0B;
//        odoXR = robot.motor1B;
//        odoXL = robot.motor2B;
        imu = robot.sensorIMU;

        v0 = navigator.v0;
        v1 = navigator.v1;
        v2 = navigator.v2;
        v3 = navigator.v3;

        //initMotors();

        while (!isStarted()) {
            // Prompt user to press start button.
            telemetry.addData(">", "Press Play to start");
            telemetry.addData(">", "Robot Heading = %.1f", robot.returnImuHeading(true));
            telemetry.update();
            sleep(100);
        }
//
//        encoderY0 = odoY.getCurrentPosition();
//        encoderXL0 = odoXL.getCurrentPosition();
//        encoderXR0 = odoXR.getCurrentPosition();
//        globalHeading0 = robot.imuHeading(true);
//        odoHeading0 = getOdoHeading();

        localizer.init();
        navigator.setMaxSpeed(maxSpeed);

        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {

                robot.loop();
                buttonMgr.updateAll();
                localizer.loop();

//                //encoder0 = motor0.getCurrentPosition();
//                //encoder1 = motor1.getCurrentPosition();
//                //encoder2 = motor2.getCurrentPosition();
//                //encoder3 = motor3.getCurrentPosition();
//                encoderY = odoY.getCurrentPosition();
//                encoderXL = odoXL.getCurrentPosition();
//                encoderXR = odoXR.getCurrentPosition();
//
//                // Display orientation info.
//                globalHeading = robot.imuHeading();
//                odoHeading = getOdoHeading();

//                updateXY();

                addTelemetryLoopStart();
                Controls();
                navigator.setUserDriveSettings(DriveSpeed, DriveAngle, Rotate);

                navigator.loop();

//                if (navigator.navigate == 1) {
//                    navigator.autoDrive();
//                }
//                else if (navigator.navigate == 2) {
//                    navigator.scriptedNav2();
//                }
//                else {
//                    navigator.userDrive(DriveSpeed, DriveAngle, Rotate);
//                }
//
//                // set motor power
//                drivetrain.setDrivePowers(v0, v1, v2, v3);
//                motor0.setPower(v0);
//                motor2.setPower(v2);
//                motor1.setPower(v1);
//                motor3.setPower(v3);

                addTelemetryLoopEnd();

                telemetry.update();
            }
        }
    }

    // Interpret user control inputs
    private void Controls() {

        telemetry.addData("BACK PRESSED", buttonMgr.isPressed(1, ButtonMgr.Buttons.BACK));
        if (buttonMgr.wasTapped(1, ButtonMgr.Buttons.BACK)) {
            countTap++;
        };
        telemetry.addData("BACK HELD", buttonMgr.isHeld(1, ButtonMgr.Buttons.BACK));
        if (buttonMgr.wasReleased(1, ButtonMgr.Buttons.BACK)) {
            countRelease++;
        };
        telemetry.addData("BACK TAP COUNT", countTap);
        telemetry.addData("BACK REL COUNT", countRelease);

        // This blob is for manually entering destinations
        if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.A)) {
            navigator.setTargetToCurrentPosition();
        }

        if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.dpadUP)) {
            //targetX = targetX + (gamepad2.back ? 1 : tileSize/2.0);
            navigator.setTargetByDelta((gamepad2.back ? 1 : tileSize/2.0),0,0);
        }
        if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.dpadDOWN)) {
            //targetX = targetX - (gamepad2.back ? 1 : tileSize/2.0);
            navigator.setTargetByDelta(-(gamepad2.back ? 1 : tileSize/2.0),0,0);
        }
        if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.dpadLEFT)) {
            //targetY = targetY + (gamepad2.back ? 1 : tileSize/2.0);
            navigator.setTargetByDelta(0, (gamepad2.back ? 1 : tileSize/2.0),0);
        }
        if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.dpadRIGHT)) {
            //targetY = targetY - (gamepad2.back ? 1 : tileSize/2.0);
            navigator.setTargetByDelta(0, -(gamepad2.back ? 1 : tileSize/2.0),0);
        }
        if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.X)) {
            //targetRot = targetRot + (gamepad2.back ? 2 : 45);
            navigator.setTargetByDelta(0,0,(gamepad2.back ? 2 : 45));
        }
        if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.Y)) {
            //targetRot = targetRot - (gamepad2.back ? 2 : 45);
            navigator.setTargetByDelta(0,0,-(gamepad2.back ? 2 : 45));
        }

        if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.START)) {
            navigator.beginAutoDrive();
        }

        // Cancels auto navigation
        if (buttonMgr.isPressed(2, ButtonMgr.Buttons.B)) {
            navigator.cancelAutoNavigation();
        }

        // Set to home position
        if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.rightBUMPER)) {
            navigator.setTargetToZeroPosition();
        }

        // Begin scripted navigation
        if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.leftBUMPER)) {
            navigator.beginScriptedNav();
        }

        // Get speed and direction from left stick
        DriveSpeed = JavaUtil.minOfList(JavaUtil.createListWith(1, math_hypot(gamepad1.left_stick_x, gamepad1.left_stick_y)));
        DriveAngle = Math.atan2(-gamepad1.left_stick_x, -gamepad1.left_stick_y) / Math.PI * 180;
        // Adjust heading for corrections
        DriveAngle = DriveAngle - storedHeading + deltaHeading;

        // overall plan here is to not autocorrect if toggleRotate is true
        // this stupid toggleRotate countup is to catch IMU latency
        if (toggleRotate < 0) {
            toggleRotate++;
            storedHeading = localizer.returnGlobalHeading();
        }
        if (gamepad1.right_stick_x != 0) {
            toggleRotate = 1;
            storedHeading = localizer.returnGlobalHeading();;
            Rotate = Math.pow(gamepad1.right_stick_x, 3);
        } else {
            Rotate = 0;
            if (toggleRotate == 1) {
                toggleRotate = -10;
                storedHeading = localizer.returnGlobalHeading();;
            }
        }

        // Store heading correction
        if (gamepad1.right_stick_button) {
            deltaHeading = storedHeading;
        }

        // Correct the heading if not currently being controlled
        if (toggleRotate == 0) {
            currentError = navigator.getError(storedHeading);
            Rotate = currentError / -15 * (DriveSpeed + 0.2);   // base this on speed?
        }

    }

    // Motor init
//    private void initMotors() {
//        drivetrain.initMotors();
//        motor0.setDirection(DcMotorEx.Direction.REVERSE);
//        motor2.setDirection(DcMotorEx.Direction.REVERSE);
//        motor1.setDirection(DcMotorEx.Direction.FORWARD);
//        motor3.setDirection(DcMotorEx.Direction.FORWARD);
////        odoY.setDirection(DcMotorEx.Direction.FORWARD);
////        odoXL.setDirection(DcMotorEx.Direction.REVERSE);
////        odoXR.setDirection(DcMotorEx.Direction.REVERSE);
//        motor0.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        motor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        motor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        motor3.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
////        odoY.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
////        odoXL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
////        odoXR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        motor0.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        motor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        motor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        motor3.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        motor0.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
//        motor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
//        motor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
//        motor3.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
//    }

    private void addTelemetryLoopStart() {
        telemetry.addData("Loop time (ms)", JavaUtil.formatNumber(calculateLoopTime(), 0));
        telemetry.addData("heading", JavaUtil.formatNumber(robot.returnImuHeading(),2));
    }

    private void addTelemetryLoopEnd() {
        telemetry.addData("r (magnitude)", DriveSpeed);
        telemetry.addData("robotAngle", DriveAngle);
        telemetry.addData("storedHeading", JavaUtil.formatNumber(storedHeading, 2));
        telemetry.addData("deltaHeading", JavaUtil.formatNumber(deltaHeading, 2));
        telemetry.addData("error", JavaUtil.formatNumber(currentError, 2));
        telemetry.addData("v0", JavaUtil.formatNumber(v0, 2));
        telemetry.addData("v1", JavaUtil.formatNumber(v2, 2));
        telemetry.addData("v2", JavaUtil.formatNumber(v1, 2));
        telemetry.addData("v3", JavaUtil.formatNumber(v3, 2));
        telemetry.addData("rot about Z", JavaUtil.formatNumber(robot.returnImuHeading(),2));
        telemetry.addData("odo Heading", JavaUtil.formatNumber(localizer.returnOdoHeading(), 2));
        telemetry.addData("Target X", navigator.targetX);
        telemetry.addData("Target Y", navigator.targetY);
        telemetry.addData("Target Rot", navigator.targetRot);
        telemetry.addData ("OdoY", localizer.encoderY);
        telemetry.addData ("OdoXL", localizer.encoderXL);
        telemetry.addData ("OdoXR", localizer.encoderXR);
        telemetry.addData ("X", JavaUtil.formatNumber(localizer.xPos, 2));
        telemetry.addData ("Y", JavaUtil.formatNumber(localizer.yPos, 2));
    }

    // Calculate loop time for performance optimization
    private double calculateLoopTime() {
        timeLoop = timerLoop.milliseconds();
        timerLoop.reset();
        return timeLoop;
    }
}
