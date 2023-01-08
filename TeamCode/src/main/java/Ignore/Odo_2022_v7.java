package Ignore;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.robot.ButtonMgr;
import org.firstinspires.ftc.teamcode.robot.Controls;
import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.Localizer;
import org.firstinspires.ftc.teamcode.robot.Navigator;
import org.firstinspires.ftc.teamcode.robot.Navigator2;
import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name = "3Odo_2022_v7_", group = "")
@Disabled
public class Odo_2022_v7 extends LinearOpMode {

    private ElapsedTime elapsedTime; // = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private ElapsedTime timerLoop = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private double timeLoop;

    private final double maxSpeed = 1;//0.2;

//    double DriveSpeed, DriveAngle, Rotate;
    double currentError = 0;

    public ButtonMgr buttonMgr;
    public Robot robot;
    public Localizer localizer;
    public Drivetrain drivetrain;
    public Navigator2 navigator;
    public Controls controls;

    @Override
    public void runOpMode() {
        //imu = hardwareMap.get(BNO055IMU.class, "imu");

        robot = new Robot(this);
//        buttonMgr = new ButtonMgr(this);
//        localizer = new Localizer(this, robot);
//        drivetrain = new Drivetrain(this, robot);
//        navigator = new Navigator(this, robot, localizer, drivetrain);
//        controls = new Controls(this, buttonMgr, navigator);
        buttonMgr = robot.buttonMgr;
        localizer = robot.localizer;
        drivetrain = robot.drivetrain;
        navigator = robot.navigator;
        //controls = robot.controls;
        controls = new Controls(robot);   // this is likely to change, not necessarily standardized with robot

        elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        robot.init();
        drivetrain.init();
        navigator.init();

        while (!isStarted()) {
            // Prompt user to press start button.
            telemetry.addData(">", "Press Play to start");
            telemetry.addData(">", "Robot Heading = %.1f", robot.returnImuHeading(true));
            telemetry.update();
            sleep(100);
        }

        localizer.init();
        navigator.setMaxSpeed(maxSpeed);

        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {

                robot.loop();               // Clears bulk data and reads IMU
                buttonMgr.loop();           // Processes digital controller input
                localizer.loop();           // Updates odometry X, Y, Rotation

                addTelemetryLoopStart();

                controls.loop();            // Acts on user controls
                //navigator.setUserDriveSettings(controls.DriveSpeed, controls.DriveAngle, controls.Rotate);

                navigator.loop();           // Automatic navigation actions

                addTelemetryLoopEnd();
                telemetry.update();
            }
        }
    }

//    // Interpret user control inputs
//    private void Controls() {
//
//        // Reset target navigation to present position
//        if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.A))
//            navigator.setTargetToCurrentPosition();
//
//        // This blob is for manually entering destinations by adjusting X, Y, Rot
//        if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.dpadUP))
//            navigator.setTargetByDelta((gamepad2.back ? 1 : tileSize/2.0),0,0);
//        if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.dpadDOWN))
//            navigator.setTargetByDelta(-(gamepad2.back ? 1 : tileSize/2.0),0,0);
//        if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.dpadLEFT))
//            navigator.setTargetByDelta(0, (gamepad2.back ? 1 : tileSize/2.0),0);
//        if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.dpadRIGHT))
//            navigator.setTargetByDelta(0, -(gamepad2.back ? 1 : tileSize/2.0),0);
//        if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.X))
//            navigator.setTargetByDelta(0,0,(gamepad2.back ? 2 : 45));
//        if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.Y))
//            navigator.setTargetByDelta(0,0,-(gamepad2.back ? 2 : 45));
//
//        // Toggle FCD
//        if (buttonMgr.wasTapped(1, ButtonMgr.Buttons.START))
//            navigator.toggleFieldCentricDrive();
//
//        // Toggle HeadingHold
//        if (buttonMgr.wasTapped(1, ButtonMgr.Buttons.BACK))
//            navigator.toggleHeadingHold();
//
//        // Start auto navigation
//        if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.START))
//            navigator.beginAutoDrive();
//
//        // Cancels auto navigation
//        if (buttonMgr.isPressed(2, ButtonMgr.Buttons.B))
//            navigator.cancelAutoNavigation();
//
//        // Set to home position
//        if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.rightBUMPER))
//            navigator.setTargetToZeroPosition();
//
//        // Begin scripted navigation
//        if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.leftBUMPER))
//            navigator.beginScriptedNav();
//
//        // Store heading correction
//        if (buttonMgr.wasReleased(1, ButtonMgr.Buttons.rightJoyStickBUTTON))
//            navigator.setDeltaHeading();
//
//        // Get speed and direction from left stick
//        DriveSpeed = JavaUtil.minOfList(JavaUtil.createListWith(1, Support.mathHypotenuse(gamepad1.left_stick_x, gamepad1.left_stick_y)));
//        DriveAngle = Math.atan2(-gamepad1.left_stick_x, -gamepad1.left_stick_y) / Math.PI * 180;
//        // Get rotation from right stick
//        Rotate = Math.pow(gamepad1.right_stick_x, 1);
//        navigator.handleRotate(Rotate);
//    }

    private void addTelemetryLoopStart() {
        telemetry.addData("Loop time (ms)", JavaUtil.formatNumber(calculateLoopTime(), 0));
        telemetry.addData("heading", JavaUtil.formatNumber(robot.returnImuHeading(),2));
    }

    private void addTelemetryLoopEnd() {
        telemetry.addData("r (magnitude)", controls.DriveSpeed);
        telemetry.addData("robotAngle", controls.DriveAngle);
        telemetry.addData("rotate", controls.Rotate);
        telemetry.addData("storedHeading", JavaUtil.formatNumber(navigator.storedHeading, 2));
        telemetry.addData("deltaHeading", JavaUtil.formatNumber(navigator.deltaHeading, 2));
        telemetry.addData("error", JavaUtil.formatNumber(currentError, 2));
        telemetry.addData("v0", JavaUtil.formatNumber(navigator.v0, 2));
        telemetry.addData("v1", JavaUtil.formatNumber(navigator.v2, 2));
        telemetry.addData("v2", JavaUtil.formatNumber(navigator.v1, 2));
        telemetry.addData("v3", JavaUtil.formatNumber(navigator.v3, 2));
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

//    // Hypotenuse function I used in Blocky?
//    private double mathHypotenuse(float arg0, float arg1) {
//        return Math.sqrt(Math.pow(arg0, 2) + Math.pow(arg1, 2));
//    }
}
