package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.robot.ButtonMgr;
import org.firstinspires.ftc.teamcode.robot.Controls;
import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.LiftbotControls;
import org.firstinspires.ftc.teamcode.robot.LiftbotLifter;
import org.firstinspires.ftc.teamcode.robot.Localizer;
import org.firstinspires.ftc.teamcode.robot.Navigator2;
import org.firstinspires.ftc.teamcode.robot.Navigator3;
import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name = "AA_Liftbot", group = "")
//@Disabled
public class Liftbot extends LinearOpMode {

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
    public Navigator3 navigator;
    public LiftbotControls controls;
    public LiftbotLifter lifter;

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
        lifter = new LiftbotLifter(robot);
        controls = new LiftbotControls(robot, lifter);   // this is likely to change, not necessarily standardized with robot

        robot.useODO = false;
        //robot.reverseDrive = true;  // for AndyMark test
        robot.useDistanceSensors = false;

        elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        robot.init();
        robot.sensors.init();
//        drivetrain.init();
//        navigator.init();
        lifter.init();

        while (!isStarted()) {
            robot.buttonMgr.loop();
            if (robot.buttonMgr.wasTapped(1, ButtonMgr.Buttons.X))
                robot.reverseDrive = !robot.reverseDrive;
            if (robot.buttonMgr.wasTapped(2, ButtonMgr.Buttons.X))
                robot.reverseDrive = !robot.reverseDrive;

            // Prompt user to press start button.
            telemetry.addData(">", "Press Play to start");
            telemetry.addData(">", "Robot Heading = %.1f", robot.returnImuHeading(true));
            telemetry.addData("Drive Type:", robot.reverseDrive ? "AndyMark" : "GobildaBot");
            telemetry.update();
            sleep(20);
        }

        drivetrain.init();
        navigator.init();
        localizer.init();
        navigator.setMaxSpeed(maxSpeed);
        //navigator.setDeltaHeading();

        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {

                robot.loop();               // Clears bulk data and reads IMU
                buttonMgr.loop();           // Processes digital controller input
                localizer.loop();           // Updates odometry X, Y, Rotation
                robot.sensors.loop();       // Update distance sensors, etc.

                addTelemetryLoopStart();

                controls.loop();            // Acts on user controls
                //navigator.setUserDriveSettings(controls.DriveSpeed, controls.DriveAngle, controls.Rotate);

                lifter.loop();
                navigator.loop();           // Automatic navigation actions

                addTelemetryLoopEnd();
                telemetry.update();
            }
        }
    }

    private void addTelemetryLoopStart() {
        telemetry.addData("Loop time (ms)", JavaUtil.formatNumber(calculateLoopTime(), 0));
        telemetry.addData("heading", JavaUtil.formatNumber(robot.returnImuHeading(),2));
        telemetry.addData("rangeL", String.format("%.01f in", robot.sensors.distL));
        telemetry.addData("rangeM", String.format("%.01f in", robot.sensors.distM));
        telemetry.addData("rangeR", String.format("%.01f in", robot.sensors.distR));
//        telemetry.addData("raw__", localizer.odoRawPose.toString(2));
//        telemetry.addData("robot", localizer.odoRobotPose.toString(2));
//        telemetry.addData("final", localizer.odoFinalPose.toString(2));
        robot.localizer.addTeleOpTelemetry();
    }

    private void addTelemetryLoopEnd() {
        telemetry.addData("r (magnitude)", controls.DriveSpeed);
        telemetry.addData("robotAngle", controls.DriveAngle);
        telemetry.addData("rotate", controls.Rotate);
        telemetry.addData("storedHeading", JavaUtil.formatNumber(navigator.storedHeading, 2));
        telemetry.addData("deltaHeading", JavaUtil.formatNumber(navigator.deltaHeading, 2));
//        telemetry.addData("error", JavaUtil.formatNumber(currentError, 2));
//        telemetry.addData("v0", JavaUtil.formatNumber(navigator.v0, 2));
//        telemetry.addData("v1", JavaUtil.formatNumber(navigator.v2, 2));
//        telemetry.addData("v2", JavaUtil.formatNumber(navigator.v1, 2));
//        telemetry.addData("v3", JavaUtil.formatNumber(navigator.v3, 2));
//        telemetry.addData("rot about Z", JavaUtil.formatNumber(robot.returnImuHeading(),2));
//        telemetry.addData("odo Heading", JavaUtil.formatNumber(localizer.returnOdoHeading(), 2));
//        telemetry.addData("Target X", navigator.targetX);
//        telemetry.addData("Target Y", navigator.targetY);
//        telemetry.addData("Target Rot", navigator.targetRot);
//        telemetry.addData ("OdoY", localizer.encoderY);
//        telemetry.addData ("OdoXL", localizer.encoderXL);
//        telemetry.addData ("OdoXR", localizer.encoderXR);
//        telemetry.addData ("X", JavaUtil.formatNumber(localizer.xPos, 2));
//        telemetry.addData ("Y", JavaUtil.formatNumber(localizer.yPos, 2));

//        telemetry.addData("rangeL", String.format("%.01f in", robot.sensors.distL));
//        telemetry.addData("rangeM", String.format("%.01f in", robot.sensors.distM));
//        telemetry.addData("rangeR", String.format("%.01f in", robot.sensors.distR));
    }

    // Calculate loop time for performance optimization
    private double calculateLoopTime() {
        timeLoop = timerLoop.milliseconds();
        timerLoop.reset();
        return timeLoop;
    }

}
