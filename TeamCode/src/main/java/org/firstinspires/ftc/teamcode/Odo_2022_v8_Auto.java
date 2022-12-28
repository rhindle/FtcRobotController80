package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.robot.Auto;
import org.firstinspires.ftc.teamcode.robot.ButtonMgr;
import org.firstinspires.ftc.teamcode.robot.Controls;
import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.Localizer;
import org.firstinspires.ftc.teamcode.robot.Navigator;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous(name = "3Odo_2022_v8_Auto", group = "")
//@Disabled
public class Odo_2022_v8_Auto extends LinearOpMode {

    private ElapsedTime elapsedTime; // = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

//    private ElapsedTime timerLoop = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
//    private double timeLoop;

    private final double maxSpeed = 1;//0.2;

//    double DriveSpeed, DriveAngle, Rotate;
    double currentError = 0;

//    public ButtonMgr buttonMgr;
    public Robot robot;
//    public Localizer localizer;
//    public Drivetrain drivetrain;
    public Navigator navigator;
//    public Controls controls;
    public Auto auto;

    @Override
    public void runOpMode() {
        //imu = hardwareMap.get(BNO055IMU.class, "imu");

        this.robot = new Robot(this);
//        buttonMgr = robot.buttonMgr;
//        localizer = robot.localizer;
//        drivetrain = robot.drivetrain;
        this.navigator = robot.navigator;
        //controls = robot.controls;
//        controls = new Controls(robot);   // this is likely to change, not necessarily standardized with robot
        auto = new Auto(robot);
//       controls = auto.controls;

        elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        robot.init();
        robot.sensors.init();
        robot.drivetrain.init();
        navigator.init();

        while (!isStarted()) {
            // Prompt user to press start button.
            robot.buttonMgr.loop();
            if (robot.buttonMgr.wasTapped(1, ButtonMgr.Buttons.A)) robot.localizer.toggleUseFusedHeading();
            telemetry.addData(">", "Press Play to start");
            telemetry.addData(">", "Robot Heading = %.1f", robot.returnImuHeading(true));
            telemetry.addData("Heading Type:", robot.localizer.useFusedHeading ? "FUSED/HYBRID" : "IMU ONLY");
            telemetry.update();
            sleep(100);
        }

        robot.localizer.init();
        navigator.setMaxSpeed(maxSpeed);
        elapsedTime.reset();

        if (opModeIsActive()) {
            navigator.setTargetToCurrentPosition();
//            navigator.setAutoDrive(true);
            // Put run blocks here.

            autonomousV1();

            while (opModeIsActive()) {
                auto.robotLoop();
            }
        }
    }

    private void autonomousV1() {
        //navigator.setTargetToCurrentPosition();
        //navigator.setAutoDrive(true);
        navigator.setUseAutoDistanceActivation(false);
        navigator.setUseHeadingHold(false);
//        driveTo(10,0,-45,0,3000);
//        driveTo(10,10,-90,2,3000);
//        driveTo(10,12,-90,1,1000);

//        driveTo(20,3,-45,0,2000);
//        driveTo(25.5,-21.5,-90,2,2000);
//        delay(3000);
//        driveTo(0,-21.5,0,3,2000);
//        delay(3000);
//        driveTo(0,0,0,1,3000);

        auto.driveTo(23,0,0,3,2000);     //forward

        //fake grab
        auto.driveTo(25.5,0,-90,3,2000); //rotate
        auto.driveTo(25.5,-21.5,-90,2,2000);  //right
        auto.delay(1500);

        //fake deposit
        auto.driveTo(25.5,0,-90,3,2000); // backup
        //auto.driveTo(25.5,0,-45,2,2000); //rotate to position
        auto.driveTo(20.5,3,-45,2,2000); //rotate to position
        auto.delay(2000);

        //fake grab
        auto.driveTo(25.5,0,-90,3,2000); //rotate
        auto.driveTo(25.5,-21.5,-90,2,2000);  //right
        auto.delay(1500);

        //fake deposit
        auto.driveTo(25.5,0,-90,3,2000); // backup
        //auto.driveTo(25.5,0,-45,2,2000); //rotate to position
        auto.driveTo(20.5,3,-45,2,2000); //rotate to positio
        auto.delay(2000);

        //fake grab
        auto.driveTo(25.5,0,-90,3,2000); //rotate
        auto.driveTo(25.5,-21.5,-90,2,2000);  //right
        auto.delay(1500);

        auto.driveTo(23,-23,0,3,2000);        //rotate
        auto.driveTo(0,-23,0,3,2000);         //back
        //delay(3000);
        auto.driveTo(0,0,0,1,3000);           //strafe left

    }

}
