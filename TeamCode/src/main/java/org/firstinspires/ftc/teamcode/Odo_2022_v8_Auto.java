package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Auto;
import org.firstinspires.ftc.teamcode.robot.ButtonMgr;
import org.firstinspires.ftc.teamcode.robot.Navigator2;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Tools.Position;

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
    public Navigator2 navigator;
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
        robot.localizer.odoFieldStart = new Position (-35.25,58.75,-90);  //needs to be set before navigator is inited
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

        //Starting point:  odoFieldStart = new Position (-36,63,-90);
       // new Position (-35.25,58.75,-90);

        //auto.driveTo(23,0,0,3,2000);     //forward

        auto.driveTo(-35.25,39.25,-90,3,2000);     //forward

        //fake grab
        auto.driveTo(-35.25,35.25,180,3,2000); //rotate
        auto.driveTo(-58.75,35.25,180,2,2000);  //right
        auto.delay(1500);

        //fake deposit
        auto.driveTo(-39.25,35.25,180,3,2000); // backup
        auto.driveTo(-31.25,39.25,-135,2,2000); //rotate to position
        auto.delay(2000);

        //fake grab
        auto.driveTo(-35.25,35.25,180,3,2000); //rotate
        auto.driveTo(-58.75,35.25,180,2,2000);  //right
        auto.delay(1500);

        //fake deposit
        auto.driveTo(-39.25,35.25,180,3,2000); // backup
        auto.driveTo(-31.25,39.25,-135,2,2000); //rotate to position
        auto.delay(2000);

        //fake grab
        auto.driveTo(-35.25,35.25,180,3,2000); //rotate
        auto.driveTo(-58.75,35.25,180,2,2000);  //right
        auto.delay(1500);

        //return home
        auto.driveTo(-58.75,35.25,-90,3,2000);  //rotate
        auto.driveTo(-58.75,58.75,-90,3,2000);  //rotate
        //delay(3000);
        auto.driveTo(-35.25,58.75,-90,1,3000);  //strafe left
/*        auto.driveTo(-35.25,35.25,-90,3,2000);     //forward

        //fake grab
        auto.driveTo(-35.25,35.25,180,3,2000); //rotate
        auto.driveTo(-58.75,35.25,180,2,2000);  //right
        auto.delay(1500);

        //fake deposit
        auto.driveTo(-35.25,35.25,180,3,2000); // backup
        auto.driveTo(-31.25,39.25,-135,2,2000); //rotate to position
        auto.delay(2000);

        //fake grab
        auto.driveTo(-35.25,35.25,180,3,2000); //rotate
        auto.driveTo(-58.75,35.25,180,2,2000);  //right
        auto.delay(1500);

        //fake deposit
        auto.driveTo(-35.25,35.25,180,3,2000); // backup
        auto.driveTo(-31.25,39.25,-135,2,2000); //rotate to position
        auto.delay(2000);

        //fake grab
        auto.driveTo(-35.25,35.25,180,3,2000); //rotate
        auto.driveTo(-58.75,35.25,180,2,2000);  //right
        auto.delay(1500);

        //return home
        auto.driveTo(-58.75,35.25,-90,3,2000);  //rotate
        auto.driveTo(-58.75,58.75,-90,3,2000);  //rotate
        //delay(3000);
        auto.driveTo(-35.25,58.75,-90,1,3000);  //strafe left

 */
/*
        //fake grab
        auto.driveTo(25.5,0,-90,3,2000); //rotate
        if (!auto.driveTo(25.5,-21.5,-90,2,4000)) {  //right
            //auto.park();  // probably hung up somewhere - give up and attempt to park
            return;
        }
        auto.delay(1500);
        if (elapsedTime.seconds() > 25) {
            //auto.park();
            return;
        }

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
*/
    }

}
