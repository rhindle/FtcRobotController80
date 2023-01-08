package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Auto;
import org.firstinspires.ftc.teamcode.robot.ButtonMgr;
import org.firstinspires.ftc.teamcode.robot.Navigator2;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Tools.Position;
import org.firstinspires.ftc.teamcode.robot.Tools.Functions;

@Autonomous(name = "3Odo_2022_v8_Auto2", group = "")
//@Disabled
public class Odo_2022_v8_Auto2 extends LinearOpMode {

    private ElapsedTime elapsedTime; // = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private final double maxSpeed = 1;//0.2;

    public Robot robot;
    public Navigator2 navigator;
    public Auto auto;

    boolean home = true;

    boolean isBlueAlliance = true;
    boolean isRightSide = true;

    @Override
    public void runOpMode() {

        this.robot = new Robot(this);
        this.navigator = robot.navigator;
        auto = new Auto(robot);

        elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        robot.init();
        robot.sensors.init();
        robot.drivetrain.init();

        while (!isStarted()) {
            // Prompt user to press start button.
            robot.buttonMgr.loop();
            if (robot.buttonMgr.wasTapped(1, ButtonMgr.Buttons.A))
                robot.localizer.toggleUseFusedHeading();
            if (robot.buttonMgr.wasTapped(1, ButtonMgr.Buttons.Y))
                home = !home;
            if (robot.buttonMgr.wasTapped(1, ButtonMgr.Buttons.dpadLEFT))
                isRightSide = false;
            if (robot.buttonMgr.wasTapped(1, ButtonMgr.Buttons.dpadRIGHT))
                isRightSide = true;
            if (robot.buttonMgr.wasTapped(1, ButtonMgr.Buttons.X))
                isBlueAlliance = true;
            if (robot.buttonMgr.wasTapped(1, ButtonMgr.Buttons.B))
                isBlueAlliance = false;


            telemetry.addData(">", "Press Play to start");
            telemetry.addData(">", "Robot Heading = %.1f", robot.returnImuHeading(true));
            telemetry.addData("Heading Type:", robot.localizer.useFusedHeading ? "FUSED/HYBRID" : "IMU ONLY");
            telemetry.addData("Test Type:", home ? "Home" : "Kroon");
            telemetry.addData("Alliance:", isBlueAlliance ? "BLUE" : "RED");
            telemetry.addData("Left/Right:", isRightSide ? "RIGHT" : "LEFT");
            telemetry.update();
            sleep(20);
        }

        if (home) {
            robot.localizer.odoFieldStart = modifyPosition(new Position(-35.25, 58.75, -90));  //needs to be set before navigator is inited
        } else {
            //Vector3(-1.5,2.68,90) = -35.25, 62.98, 90
            robot.localizer.odoFieldStart = modifyPosition(new Position(-35.25, 63, 90));  //needs to be set before navigator is inited
            //for testing at home...  comment next line out!
            //robot.localizer.odoFieldStart = modifyPosition(new Position(-35.25, 35.25, 90));  //needs to be set before navigator is inited
        }
        navigator.init();

        robot.localizer.init();
        navigator.setMaxSpeed(maxSpeed);
        elapsedTime.reset();

        if (opModeIsActive()) {
            navigator.setTargetToCurrentPosition();

            if (home) autonomousV1(); else autonomousBig();

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

        Position posFirstStop        = new Position(-35.25, 39.25, -90);
        DriveToData dtdStackGrab     = new DriveToData(new Position(-58.75, 35.25, 180),2,2000);
        Position posPreDrop          = new Position(-39.25, 35.25, 180);
        DriveToData dtdDrop          = new DriveToData(new Position(-31.25, 39.25, -135),2,2000);
        Position posPostDropPreGrab  = new Position(-35.25, 35.25, 180);

        DriveToData dtdGrabToHome    = new DriveToData(new Position(-58.75, 35.25, -90), 3,2000);
        DriveToData dtdGrabToHome2   = new DriveToData(new Position(-58.75, 58.75, -90), 3, 2000);

        driveToPosition(posFirstStop, 3, 2000);     //forward

        //fake grab
        driveToPosition(posPostDropPreGrab, 3, 2000); //rotate
        driveToPosition(dtdStackGrab);
        auto.delay(1500);

        //fake deposit
        driveToPosition(posPreDrop, 3, 2000); // backup
        driveToPosition(dtdDrop);
        auto.delay(2000);

        //fake grab
        driveToPosition(posPostDropPreGrab, 3, 2000); //rotate
        driveToPosition(dtdStackGrab);
        auto.delay(1500);

        //fake deposit
        driveToPosition(posPreDrop, 3, 2000); // backup
        driveToPosition(dtdDrop);
        auto.delay(2000);

        //fake grab
        driveToPosition(posPostDropPreGrab, 3, 2000); //rotate
        driveToPosition(dtdStackGrab);
        auto.delay(1500);

        //return home
        driveToPosition(dtdGrabToHome);
        driveToPosition(dtdGrabToHome2);
        driveToPosition(modifyPosition(robot.localizer.odoFieldStart), 1, 3000);  //strafe left

    }

    private void autonomousBig() {
        navigator.setUseAutoDistanceActivation(false);
        navigator.setUseHeadingHold(false);

        /*
        Vector3 blueLoadedPrep = new Vector3(-1.5,.5,90);
        Vector3 blueTallPrep = new Vector3(-1.5,0.5,135);
        Vector3 blueTallPrep180 = new Vector3(-1.5,0.5,180);
        Vector3 blueTall = new Vector3(-1.2, 0.2, 135);
        Vector3 blueStack = new Vector3(-2.58,.5,180);
        Vector3 blueStackPrep = new Vector3(-2.4,.5,180);
        Vector3[] position = {
                blueLoadedPrep,
                blueTall,
                blueTallPrep,
                blueTallPrep180,
                blueStackPrep,
                blueStack,
                blueTallPrep180,
                blueTall,
                blueTallPrep,
                blueTallPrep180,
                blueStackPrep,
                blueStack,
                blueTallPrep180,
                blueTall
        };
        */

        DriveToData bluLoadedPrep  = new DriveToData(true, new Position(-1.5,0.67,90),3,3000);
        DriveToData bluTall        = new DriveToData(true, new Position(-1.3,0.3,135),2,2000);
        DriveToData bluTallPrep    = new DriveToData(true, new Position(-1.5,0.5,135),3,2000);
        DriveToData bluTallPrep180 = new DriveToData(true, new Position(-1.5,0.5,180),3,2000);
        DriveToData bluStackPrep   = new DriveToData(true, new Position(-2.2,0.5,180),3,2000);
        DriveToData bluStackAvoid  = new DriveToData(true, new Position(-2.0,0.6,180),3,2000);
        DriveToData bluStack       = new DriveToData(true, new Position(-2.5,0.5,180),2,2000);
        DriveToData bluPark        = new DriveToData(true, new Position(-1.5,0.5,-90),1,2000);

        driveToPosition(bluLoadedPrep);
        driveToPosition(bluTall);
        auto.delay(1000);

        //driveToPosition(bluTallPrep);
        driveToPosition(bluTallPrep180);
        //driveToPosition(bluStackPrep);
        driveToPosition(bluStackAvoid);
        driveToPosition(bluStack);
        auto.delay(1000);

        driveToPosition(bluTallPrep180);
        driveToPosition(bluTall);
        auto.delay(1000);

        //driveToPosition(bluTallPrep);
        driveToPosition(bluTallPrep180);
        //driveToPosition(bluStackPrep);
        driveToPosition(bluStackAvoid);
        driveToPosition(bluStack);
        auto.delay(1000);

        driveToPosition(bluTallPrep180);
        driveToPosition(bluTall);
        auto.delay(1000);

        //driveToPosition(bluTallPrep);
        driveToPosition(bluTallPrep180);
        //driveToPosition(bluStackPrep);
        driveToPosition(bluStackAvoid);
        driveToPosition(bluStack);
        auto.delay(1000);

        driveToPosition(bluTallPrep180);
        driveToPosition(bluTall);
        auto.delay(1000);

        //Park
        driveToPosition(bluTallPrep180);
        driveToPosition(bluPark);

    }

    Position modifyPosition (Position pos) {
        //modify coordinates depending on robot position
        if (isBlueAlliance && isRightSide)   // BLUE-RIGHT
            return pos;
        if (isBlueAlliance && !isRightSide)  // BLUE-LEFT
            return new Position(-pos.X, pos.Y, Functions.normalizeAngle(-pos.R+180));
        if (!isBlueAlliance && !isRightSide) // RED-LEFT
            return new Position(pos.X, -pos.Y, -pos.R);
        if (!isBlueAlliance && isRightSide)  // RED-RIGHT
            return new Position(-pos.X, -pos.Y, Functions.normalizeAngle(pos.R+180));
        return null;
    }

    boolean driveToPosition (DriveToData dTD) {
        //robot.drivetrain.stopDriveMotors();  // for debugging reasons
        //modify coordinates depending on robot position
        Position modPos = modifyPosition(dTD.position);
        //dTD.position = modifyPosition(dTD.position);

        if (!dTD.tileCoord) {
            return auto.driveTo(modPos, dTD.accuracy, dTD.timeout);
        } else {
            return auto.driveToTile(modPos, dTD.accuracy, dTD.timeout);
        }
    }
    boolean driveToPosition (Position pos, int accuracy, long timeout) {
        return driveToPosition(new DriveToData(pos, accuracy, timeout));
    }

    class DriveToData {
        public Position position;
        public int accuracy;
        public long timeout;
        public boolean tileCoord;

        public DriveToData(Position position, int accuracy, long timeout) {
            this.position = position;
            this.accuracy = accuracy;
            this.timeout = timeout;
            this.tileCoord = false;
        }

        public DriveToData(Boolean tileCoord, Position position, int accuracy, long timeout) {
            this.position = position;
            this.accuracy = accuracy;
            this.timeout = timeout;
            this.tileCoord = tileCoord;
        }
    }
}