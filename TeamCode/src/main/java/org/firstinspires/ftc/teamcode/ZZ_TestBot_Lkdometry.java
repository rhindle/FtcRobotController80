package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.ButtonMgr;
import org.firstinspires.ftc.teamcode.robot.Localizer2;
import org.firstinspires.ftc.teamcode.robot.Om.Vector3;
import org.firstinspires.ftc.teamcode.robot.Omdometry;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Tools.Position;

@TeleOp (name="ZZ_TestBot_Lkdometry", group="Test")
//@Disabled
public class ZZ_TestBot_Lkdometry extends LinearOpMode {

//    Robot robot   = new Robot(this);
//    ButtonMgr buttonMgr = new ButtonMgr(this);
//    Orientation angles;

    Robot robot;
    ButtonMgr buttonMgr;
    Localizer2 odo;

    int loopySample = 50;
    double[] loopyTime = new double[loopySample];
    int loopyTimeCounter = 0;

    @Override
    public void runOpMode() {
        robot = new Robot(this);
        buttonMgr = new ButtonMgr(this);
        odo = new Localizer2(robot);
        robot.init();
        //odo.odoFieldStart = new Position(-1.5 * 23.5,-62,-90);
        //odo.odoFieldStart = new Position (-36,63,-90);
        odo.odoFieldStart = new Position(0,0,-90);
        odo.init();

        //colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        //qled = hardwareMap.get(QwiicLEDStick.class, "led");

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

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            //angles = robot.sensorIMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData(">", "Robot Heading = %.1f", robot.returnImuHeading(true));
            telemetry.update();
            sleep(100);
        }

        robot.motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        ElapsedTime loopElapsedTime = new ElapsedTime();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            robot.loop();  // this will take care of clearing out the bulk reads
            buttonMgr.updateAll();
            odo.loop();

            loopyTime[loopyTimeCounter]=loopElapsedTime.milliseconds();
            loopyTimeCounter++;
            if (loopyTimeCounter >= loopySample) loopyTimeCounter = 0;
            double loopyTimeAverage = 0;
            for(int i=0; i<loopySample; i++) loopyTimeAverage+=loopyTime[i];
            loopyTimeAverage /= loopySample;

            telemetry.addData ("OdoY", odo.encoderY);
            telemetry.addData ("OdoXL", odo.encoderXL);
            telemetry.addData ("OdoXR", odo.encoderXR);
            //telemetry.addData("position", odo.robotPosition);
            odo.addTeleOpTelemetry();

            telemetry.addData("Heading", "%.1f", robot.returnImuHeading());
            telemetry.addData("LoopTime(ms)","%.1f",loopElapsedTime.milliseconds());
            telemetry.addData("LoopTimeAvg10(ms)","%.1f",loopyTimeAverage);
            telemetry.addData("LoopSpeed(lps)","%.1f",1/(loopElapsedTime.milliseconds()/1000));
            loopElapsedTime.reset();
            telemetry.update();
        }
    }
}