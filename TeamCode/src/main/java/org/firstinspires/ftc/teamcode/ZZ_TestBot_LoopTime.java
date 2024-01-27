package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.ButtonMgr;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Tools.QwiicLEDStick;

import androidx.annotation.ColorInt;

@TeleOp (name="ZZ_TestBot_LoopTime", group="Test")
//@Disabled
public class ZZ_TestBot_LoopTime extends LinearOpMode {

//    Robot robot   = new Robot(this);
//    ButtonMgr buttonMgr = new ButtonMgr(this);
//    Orientation angles;

    Robot robot;
    ButtonMgr buttonMgr;


    int loopySample = 50;
    double[] loopyTime = new double[loopySample];
    int loopyTimeCounter = 0;

    int motorMode = 0;
    int servoMode = 0;
    boolean IMUmode = false;
    boolean readColorSensor = false;
    boolean readColorSensorDistance = false;
    boolean UpdateLED = false;
    boolean readEncoders = false;
    boolean hub1 = false;
    boolean hub2 = false;
    int hubMode = 0;

    NormalizedColorSensor colorSensor;
    QwiicLEDStick qled = null;

    @Override
    public void runOpMode() {
        robot = new Robot(this);
        buttonMgr = new ButtonMgr(this);
        robot.init();

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        qled = hardwareMap.get(QwiicLEDStick.class, "led");


        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        robot.motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor0B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor1B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor2B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor3B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


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
        robot.motor0B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motor1B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motor2B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motor3B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motor0B.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motor1B.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motor2B.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motor3B.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            //angles = robot.sensorIMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData(">", "Robot Heading = %.1f", robot.returnImuHeading(true));
            telemetry.update();
            sleep(100);
        }

        ElapsedTime loopElapsedTime = new ElapsedTime();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            robot.loop();  // this will take care of clearing out the bulk reads
            buttonMgr.updateAll();

            if(buttonMgr.wasTapped(1, ButtonMgr.Buttons.A)) {
                if(buttonMgr.isPressed(1, ButtonMgr.Buttons.leftBUMPER)) motorMode = 2;
                else motorMode = 1;
            }

            if(buttonMgr.wasTapped(1, ButtonMgr.Buttons.B)) {
                motorMode = 0;
                stopAllMotors();
            }

            if(buttonMgr.wasTapped(1, ButtonMgr.Buttons.X)) {
                if(buttonMgr.isPressed(1, ButtonMgr.Buttons.leftBUMPER)) servoMode = 2;
                else servoMode = 1;
            }

            if(buttonMgr.wasTapped(1, ButtonMgr.Buttons.Y)) {
                servoMode = 0;
                centerAllServos();
            }

            if(buttonMgr.wasTapped(1, ButtonMgr.Buttons.dpadLEFT)) {
                robot.disableIMUupdate = IMUmode;
                IMUmode = !IMUmode;
            }

            if(buttonMgr.wasTapped(1, ButtonMgr.Buttons.dpadRIGHT)) {
                UpdateLED = !UpdateLED;
                if (!UpdateLED) qled.turnAllOff();
            }

            if(buttonMgr.wasTapped(1, ButtonMgr.Buttons.dpadUP)) {
                readColorSensor = !readColorSensor;
            }

            if(buttonMgr.wasTapped(1, ButtonMgr.Buttons.dpadDOWN)) {
                readColorSensorDistance = !readColorSensorDistance;
            }

            if(buttonMgr.wasTapped(1, ButtonMgr.Buttons.rightBUMPER)) {
                readEncoders = !readEncoders;
            }

            if(buttonMgr.wasTapped(1, ButtonMgr.Buttons.BACK)) {
                hubMode = (hubMode==3) ? 0 : hubMode+1;
            }

            switch (hubMode) {
                case 1:
                case 3:
                    hub1 = true;
                    break;
                case 2:
                    hub1 = false;
                    hub2 = true;
                    break;
                default:
                    hub1 = false;
                    hub2 = false;
            }

            if (readEncoders) readAllMotors();

            if (servoMode==1) setAllServos(1);
            if (motorMode==1) setAllMotors(0.1);

            if (servoMode==2) setAllServos(Math.random()*.2+.5);
            if (motorMode==2) setAllMotors(Math.random()*.1);

            if (readColorSensor) {
                //double distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
                NormalizedRGBA colors = colorSensor.getNormalizedColors();
            }

            if (readColorSensorDistance) {
                double distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
                //NormalizedRGBA colors = colorSensor.getNormalizedColors();
            }

            if (UpdateLED) {
                qled.setColorGroup(0,5, Color.rgb(5,5,5));
            }


            // If needed, could disable the servo signal with Servo.getController().pwmDisable()

            loopyTime[loopyTimeCounter]=loopElapsedTime.milliseconds();
            loopyTimeCounter++;
            if (loopyTimeCounter >= loopySample) loopyTimeCounter = 0;
            double loopyTimeAverage = 0;
            for(int i=0; i<loopySample; i++) loopyTimeAverage+=loopyTime[i];
            loopyTimeAverage /= loopySample;

            telemetry.addData("HUB", (hub1 ? "HUB1 " : "") + (hub2 ? "HUB2" : ""));
            telemetry.addData("IMU read",IMUmode);
            telemetry.addData("CS read", readColorSensor);
            telemetry.addData("CSD read", readColorSensorDistance);
            telemetry.addData("Encoders read", readEncoders);
            telemetry.addData("LED update", UpdateLED);

            telemetry.addData("Heading", "%.1f", robot.returnImuHeading());
            telemetry.addData("LoopTime(ms)","%.1f",loopElapsedTime.milliseconds());
            telemetry.addData("LoopTimeAvg10(ms)","%.1f",loopyTimeAverage);
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
        robot.motor0B.setPower(0);
        robot.motor1B.setPower(0);
        robot.motor2B.setPower(0);
        robot.motor3B.setPower(0);
    }

    public void setAllMotors (double speed) {
        if (hub1) {
            robot.motor0.setPower(speed);
            robot.motor1.setPower(speed);
            robot.motor2.setPower(speed);
            robot.motor3.setPower(speed);
        }
        if (hub2) {
            robot.motor0B.setPower(speed);
            robot.motor1B.setPower(speed);
            robot.motor2B.setPower(speed);
            robot.motor3B.setPower(speed);
        }
    }

    public void centerAllServos () {
        robot.servo0.setPosition(0.5);
        robot.servo1.setPosition(0.5);
        robot.servo2.setPosition(0.5);
        robot.servo3.setPosition(0.5);
        robot.servo4.setPosition(0.5);
        robot.servo5.setPosition(0.5);
        robot.servo0B.setPosition(0.5);
        robot.servo1B.setPosition(0.5);
        robot.servo2B.setPosition(0.5);
        robot.servo3B.setPosition(0.5);
        robot.servo4B.setPosition(0.5);
        robot.servo5B.setPosition(0.5);
    }

    public void setAllServos (double position) {
        if (hub1) {
            robot.servo0.setPosition(position);
            robot.servo1.setPosition(position);
            robot.servo2.setPosition(position);
            robot.servo3.setPosition(position);
            robot.servo4.setPosition(position);
            robot.servo5.setPosition(position);
        }
        if (hub2) {
            robot.servo1B.setPosition(position);
            robot.servo2B.setPosition(position);
            robot.servo3B.setPosition(position);
            robot.servo4B.setPosition(position);
            robot.servo5B.setPosition(position);
            robot.servo0B.setPosition(position);
        }
    }

    public int readAllMotors () {
        int i0, i1, i2, i3, i0B, i1B, i2B, i3B;  // Want to make absolutely certain the compiler doesn't optimize the reads out
        i0 = i1 = i2 = i3 = i0B = i1B = i2B = i3B = 0;
        if (hub1) {
            i0 = robot.motor0.getCurrentPosition();
            i1 = robot.motor1.getCurrentPosition();
            i2 = robot.motor2.getCurrentPosition();
            i3 = robot.motor3.getCurrentPosition();
        }
        if (hub2) {
            i0B = robot.motor0B.getCurrentPosition();
            i1B = robot.motor1B.getCurrentPosition();
            i2B = robot.motor2B.getCurrentPosition();
            i3B = robot.motor3B.getCurrentPosition();
        }
        return i0+i1+i2+i3+i0B+i1B+i2B+i3B;
    }
}