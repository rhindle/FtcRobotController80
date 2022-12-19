package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drivetrain {

    LinearOpMode opMode;
//    HardwareMap hardwareMap;
//    Gamepad gamepad1;
//    Gamepad gamepad2;
    Telemetry telemetry;
    Robot robot;

    private DcMotorEx motorLF, motorRF, motorLR, motorRR;

    /* Constructor */
    public Drivetrain(LinearOpMode opMode, Robot robot){
        construct(opMode, robot);
    }

    void construct(LinearOpMode opMode, Robot robot){
        this.opMode = opMode;
//        this.hardwareMap = opMode.hardwareMap;
//        this.gamepad1 = opMode.gamepad1;
//        this.gamepad2 = opMode.gamepad2;
        this.telemetry = opMode.telemetry;
        this.robot = robot;
    }

    public void init() {
        motorLF = robot.motor0;
        motorRF = robot.motor1;
        motorLR = robot.motor2;
        motorRR = robot.motor3;
        initMotors();
    }

    public void setDrivePowers (double m0, double m1, double m2, double m3) {
        motorLF.setPower(m0);
        motorRF.setPower(m1);
        motorLR.setPower(m2);
        motorRR.setPower(m3);
    }

    public void stopDriveMotors() {
        motorLF.setPower(0);
        motorRF.setPower(0);
        motorLR.setPower(0);
        motorRR.setPower(0);
    }

    public void initMotors () {
        motorLF.setDirection(DcMotorEx.Direction.REVERSE);
        motorLR.setDirection(DcMotorEx.Direction.REVERSE);
        motorRF.setDirection(DcMotorEx.Direction.FORWARD);
        motorRR.setDirection(DcMotorEx.Direction.FORWARD);

        motorLF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorLR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorRF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorRR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        motorLF.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorLR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorRF.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorRR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        motorLF.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motorLR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motorRF.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motorRR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }
}
