package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Prototype {

   LinearOpMode opMode;
   HardwareMap hardwareMap;
   Gamepad gamepad1;
   Gamepad gamepad2;
   Telemetry telemetry;
   Robot robot;

   /* Constructor */
   public Prototype(LinearOpMode opMode, Robot robot){
      construct(opMode, robot);
   }

   void construct(LinearOpMode opMode, Robot robot){
      this.opMode = opMode;
      this.hardwareMap = opMode.hardwareMap;
      this.gamepad1 = opMode.gamepad1;
      this.gamepad2 = opMode.gamepad2;
      this.telemetry = opMode.telemetry;
      this.robot = robot;
   }
}
