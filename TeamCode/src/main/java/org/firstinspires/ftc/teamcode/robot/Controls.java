package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

public class Controls {

//   LinearOpMode opMode;
//   HardwareMap hardwareMap;
   Gamepad gamepad1;
   Gamepad gamepad2;
//   Telemetry telemetry;
   Robot robot;
   ButtonMgr buttonMgr;
   Navigator navigator;
   public double DriveSpeed, DriveAngle, Rotate;

   public final double tileSize = 23.5;  //in inches

   /* Constructor */
   public Controls(Robot robot){//ButtonMgr buttonMgr, Navigator navigator){
      construct(robot);//buttonMgr, navigator);
   }

   void construct(Robot robot){//ButtonMgr buttonMgr, Navigator navigator){
//      this.opMode = opMode;
//      this.hardwareMap = opMode.hardwareMap;

//      this.telemetry = opMode.telemetry;
      this.robot = robot;
      this.gamepad1 = robot.opMode.gamepad1;
      this.gamepad2 = robot.opMode.gamepad2;
      this.buttonMgr = robot.buttonMgr;
      this.navigator = robot.navigator;
   }

   void init() {

   }

   public void loop() {
      readAndAct();
      navigator.setUserDriveSettings(DriveSpeed, DriveAngle, Rotate);
   }

   public void readAndAct() {

      // TeleOp / normal drive

      // Get speed and direction from left stick
      DriveSpeed = JavaUtil.minOfList(JavaUtil.createListWith(1, Support.mathHypotenuse(gamepad1.left_stick_x, gamepad1.left_stick_y)));
      DriveAngle = Math.atan2(-gamepad1.left_stick_x, -gamepad1.left_stick_y) / Math.PI * 180;
      // Get rotation from right stick
      Rotate = Math.pow(gamepad1.right_stick_x, 1);
      navigator.handleRotate(Rotate);

      // Toggle FCD
      if (buttonMgr.wasTapped(1, ButtonMgr.Buttons.START))
         navigator.toggleFieldCentricDrive();

      // Toggle HeadingHold
      if (buttonMgr.wasTapped(1, ButtonMgr.Buttons.BACK))
         navigator.toggleHeadingHold();

      // Store heading correction
      if (buttonMgr.wasReleased(1, ButtonMgr.Buttons.rightJoyStickBUTTON))
         navigator.setDeltaHeading();

      // This blob is for manually entering destinations by adjusting X, Y, Rot
      if (buttonMgr.wasTapped(1, ButtonMgr.Buttons.dpadUP))
         navigator.setTargetByDeltaRelative(2,0,0);
      if (buttonMgr.wasTapped(1, ButtonMgr.Buttons.dpadDOWN))
         navigator.setTargetByDeltaRelative(-2,0,0);
      if (buttonMgr.wasTapped(1, ButtonMgr.Buttons.dpadLEFT))
         navigator.setTargetByDeltaRelative(0, 2,0);
      if (buttonMgr.wasTapped(1, ButtonMgr.Buttons.dpadRIGHT))
         navigator.setTargetByDeltaRelative(0, -2,0);
      if (buttonMgr.wasTapped(1, ButtonMgr.Buttons.X))
         navigator.setTargetByDeltaRelative(0,0,10);
      if (buttonMgr.wasTapped(1, ButtonMgr.Buttons.Y))
         navigator.setTargetByDeltaRelative(0,0,-10);
      if (buttonMgr.wasTapped(1, ButtonMgr.Buttons.A)) {
//         navigator.setTargetToCurrentPosition();
//         navigator.beginAutoDrive();
//         navigator.togglePositionHold();
      }
      if (buttonMgr.wasTapped(1, ButtonMgr.Buttons.B))
         //navigator.cancelAutoNavigation();
         navigator.togglePositionHold();
      if (!buttonMgr.isPressed(1, ButtonMgr.Buttons.rightBUMPER))
         navigator.setMaxSpeed(1);
      else
         navigator.setMaxSpeed(0.25);
      if (buttonMgr.wasTapped(1, ButtonMgr.Buttons.leftBUMPER))
         robot.sensors.readDistSensors();

      // AutoDrive Testing

      // Start auto navigation
      if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.START))
         navigator.beginAutoDrive();

      // Set to home position
      if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.rightBUMPER))
         navigator.setTargetToZeroPosition();

      // Begin scripted navigation
      if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.leftBUMPER))
         navigator.beginScriptedNav();

      // Cancels auto navigation
      if (buttonMgr.isPressed(2, ButtonMgr.Buttons.B))
         navigator.cancelAutoNavigation();

      // Reset target navigation to present position
      if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.A))
         navigator.setTargetToCurrentPosition();

      // This blob is for manually entering destinations by adjusting X, Y, Rot
      if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.dpadUP))
         navigator.setTargetByDelta((gamepad2.back ? 1 : tileSize/2.0),0,0);
      if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.dpadDOWN))
         navigator.setTargetByDelta(-(gamepad2.back ? 1 : tileSize/2.0),0,0);
      if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.dpadLEFT))
         navigator.setTargetByDelta(0, (gamepad2.back ? 1 : tileSize/2.0),0);
      if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.dpadRIGHT))
         navigator.setTargetByDelta(0, -(gamepad2.back ? 1 : tileSize/2.0),0);
      if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.X))
         navigator.setTargetByDelta(0,0,(gamepad2.back ? 2 : 45));
      if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.Y))
         navigator.setTargetByDelta(0,0,-(gamepad2.back ? 2 : 45));

   }
}
