package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.robot.Tools.Functions;

public class LiftbotControls {

   Gamepad gamepad1;
   Gamepad gamepad2;
   Robot robot;
   ButtonMgr buttonMgr;
   Navigator2 navigator;
   LiftbotLifter lifter;
   public double DriveSpeed, DriveAngle, Rotate, liftSpeed;

   public final double tileSize = 23.5;  //in inches

   /* Constructor */
   public LiftbotControls(Robot robot, LiftbotLifter lifter){//ButtonMgr buttonMgr, Navigator navigator){
      construct(robot, lifter);//buttonMgr, navigator);
   }

   void construct(Robot robot, LiftbotLifter lifter){//ButtonMgr buttonMgr, Navigator navigator){
      this.robot = robot;
      this.gamepad1 = robot.opMode.gamepad1;
      this.gamepad2 = robot.opMode.gamepad2;
      this.buttonMgr = robot.buttonMgr;
      this.navigator = robot.navigator;
      this.lifter = lifter;
   }

   void init() {

   }

   public void loop() {
      readAndAct();
      navigator.setUserDriveSettings(DriveSpeed, DriveAngle, Rotate);
      lifter.setUserDriveSettings(liftSpeed);
   }

   public void readAndAct() {

      // TeleOp / normal drive

      // Get speed and direction from left stick
      DriveSpeed = JavaUtil.minOfList(JavaUtil.createListWith(1, Functions.mathHypotenuse(gamepad1.left_stick_x, gamepad1.left_stick_y)));
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

      if (buttonMgr.wasReleased(1, ButtonMgr.Buttons.leftJoyStickBUTTON))
         navigator.toggleSnapToAngle();

      // This blob is for manually entering destinations by adjusting X, Y, Rot
      if (buttonMgr.wasTapped(1, ButtonMgr.Buttons.dpadUP))
         navigator.setTargetByDeltaRelative(2,0,0);
      if (buttonMgr.wasTapped(1, ButtonMgr.Buttons.dpadDOWN))
         navigator.setTargetByDeltaRelative(-2,0,0);
      if (buttonMgr.wasTapped(1, ButtonMgr.Buttons.dpadLEFT))
         navigator.setTargetByDeltaRelative(0, 2,0);
      if (buttonMgr.wasTapped(1, ButtonMgr.Buttons.dpadRIGHT))
         navigator.setTargetByDeltaRelative(0, -2,0);
      if (buttonMgr.wasTapped(1, ButtonMgr.Buttons.X)) {
//         navigator.setTargetByDeltaRelative(0, 0, 45);
         navigator.setTargetRotBySnapRelative (45);
         navigator.headingDelay = System.currentTimeMillis() + 500;  // workaround
      }
      if (buttonMgr.wasTapped(1, ButtonMgr.Buttons.Y)) {
//         navigator.setTargetByDeltaRelative(0, 0, -45);
         navigator.setTargetRotBySnapRelative (-45);
         navigator.headingDelay = System.currentTimeMillis() + 500;  // workaround
      }
//      if (buttonMgr.wasTapped(1, ButtonMgr.Buttons.A)) {
////         navigator.setTargetToCurrentPosition();
////         navigator.beginAutoDrive();
////         navigator.togglePositionHold();
//         navigator.useAutoDistanceActivation = !navigator.useAutoDistanceActivation;
//      }
//      if (buttonMgr.wasTapped(1, ButtonMgr.Buttons.B))
//         //navigator.cancelAutoNavigation();
//         navigator.togglePositionHold();
      if (!buttonMgr.isPressed(1, ButtonMgr.Buttons.rightBUMPER))
         navigator.setMaxSpeed(1);
      else
         navigator.setMaxSpeed(0.25);
//      if (buttonMgr.wasTapped(1, ButtonMgr.Buttons.leftBUMPER))
//         robot.sensors.readDistSensors();

      // AutoDrive Testing

      liftSpeed = -gamepad2.left_stick_y;

      if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.dpadLEFT))
         lifter.action(LiftbotLifter.LiftActions.AUTOMATE_PREP_CAPTURE);

      if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.dpadDOWN))
         lifter.action(LiftbotLifter.LiftActions.AUTOMATE_PREP_DEPOSIT_LO);

      if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.dpadRIGHT))
         lifter.action(LiftbotLifter.LiftActions.AUTOMATE_PREP_DEPOSIT_MED);

      if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.dpadUP))
         lifter.action(LiftbotLifter.LiftActions.AUTOMATE_PREP_DEPOSIT_HI);

      if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.BACK))
         lifter.action(LiftbotLifter.LiftActions.CANCEL);

      if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.A))
         lifter.action(LiftbotLifter.LiftActions.AUTOMATE_CAPTURE);

      if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.B))
         lifter.action(LiftbotLifter.LiftActions.GRAB_TOGGLE);

      if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.Y))
         lifter.action(LiftbotLifter.LiftActions.LIFT_LOWER_TO_DEPOSIT);

      if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.X))
         lifter.action(LiftbotLifter.LiftActions.LIFT_RAISE_AFTER_CAPTURE);

      if (buttonMgr.isHeld(2, ButtonMgr.Buttons.leftBUMPER) &&
              buttonMgr.wasTapped(2, ButtonMgr.Buttons.rightBUMPER))
         lifter.action(LiftbotLifter.LiftActions.AUTOMATE_HOME);

      if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.rightBUMPER)) {
         if (buttonMgr.isHeld(2, ButtonMgr.Buttons.leftBUMPER)) {
            lifter.action(LiftbotLifter.LiftActions.AUTOMATE_HOME);
         } else {
            // reserved for future use
         }
      }

      if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.leftBUMPER))
         lifter.action(LiftbotLifter.LiftActions.LIFT_DOWNSTACK);


//      if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.X))
//         navigator.setTargetByDelta(0,0,(gamepad2.back ? 2 : 45));
//      if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.Y))
//         navigator.setTargetByDelta(0,0,-(gamepad2.back ? 2 : 45));

   }
}
