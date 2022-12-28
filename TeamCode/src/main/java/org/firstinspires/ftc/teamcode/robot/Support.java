package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Support {

   private static ElapsedTime timerLoop = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
   private static double timeLoop;

   // Hypotenuse function I used in Blocky?
   public static double mathHypotenuse(float arg0, float arg1) {
      return Math.sqrt(Math.pow(arg0, 2) + Math.pow(arg1, 2));
   }

   public static double normalizeAngle(double A) {
      // normalize angle A to -179 to +180 range
      while (A > 180) A -= 360;
      while (A <= -180) A += 360;
      return A;
   }

   public static double calculateLoopTime() {
      timeLoop = timerLoop.milliseconds();
      timerLoop.reset();
      return timeLoop;
   }

}

// testing - brought this in from Om's old code
class Position
{
   double X, Y, R;

   Position(double X, double Y, double R){
      this.X = X;
      this.Y = Y;
      this.R = R;
   }

   Position(){
      X = 0;
      Y = 0;
      R = 0;
   }

   Position switchXY(){
      return new Position(Y, X, R);
   }

   public String toString(int decimals){
      Position pos = round(decimals);
      return "X: " + pos.X + ", Y: " + pos.Y + ", R: " + pos.R;
   }

   public Position clone(){ return new Position(X, Y, R);}

   void add(Position pos2){
      X += pos2.X;
      Y += pos2.Y;
      R += pos2.R;
   }

   void subtract(Position pos2){
      X -= pos2.X;
      Y -= pos2.Y;
      R -= pos2.R;
   }

   void divide(double divisor){
      X /= divisor;
      Y /= divisor;
      R /= divisor;
   }

   void abs(){
      X = Math.abs(X);
      Y = Math.abs(Y);
      R = Math.abs(R);
   }

   Position round(int decimals){
      return new Position(
              Math.round(X * Math.pow(10, decimals))/ Math.pow(10, decimals),
              Math.round(Y * Math.pow(10, decimals))/ Math.pow(10, decimals),
              Math.round(R * Math.pow(10, decimals))/ Math.pow(10, decimals)
      );
   }

   Position getAbsDiff(Position pos2){
      Position diff = this.clone();
      diff.subtract(pos2);
      diff.abs();
      return diff;
   }

   boolean isPositionInRange(Position pos2, Position maxDiff){
      Position diff = getAbsDiff(pos2);
      return diff.X < maxDiff.X && diff.Y < maxDiff.Y && diff.R < maxDiff.R;
   }

}

class NavActions {
   Actions action;
   double parameter1;
   double parameter2;
   double parameter3;

   public NavActions(Actions a, double x, double y, double r) {
      action = a;
      parameter1 = x;
      parameter2 = y;
      parameter3 = r;
   }
   public NavActions(Actions a) {
      action = a;
      parameter1 = 0;
      parameter2 = 0;
      parameter3 = 0;
   }
   public NavActions(Actions a, double p) {
      action = a;
      parameter1 = p;
      parameter2 = 0;
      parameter3 = 0;
   }
}

enum Actions {
   MOVEACCURATE,
   MOVETRANSITION,
   PAUSE,
   ENDROUTINE
}

