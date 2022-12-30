package org.firstinspires.ftc.teamcode.robot.Tools;

// testing - brought this in from Om's old code
public class Position
{
    public double X, Y, R;

    public Position(double X, double Y, double R){
        this.X = X;
        this.Y = Y;
        this.R = R;
    }

    public Position(){
        X = 0;
        Y = 0;
        R = 0;
    }

    public Position switchXY(){
        return new Position(Y, X, R);
    }

    public String toString(int decimals){
        Position pos = round(decimals);
        return "X: " + pos.X + ", Y: " + pos.Y + ", R: " + pos.R;
    }

    public Position clone(){ return new Position(X, Y, R);}

    public void add(Position pos2){
        X += pos2.X;
        Y += pos2.Y;
        R += pos2.R;
    }

    public void subtract(Position pos2){
        X -= pos2.X;
        Y -= pos2.Y;
        R -= pos2.R;
    }

    public void divide(double divisor){
        X /= divisor;
        Y /= divisor;
        R /= divisor;
    }

    public void abs(){
        X = Math.abs(X);
        Y = Math.abs(Y);
        R = Math.abs(R);
    }

    public Position round(int decimals){
        return new Position(
                Math.round(X * Math.pow(10, decimals))/ Math.pow(10, decimals),
                Math.round(Y * Math.pow(10, decimals))/ Math.pow(10, decimals),
                Math.round(R * Math.pow(10, decimals))/ Math.pow(10, decimals)
        );
    }

    public Position getAbsDiff(Position pos2){
        Position diff = this.clone();
        diff.subtract(pos2);
        diff.abs();
        return diff;
    }

    public boolean isPositionInRange(Position pos2, Position maxDiff){
        Position diff = getAbsDiff(pos2);
        return diff.X < maxDiff.X && diff.Y < maxDiff.Y && diff.R < maxDiff.R;
    }

}