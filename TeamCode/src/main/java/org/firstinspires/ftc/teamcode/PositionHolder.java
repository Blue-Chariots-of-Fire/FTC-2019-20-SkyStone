package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.Position;

public class PositionHolder
{
    private double x, y, angle;

    public PositionHolder ()
    {
        x = 0;
        y = 0;
        angle = 0;
    }

    public PositionHolder (double x, double y, double angle)
    {
        this.x = x;
        this.y = y;
        this.angle = angle;
    }

    public double getX () {return x;}

    public double getY () {return y;}

    public double getAngle () {return angle;}

    public void setX(double x) {this.x = x;}

    public void setY(double y) {this.y = y;}

    public void setAngle(double angle) {this.angle = angle;}

    public void add (double x, double y, double angle)
    {
        this.x += x;
        this.y += y;
        this.angle += angle;
    }

    public void addX (double x) {this.x += x;}

    public void addY (double y) {this.y += y;}

    public void addAngle (double angle) {this.angle += angle;}

    @Override
    public String toString() {return "x: " + x + ", y: " + y + ", angle: " + angle;}

    public boolean equals (PositionHolder other) {
        return other.getX() == x && other.getY() == y && other.getAngle() == angle;
    }
}
