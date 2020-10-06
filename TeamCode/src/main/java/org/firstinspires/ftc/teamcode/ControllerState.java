package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class ControllerState
{
    private float
            rightStickX, rightStickY,
            leftStickX, leftStickY,
            leftTrigger, rightTrigger;
    private boolean
            dpadUp, dpadDown, dpadRight, dpadLeft,
            a, b, x, y,
            leftBumper, rightBumper,
            rsButton, lsButton;

    public ControllerState (Gamepad gp)
    {
        rightStickX = gp.right_stick_x;
        rightStickY = gp.right_stick_y;
        leftStickX = gp.left_stick_x;
        leftStickY = gp.left_stick_y;

        leftTrigger = gp.left_trigger;
        rightTrigger = gp.right_trigger;

        dpadDown = gp.dpad_down;
        dpadLeft = gp.dpad_left;
        dpadRight = gp.dpad_right;
        dpadUp = gp.dpad_up;

        a = gp.a;
        b = gp.b;
        x = gp.x;
        y = gp.y;

        leftBumper = gp.left_bumper;
        rightBumper = gp.right_bumper;

        rsButton = gp.right_stick_button;
        lsButton = gp.left_stick_button;
    }

    public ControllerState (float right_stick_x, float right_stick_y,
                            float left_stick_x, float left_stick_y,
                            float left_trigger, float right_trigger,
                            boolean dpad_down, boolean dpad_left,
                            boolean dpad_right, boolean dpad_up,
                            boolean a, boolean b, boolean x, boolean y,
                            boolean left_bumper, boolean right_bumper,
                            boolean right_stick_button, boolean left_stick_button)
    {
        rightStickX = right_stick_x;
        rightStickY = right_stick_y;
        leftStickX = left_stick_x;
        leftStickY = left_stick_y;

        leftTrigger = left_trigger;
        rightTrigger = right_trigger;

        dpadDown = dpad_down;
        dpadLeft = dpad_left;
        dpadRight = dpad_right;
        dpadUp = dpad_up;

        this.a = a;
        this.b = b;
        this.x = x;
        this.y = y;

        leftBumper = left_bumper;
        rightBumper = right_bumper;

        rsButton = right_stick_button;
        lsButton = left_stick_button;
    }

    public float getRightStickX()
    {
        return rightStickX;
    }

    public float getLeftStickX() {
        return leftStickX;
    }

    public float getLeftStickY() {
        return leftStickY;
    }

    public float getRightStickY() {
        return rightStickY;
    }

    public float getLeftTrigger() {
        return leftTrigger;
    }

    public float getRightTrigger() {
        return rightTrigger;
    }

    public boolean isDpadDown() {
        return dpadDown;
    }

    public boolean isDpadLeft() {
        return dpadLeft;
    }

    public boolean isDpadRight() {
        return dpadRight;
    }

    public boolean isDpadUp() {
        return dpadUp;
    }

    public boolean isA() {
        return a;
    }

    public boolean isX() {
        return x;
    }

    public boolean isY() {
        return y;
    }

    public boolean isB() {
        return b;
    }

    public boolean isLeftBumper () {
        return leftBumper;
    }

    public boolean isRightBumper() {
        return rightBumper;
    }

    public boolean isLsButton() {
        return lsButton;
    }

    public boolean isRsButton() {
        return rsButton;
    }

    @Override
    public String toString() {
        String out =
                rightStickX + " " + rightStickY + " " +
                leftStickX + " " + leftStickY + " " +
                leftTrigger + " " + rightTrigger +
                boo(dpadUp) + boo(dpadDown) + boo(dpadRight) + boo(dpadLeft) +
                boo(a) + boo(b) + boo(x) + boo(y) +
                boo(leftBumper) + boo(rightBumper) +
                boo(rsButton) + boo(lsButton) + "\n";
        return out;
    }

    private String boo (boolean in)
    {
        return (" " + (in? "true" : "false"));
    }
}
