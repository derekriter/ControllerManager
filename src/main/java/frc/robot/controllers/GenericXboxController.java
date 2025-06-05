package frc.robot.controllers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A controller class for generic Xbox controllers. It is better to use more specific class if a suitable one is available
 */
public class GenericXboxController extends BasicController {
    
    //TODO Verify button and axis ids
    public static final int A = 1;
    public static final int B = 2;
    public static final int X = 3;
    public static final int Y = 4;
    /**Left bumper */
    public static final int LB = 5;
    /**Right bumper */
    public static final int RB = 6;
    /**The button with the two rectangles. Called 'Back' in {@link edu.wpi.first.wpilibj.XboxController.Button} */
    public static final int VIEW = 7;
    /**The button with the three lines. Called 'Start' in {@link edu.wpi.first.wpilibj.XboxController.Button} */
    public static final int MENU = 8;
    /**Left stick. Called 'LeftStick' in {@link edu.wpi.first.wpilibj.XboxController.Button} */
    public static final int L3 = 9;
    /**Right stick. Called 'RightStick' in {@link edu.wpi.first.wpilibj.XboxController.Button} */
    public static final int R3 = 10;
    public static final int LEFT_X = 0;
    public static final int LEFT_Y = 1;
    /**Left trigger */
    public static final int LT = 2;
    /**Right trigger */
    public static final int RT = 3;
    public static final int RIGHT_X = 4;
    public static final int RIGHT_Y = 5;
    
    /**
     * Create a controller from a new HID device
     * @param port Which port to connect the hid device to. Should be in the range of [0, 5]
     * @param robot A reference to a TimedRobot instance
     * @throws IllegalArgumentException If port is not within the allowed range
     * @throws IllegalArgumentException If robot is null
     */
    public GenericXboxController(int port, TimedRobot robot) throws IllegalArgumentException {
        super(port, robot);
    }
    /**
     * Create a controller from an already created HID device
     * @param hid An initialized GenericHID or GenericHID child such as {@link edu.wpi.first.wpilibj.Joystick}
     * @param robot A reference to a TimedRobot instance
     * @throws IllegalArgumentException If hid is null
     * @throws IllegalArgumentException If robot is null
     */
    public GenericXboxController(GenericHID hid, TimedRobot robot) throws IllegalArgumentException {
        super(hid, robot);
    }
    
    public boolean getA() { return getButton(A); }
    public boolean getAPressed() { return getButtonPressed(A); }
    public boolean getAReleased() { return getButtonReleased(A); }
    public Trigger getATrigger() { return getButtonTrigger(A); }
    public boolean getB() { return getButton(B); }
    public boolean getBPressed() { return getButtonPressed(B); }
    public boolean getBReleased() { return getButtonReleased(B); }
    public Trigger getBTrigger() { return getButtonTrigger(B); }
    public boolean getX() { return getButton(X); }
    public boolean getXPressed() { return getButtonPressed(X); }
    public boolean getXReleased() { return getButtonReleased(X); }
    public Trigger getXTrigger() { return getButtonTrigger(X); }
    public boolean getY() { return getButton(Y); }
    public boolean getYPressed() { return getButtonPressed(Y); }
    public boolean getYReleased() { return getButtonReleased(Y); }
    public Trigger getYTrigger() { return getButtonTrigger(Y); }
    public boolean getLB() { return getButton(LB); }
    public boolean getLBPressed() { return getButtonPressed(LB); }
    public boolean getLBReleased() { return getButtonReleased(LB); }
    public Trigger getLBTrigger() { return getButtonTrigger(LB); }
    public boolean getRB() { return getButton(RB); }
    public boolean getRBPressed() { return getButtonPressed(RB); }
    public boolean getRBReleased() { return getButtonReleased(RB); }
    public Trigger getRBTrigger() { return getButtonTrigger(RB); }
    public boolean getView() { return getButton(VIEW); }
    public boolean getViewPressed() { return getButtonPressed(VIEW); }
    public boolean getViewReleased() { return getButtonReleased(VIEW); }
    public Trigger getViewTrigger() { return getButtonTrigger(VIEW); }
    public boolean getMenu() { return getButton(MENU); }
    public boolean getMenuPressed() { return getButtonPressed(MENU); }
    public boolean getMenuReleased() { return getButtonReleased(MENU); }
    public Trigger getMenuTrigger() { return getButtonTrigger(MENU); }
    public boolean getL3() { return getButton(L3); }
    public boolean getL3Pressed() { return getButtonPressed(L3); }
    public boolean getL3Released() { return getButtonReleased(L3); }
    public Trigger getL3Trigger() { return getButtonTrigger(L3); }
    public boolean getR3() { return getButton(R3); }
    public boolean getR3Pressed() { return getButtonPressed(R3); }
    public boolean getR3Released() { return getButtonReleased(R3); }
    public Trigger getR3Trigger() { return getButtonTrigger(R3); }
    
    public double getLeftXRaw() { return getAxisRaw(LEFT_X); }
    public double getLeftXLinear() { return getAxisLinear(LEFT_X); }
    public double getLeftXExponential(double power) { return getAxisExponential(LEFT_X, power); }
    public boolean getLeftXRawGreaterThan(double val) { return getAxisRawGreaterThan(LEFT_X, val); }
    public boolean getLeftXRawLessThan(double val) { return getAxisRawLessThan(LEFT_X, val); }
    public boolean getLeftXLinearGreaterThan(double val) { return getAxisLinearGreaterThan(LEFT_X, val); }
    public boolean getLeftXLinearLessThan(double val) { return getAxisLinearLessThan(LEFT_X, val); }
    public boolean getLeftXExponentialGreaterThan(double power, double val) { return getAxisExponentialGreaterThan(LEFT_X, power, val); }
    public boolean getLeftXExponentialLessThan(double power, double val) { return getAxisExponentialLessThan(LEFT_X, power, val); }
    public Trigger getLeftXRawGreaterThanTrigger(double val) { return getAxisRawGreaterThanTrigger(LEFT_X, val); }
    public Trigger getLeftXRawLessThanTrigger(double val) { return getAxisRawLessThanTrigger(LEFT_X, val); }
    public Trigger getLeftXLinearGreaterThanTrigger(double val) { return getAxisLinearGreaterThanTrigger(LEFT_X, val); }
    public Trigger getLeftXLinearLessThanTrigger(double val) { return getAxisLinearLessThanTrigger(LEFT_X, val); }
    public Trigger getLeftXExponentialGreaterThanTrigger(double power, double val) { return getAxisExponentialGreaterThanTrigger(LEFT_X, power, val); }
    public Trigger getLeftXExponentialLessThanTrigger(double power, double val) { return getAxisExponentialLessThanTrigger(LEFT_X, power, val); }
    public double getLeftYRaw() { return getAxisRaw(LEFT_Y); }
    public double getLeftYLinear() { return getAxisLinear(LEFT_Y); }
    public double getLeftYExponential(double power) { return getAxisExponential(LEFT_Y, power); }
    public boolean getLeftYRawGreaterThan(double val) { return getAxisRawGreaterThan(LEFT_Y, val); }
    public boolean getLeftYRawLessThan(double val) { return getAxisRawLessThan(LEFT_Y, val); }
    public boolean getLeftYLinearGreaterThan(double val) { return getAxisLinearGreaterThan(LEFT_Y, val); }
    public boolean getLeftYLinearLessThan(double val) { return getAxisLinearLessThan(LEFT_Y, val); }
    public boolean getLeftYExponentialGreaterThan(double power, double val) { return getAxisExponentialGreaterThan(LEFT_Y, power, val); }
    public boolean getLeftYExponentialLessThan(double power, double val) { return getAxisExponentialLessThan(LEFT_Y, power, val); }
    public Trigger getLeftYRawGreaterThanTrigger(double val) { return getAxisRawGreaterThanTrigger(LEFT_Y, val); }
    public Trigger getLeftYRawLessThanTrigger(double val) { return getAxisRawLessThanTrigger(LEFT_Y, val); }
    public Trigger getLeftYLinearGreaterThanTrigger(double val) { return getAxisLinearGreaterThanTrigger(LEFT_Y, val); }
    public Trigger getLeftYLinearLessThanTrigger(double val) { return getAxisLinearLessThanTrigger(LEFT_Y, val); }
    public Trigger getLeftYExponentialGreaterThanTrigger(double power, double val) { return getAxisExponentialGreaterThanTrigger(LEFT_Y, power, val); }
    public Trigger getLeftYExponentialLessThanTrigger(double power, double val) { return getAxisExponentialLessThanTrigger(LEFT_Y, power, val); }
    public double getLeftTriggerRaw() { return getAxisRaw(LT); }
    public double getLeftTriggerLinear() { return getAxisLinear(LT); }
    public double getLeftTriggerExponential(double power) { return getAxisExponential(LT, power); }
    public boolean getLeftTriggerRawGreaterThan(double val) { return getAxisRawGreaterThan(LT, val); }
    public boolean getLeftTriggerRawLessThan(double val) { return getAxisRawLessThan(LT, val); }
    public boolean getLeftTriggerLinearGreaterThan(double val) { return getAxisLinearGreaterThan(LT, val); }
    public boolean getLeftTriggerLinearLessThan(double val) { return getAxisLinearLessThan(LT, val); }
    public boolean getLeftTriggerExponentialGreaterThan(double power, double val) { return getAxisExponentialGreaterThan(LT, power, val); }
    public boolean getLeftTriggerExponentialLessThan(double power, double val) { return getAxisExponentialLessThan(LT, power, val); }
    public Trigger getLeftTriggerRawGreaterThanTrigger(double val) { return getAxisRawGreaterThanTrigger(LT, val); }
    public Trigger getLeftTriggerRawLessThanTrigger(double val) { return getAxisRawLessThanTrigger(LT, val); }
    public Trigger getLeftTriggerLinearGreaterThanTrigger(double val) { return getAxisLinearGreaterThanTrigger(LT, val); }
    public Trigger getLeftTriggerLinearLessThanTrigger(double val) { return getAxisLinearLessThanTrigger(LT, val); }
    public Trigger getLeftTriggerExponentialGreaterThanTrigger(double power, double val) { return getAxisExponentialGreaterThanTrigger(LT, power, val); }
    public Trigger getLeftTriggerExponentialLessThanTrigger(double power, double val) { return getAxisExponentialLessThanTrigger(LT, power, val); }
    public double getRightTriggerRaw() { return getAxisRaw(RT); }
    public double getRightTriggerLinear() { return getAxisLinear(RT); }
    public double getRightTriggerExponential(double power) { return getAxisExponential(RT, power); }
    public boolean getRightTriggerRawGreaterThan(double val) { return getAxisRawGreaterThan(RT, val); }
    public boolean getRightTriggerRawLessThan(double val) { return getAxisRawLessThan(RT, val); }
    public boolean getRightTriggerLinearGreaterThan(double val) { return getAxisLinearGreaterThan(RT, val); }
    public boolean getRightTriggerLinearLessThan(double val) { return getAxisLinearLessThan(RT, val); }
    public boolean getRightTriggerExponentialGreaterThan(double power, double val) { return getAxisExponentialGreaterThan(RT, power, val); }
    public boolean getRightTriggerExponentialLessThan(double power, double val) { return getAxisExponentialLessThan(RT, power, val); }
    public Trigger getRightTriggerRawGreaterThanTrigger(double val) { return getAxisRawGreaterThanTrigger(RT, val); }
    public Trigger getRightTriggerRawLessThanTrigger(double val) { return getAxisRawLessThanTrigger(RT, val); }
    public Trigger getRightTriggerLinearGreaterThanTrigger(double val) { return getAxisLinearGreaterThanTrigger(RT, val); }
    public Trigger getRightTriggerLinearLessThanTrigger(double val) { return getAxisLinearLessThanTrigger(RT, val); }
    public Trigger getRightTriggerExponentialGreaterThanTrigger(double power, double val) { return getAxisExponentialGreaterThanTrigger(RT, power, val); }
    public Trigger getRightTriggerExponentialLessThanTrigger(double power, double val) { return getAxisExponentialLessThanTrigger(RT, power, val); }
    public double getRightXRaw() { return getAxisRaw(RIGHT_X); }
    public double getRightXLinear() { return getAxisLinear(RIGHT_X); }
    public double getRightXExponential(double power) { return getAxisExponential(RIGHT_X, power); }
    public boolean getRightXRawGreaterThan(double val) { return getAxisRawGreaterThan(RIGHT_X, val); }
    public boolean getRightXRawLessThan(double val) { return getAxisRawLessThan(RIGHT_X, val); }
    public boolean getRightXLinearGreaterThan(double val) { return getAxisLinearGreaterThan(RIGHT_X, val); }
    public boolean getRightXLinearLessThan(double val) { return getAxisLinearLessThan(RIGHT_X, val); }
    public boolean getRightXExponentialGreaterThan(double power, double val) { return getAxisExponentialGreaterThan(RIGHT_X, power, val); }
    public boolean getRightXExponentialLessThan(double power, double val) { return getAxisExponentialLessThan(RIGHT_X, power, val); }
    public Trigger getRightXRawGreaterThanTrigger(double val) { return getAxisRawGreaterThanTrigger(RIGHT_X, val); }
    public Trigger getRightXRawLessThanTrigger(double val) { return getAxisRawLessThanTrigger(RIGHT_X, val); }
    public Trigger getRightXLinearGreaterThanTrigger(double val) { return getAxisLinearGreaterThanTrigger(RIGHT_X, val); }
    public Trigger getRightXLinearLessThanTrigger(double val) { return getAxisLinearLessThanTrigger(RIGHT_X, val); }
    public Trigger getRightXExponentialGreaterThanTrigger(double power, double val) { return getAxisExponentialGreaterThanTrigger(RIGHT_X, power, val); }
    public Trigger getRightXExponentialLessThanTrigger(double power, double val) { return getAxisExponentialLessThanTrigger(RIGHT_X, power, val); }
    public double getRightYRaw() { return getAxisRaw(RIGHT_Y); }
    public double getRightYLinear() { return getAxisLinear(RIGHT_Y); }
    public double getRightYExponential(double power) { return getAxisExponential(RIGHT_Y, power); }
    public boolean getRightYRawGreaterThan(double val) { return getAxisRawGreaterThan(RIGHT_Y, val); }
    public boolean getRightYRawLessThan(double val) { return getAxisRawLessThan(RIGHT_Y, val); }
    public boolean getRightYLinearGreaterThan(double val) { return getAxisLinearGreaterThan(RIGHT_Y, val); }
    public boolean getRightYLinearLessThan(double val) { return getAxisLinearLessThan(RIGHT_Y, val); }
    public boolean getRightYExponentialGreaterThan(double power, double val) { return getAxisExponentialGreaterThan(RIGHT_Y, power, val); }
    public boolean getRightYExponentialLessThan(double power, double val) { return getAxisExponentialLessThan(RIGHT_Y, power, val); }
    public Trigger getRightYRawGreaterThanTrigger(double val) { return getAxisRawGreaterThanTrigger(RIGHT_Y, val); }
    public Trigger getRightYRawLessThanTrigger(double val) { return getAxisRawLessThanTrigger(RIGHT_Y, val); }
    public Trigger getRightYLinearGreaterThanTrigger(double val) { return getAxisLinearGreaterThanTrigger(RIGHT_Y, val); }
    public Trigger getRightYLinearLessThanTrigger(double val) { return getAxisLinearLessThanTrigger(RIGHT_Y, val); }
    public Trigger getRightYExponentialGreaterThanTrigger(double power, double val) { return getAxisExponentialGreaterThanTrigger(RIGHT_Y, power, val); }
    public Trigger getRightYExponentialLessThanTrigger(double power, double val) { return getAxisExponentialLessThanTrigger(RIGHT_Y, power, val); }
}
