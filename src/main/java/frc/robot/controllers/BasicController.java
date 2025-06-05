package frc.robot.controllers;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/*
 * TODO Javadocs on github?
 */
/**
 * A basic controller with no predefined button or axis IDs. It is recommended to use a more specific class if a suitable one is available
 */
public class BasicController {
    
    /**
     * The default number of rumbles that are allowed to be active at once per controller
     * @see frc.robot.controllers.BasicController#setRumbleLimit
     * @see frc.robot.controllers.BasicController#MAX_RUMBLE_LIMIT
     */
    public static final int DEFAULT_RUMBLE_LIMIT = 10;
    
    /**
     * Max value for the rumble limit
     * @see frc.robot.controllers.BasicController#DEFAULT_RUMBLE_LIMIT
     * @see frc.robot.controllers.BasicController#setRumbleLimit
     */
    public static final int MAX_RUMBLE_LIMIT = 30;
    
    protected static boolean buttonCheck(BasicController controller, int button) {
        //button ids start at 1
        if(button <= 0) {
            ControllerLogger.warningButtonUnderMin(button);
            return false;
        }
        if(button > controller.getHID().getButtonCount()) {
            ControllerLogger.warningButtonOverMax(controller.getPort(), button, controller.getHID().getButtonCount());
            return false;
        }
        
        return true;
    }
    protected static boolean axisCheck(BasicController controller, int axis) {
        //axis ids start at 0
        if(axis < 0) {
            ControllerLogger.warningAxisUnderMin(axis);
            return false;
        }
        if(axis >= controller.getHID().getAxisCount()) {
            ControllerLogger.warningAxisOverMax(controller.getPort(), axis, controller.getHID().getAxisCount() - 1);
            return false;
        }
        return true;
    }
    /*
     * https://www.desmos.com/calculator/07bcdud2oy
     */
    protected static double applyLinearDeadzone(double val, double deadzone) {
        if(-deadzone <= val && val <= deadzone) return 0;
        
        return (val - (val > 0 ? deadzone : -deadzone)) / (1 - deadzone);
    }
    /*
     * https://www.desmos.com/calculator/07bcdud2oy
     */
    protected static double applyExponentialDeadzone(double val, double deadzone, double power) {
        if(-deadzone <= val && val <= deadzone) return 0;
        
        return Math.pow(Math.abs((val - (val > 0 ? deadzone : -deadzone)) / (1 - deadzone)), power) * (val < 0 ? -1 : 1);
    }
    
    protected GenericHID hid;
    protected Map<Integer, Double> axisDeadzones = new HashMap<>();
    protected Map<Integer, Boolean> buttonBuffer = new HashMap<>();
    protected Map<Integer, Boolean> buttonPressedBuffer = new HashMap<>();
    protected Map<Integer, Boolean> buttonReleasedBuffer = new HashMap<>();
    protected Map<Integer, Double> axisBuffer = new HashMap<>();
    protected int povBuffer = -2;
    protected Map<Integer, Rumble> rumbles = new HashMap<>();
    protected double leftMax = 0;
    protected double rightMax = 0;
    protected int rumbleLimit = DEFAULT_RUMBLE_LIMIT;
    
    /**
     * Create a controller from a new HID device
     * @param port Which port to connect the hid device to. Should be in the range of [0, 5]
     * @param robot A reference to a TimedRobot instance
     * @throws IllegalArgumentException If port is not within the allowed range
     * @throws IllegalArgumentException If robot is null
     */
    public BasicController(int port, TimedRobot robot) throws IllegalArgumentException {
        if(port < 0 || port > 5) {
            ControllerLogger.errorOutOfRange("port", port, "[0, 5]");
        }
        if(robot == null) {
            ControllerLogger.errorNullArgument("robot");
        }
        
        this.hid = new GenericHID(port);
        
        //reset rumbles just in case
        hid.setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 0);
        
        //run periodic at 50 Hz, same cycle as robotPeriodic
        robot.addPeriodic(this::periodic, TimedRobot.kDefaultPeriod);
    }
    /**
     * Create a controller from an already created HID device
     * @param hid An initialized GenericHID or GenericHID child such as {@link edu.wpi.first.wpilibj.Joystick}
     * @param robot A reference to a TimedRobot instance
     * @throws IllegalArgumentException If hid is null
     * @throws IllegalArgumentException If robot is null
     */
    public BasicController(GenericHID hid, TimedRobot robot) throws IllegalArgumentException {
        if(hid == null) {
            ControllerLogger.errorNullArgument("hid");
        }
        if(robot == null) {
            ControllerLogger.errorNullArgument("robot");
        }
        
        this.hid = hid;
        
        //reset rumbles just in case
        hid.setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 0);
        
        //run periodic at 50 Hz, same cycle as robotPeriodic
        robot.addPeriodic(this::periodic, TimedRobot.kDefaultPeriod);
    }
    
    protected void periodic() {
        forceClearBuffers();
        updateRumble();
    }
    protected void updateRumble() {
        //don't bother with controller validity checks since this only being used internally
        
        List<Integer> toRemove = new ArrayList<>();
        double newLeftMax = 0;
        double newRightMax = 0;
        for(int id : rumbles.keySet()) {
            Rumble r = rumbles.get(id);
            
            if(r.timer.hasElapsed(r.duration)) {
                toRemove.add(id); //remove from map after loop is over to avoid undefined behaviour
                continue;
            }
            
            switch(r.type) {
                case LEFT:
                    newLeftMax = Math.max(newLeftMax, r.strength);
                    break;
                case RIGHT:
                    newRightMax = Math.max(newRightMax, r.strength);
                    break;
                case BOTH:
                    newLeftMax = Math.max(newLeftMax, r.strength);
                    newRightMax = Math.max(newRightMax, r.strength);
                    break;
            }
        }
        /*
         * From documentation on Map.keySet():
         * The set supports element removal, which removes the corresponding mapping from the map, via the Iterator.remove, Set.remove, removeAll, retainAll, and clear operations.
         */
        rumbles.keySet().removeAll(toRemove);
        
        //only send update to controller if necessary
        if(newLeftMax != leftMax) {
            leftMax = newLeftMax;
            hid.setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kLeftRumble, leftMax);
        }
        if(newRightMax != leftMax) {
            rightMax = newRightMax;
            hid.setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kRightRumble, rightMax);
        }
    }
    
    /*
     * Setters
     */
    /**
     * Forceably clear the input buffers. Doing so will cause the input to be requeried next time an input function is called. This is already called in a 50 Hz periodic loop. This should not need to be called manually under normal circumstances
     */
    public void forceClearBuffers() {
        buttonBuffer.clear();
        buttonPressedBuffer.clear();
        buttonReleasedBuffer.clear();
        axisBuffer.clear();
        povBuffer = -2;
    }
    /**
     * Configure the deadzone for the given axis. This deadzone is used when calling {@link frc.robot.controllers.BasicController#getAxisLinear} and {@link frc.robot.controllers.BasicController#getAxisExponential}
     * @param axis ID of a controller axis, starting at 0
     * @param deadzone The range in which if the absolute value of the axis is <= the deadzone, then the axis will evaluate to 0. This value is clamped to the range [0, 1]
     * @see frc.robot.controllers.BasicController#getAxisLinear
     * @see frc.robot.controllers.BasicController#getAxisExponential
     */
    public void setControllerAxisDeadzone(int axis, double deadzone) {
        double clampedDeadzone = MathUtil.clamp(deadzone, 0, 1);
        if(clampedDeadzone != deadzone) {
            ControllerLogger.warningClamp("deadzone", deadzone, "[0, 1]");
        }
        axisDeadzones.put(axis, clampedDeadzone);
    }
    /**
     * This will configure how many scheduled rumbles will be allowed to be active at once on this controller. This defaults to {@link frc.robot.controllers.BasicController#DEFAULT_RUMBLE_LIMIT}. The rumble limit only affects newly scheduled rumbles, meaing decreasing this value below what is currently active will not cancel any rumbles. e.g. If 15 rumbles are currently active and you set the rumble limit to 10, then the extra 5 will continue to operate normally, as they were scheduled before the limit was decreased
     * @param maxRumbles Maximum amount of rumbles allowed to be active at once. This value will be clamped to the range [0, {@link frc.robot.controllers.BasicController#MAX_RUMBLE_LIMIT}]
     * @see frc.robot.controllers.BasicController#DEFAULT_RUMBLE_LIMIT
     * @see frc.robot.controllers.BasicController#MAX_RUMBLE_LIMIT
     * @see frc.robot.controllers.BasicController#getCurrentRumbleLimit
     */
    public void setRumbleLimit(int maxRumbles) {
        rumbleLimit = MathUtil.clamp(maxRumbles, 0, MAX_RUMBLE_LIMIT);
        
        if(rumbleLimit != maxRumbles) {
            ControllerLogger.warningClamp("maxRumbles", maxRumbles, String.format("[0, %d]", MAX_RUMBLE_LIMIT));
        }
    }
    
    /*
     * Getters
     */
    /**
     * Get the interal HID device
     */
    public GenericHID getHID() {
        return hid;
    }
    /**
     * Get the currently configured maximum rumbles. This will default to {@link frc.robot.controllers.BasicController#DEFAULT_RUMBLE_LIMIT}
     * @see frc.robot.controllers.BasicController#DEFAULT_RUMBLE_LIMIT
     * @see frc.robot.controllers.BasicController#setRumbleLimit
     */
    public int getCurrentRumbleLimit() {
        return rumbleLimit;
    }
    /**
     * Get what port this controller is connected to in the range [0, 5]
     */
    public int getPort() {
        return hid.getPort();
    }
    
    /**
     * Get whether a button is currently pressed or not
     * @param button ID of a controller button, starting at 1
     * @return Will return false if the given button doesn't exist
     */
    public boolean getButton(int button) {
        if(!buttonCheck(this, button)) return false;
        
        if(buttonBuffer.containsKey(button)) {
            return buttonBuffer.get(button);
        }
        
        boolean val = hid.getRawButton(button);
        buttonBuffer.put(button, val);
        return val;
    }
    /**
     * Get whether a button is currently pressed but was not the previous frame
     * @param button ID of a controller button, starting at 1
     * @return Will return false if the given button doesn't exist
     */
    public boolean getButtonPressed(int button) {
        if(!buttonCheck(this, button)) return false;
        
        if(buttonPressedBuffer.containsKey(button)) {
            return buttonPressedBuffer.get(button);
        }
        
        boolean val = hid.getRawButtonPressed(button);
        buttonPressedBuffer.put(button, val);
        return val;
    }
    /**
     * Get whether a button is currently <b>not</b> pressed but was the previous frame
     * @param button ID of a controller button, starting at 1
     * @return Will return false if the given button doesn't exist
     */
    public boolean getButtonReleased(int button) {
        if(!buttonCheck(this, button)) return false;
        
        if(buttonReleasedBuffer.containsKey(button)) {
            return buttonReleasedBuffer.get(button);
        }
        
        boolean val = hid.getRawButtonReleased(button);
        buttonReleasedBuffer.put(button, val);
        return val;
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.BasicController#getButton}. <b>Do not</b> use this function to get the value of a button as a boolean. Use {@link frc.robot.controllers.BasicController#getButton} for that.
     * @param button ID of a controller button, starting at 1
     * @see https://github.com/wpilibsuite/allwpilib/issues/5903
     * @see frc.robot.controllers.BasicController#getButton
     */
    public Trigger getButtonTrigger(int button) {
        return new Trigger(() -> getButton(button));
    }
    
    /**
     * Get the axis value without applying deadzone configurations
     * @param axis ID of a controller axis, starting at 0
     * @return Will return 0 if the given axis doesn't exist
     */
    public double getAxisRaw(int axis) {
        if(!axisCheck(this, axis)) return 0;
        
        if(axisBuffer.containsKey(axis)) {
            return axisBuffer.get(axis);
        }
        
        double val = hid.getRawAxis(axis);
        axisBuffer.put(axis, val);
        return val;
    }
    /**
     * Get the axis value with the configured axis deadzone
     * @param axis ID of a controller axis, starting at 0
     * @return Will return 0 if the given axis doesn't exist
     * @see https://www.desmos.com/calculator/07bcdud2oy
     * @see frc.robot.controllers.BasicController#setControllerAxisDeadzone
     */
    public double getAxisLinear(int axis) {
        if(!axisCheck(this, axis)) return 0;
        
        double deadzone = axisDeadzones.getOrDefault(axis, 0d);
        if(axisBuffer.containsKey(axis)) {
            return applyLinearDeadzone(axisBuffer.get(axis), deadzone);
        }
        
        double val = hid.getRawAxis(axis);
        axisBuffer.put(axis, val);
        return applyLinearDeadzone(val, deadzone);
    }
    /**
     * Get the axis value with the configured axis deadzone and apply an exponential curve to it
     * @param axis ID of a controller axis, starting at 0
     * @param power What power of exponent to apply. Will be clamped to the range [0, ∞) See variable <i>s</i> in the example graph
     * @return Will return 0 if the given axis doesn't exist
     * @see https://www.desmos.com/calculator/07bcdud2oy
     * @see frc.robot.controllers.BasicController#setControllerAxisDeadzone
     */
    public double getAxisExponential(int axis, double power) {
        if(!axisCheck(this, axis)) return 0;
        
        double deadzone = axisDeadzones.getOrDefault(axis, 0d);
        double clampedPower = Math.max(power, 0);
        if(clampedPower != power) {
            ControllerLogger.warningClamp("power", power, "[0, ∞)");
        }
        
        if(axisBuffer.containsKey(axis)) {
            return applyExponentialDeadzone(axisBuffer.get(axis), deadzone, clampedPower);
        }
        
        double val = hid.getRawAxis(axis);
        axisBuffer.put(axis, val);
        return applyExponentialDeadzone(val, deadzone, clampedPower);
    }
    /**
     * Get if the raw axis value is greater than the given value
     * @param axis ID of a controller axis, starting at 0
     * @param val Discriminating value
     * @return Will return false if the given axis doesn't exist
     * @see frc.robot.controllers.BasicController#getAxisRaw
     */
    public boolean getAxisRawGreaterThan(int axis, double val) {
        if(!axisCheck(this, axis)) return false;
        
        return getAxisRaw(axis) > val;
    }
    /**
     * Get if the raw axis value is less than the given value
     * @param axis ID of a controller axis, starting at 0
     * @param val Discriminating value
     * @return Will return false if the given axis doesn't exist
     * @see frc.robot.controllers.BasicController#getAxisRaw
     */
    public boolean getAxisRawLessThan(int axis, double val) {
        if(!axisCheck(this, axis)) return false;
        
        return getAxisRaw(axis) < val;
    }
    /**
     * Get if the linearly calibrated axis value is greater than the given value
     * @param axis ID of a controller axis, starting at 0
     * @param val Discriminating value
     * @return Will return false if the given axis doesn't exist
     * @see frc.robot.controllers.BasicController#getAxisLinear
     */
    public boolean getAxisLinearGreaterThan(int axis, double val) {
        if(!axisCheck(this, axis)) return false;
        
        return getAxisLinear(axis) > val;
    }
    /**
     * Get if the linearly calibrated axis value is less than the given value
     * @param axis ID of a controller axis, starting at 0
     * @param val Discriminating value
     * @return Will return false if the given axis doesn't exist
     * @see frc.robot.controllers.BasicController#getAxisLinear
     */
    public boolean getAxisLinearLessThan(int axis, double val) {
        if(!axisCheck(this, axis)) return false;
        
        return getAxisLinear(axis) < val;
    }
    /**
     * Get if the exponentially calibrated axis value is greater than the given value
     * @param axis ID of a controller axis, starting at 0
     * @param power What power of exponential curve to apply
     * @param val Discriminating value
     * @return Will return false if the given axis doesn't exist
     * @see frc.robot.controllers.BasicController#getAxisExponential
     */
    public boolean getAxisExponentialGreaterThan(int axis, double power, double val) {
        if(!axisCheck(this, axis)) return false;
        
        return getAxisExponential(axis, power) > val;
    }
    /**
     * Get if the linearly calibrated axis value is less than the given value
     * @param axis ID of a controller axis, starting at 0
     * @param power What power of exponential curve to apply
     * @param val Discriminating value
     * @return Will return false if the given axis doesn't exist
     * @see frc.robot.controllers.BasicController#getAxisExponential
     */
    public boolean getAxisExponentialLessThan(int axis, double power, double val) {
        if(!axisCheck(this, axis)) return false;
        
        return getAxisExponential(axis, power) < val;
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.BasicController#getAxisRawGreaterThan}
     * @param axis ID of a controller axis, starting at 0
     * @param val Discriminating value
     * @see frc.robot.controllers.BasicController#getAxisRawGreaterThan
     */
    public Trigger getAxisRawGreaterThanTrigger(int axis, double val) {
        return new Trigger(() -> getAxisRaw(axis) > val);
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.BasicController#getAxisRawLessThan}
     * @param axis ID of a controller axis, starting at 0
     * @param val Discriminating value
     * @see frc.robot.controllers.BasicController#getAxisRawLessThan
     */
    public Trigger getAxisRawLessThanTrigger(int axis, double val) {
        return new Trigger(() -> getAxisRaw(axis) < val);
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.BasicController#getAxisLinearGreaterThan}
     * @param axis ID of a controller axis, starting at 0
     * @param val Discriminating value
     * @see frc.robot.controllers.BasicController#getAxisLinearGreaterThan
     */
    public Trigger getAxisLinearGreaterThanTrigger(int axis, double val) {
        return new Trigger(() -> getAxisLinear(axis) > val);
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.BasicController#getAxisLinearLessThan}
     * @param axis ID of a controller axis, starting at 0
     * @param val Discriminating value
     * @see frc.robot.controllers.BasicController#getAxisLinearLessThan
     */
    public Trigger getAxisLinearLessThanTrigger(int axis, double val) {
        return new Trigger(() -> getAxisLinear(axis) < val);
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.BasicController#getAxisExponentialGreaterThan}
     * @param axis ID of a controller axis, starting at 0
     * @param power What power of exponential curve to apply. Will be clamped to the range [0, ∞)
     * @param val Discriminating value
     * @see frc.robot.controllers.BasicController#getAxisExponentialGreaterThan
     */
    public Trigger getAxisExponentialGreaterThanTrigger(int axis, double power, double val) {
        return new Trigger(() -> getAxisExponential(axis, power) > val);
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.BasicController#getAxisExponentialLessThan}
     * @param axis ID of a controller axis, starting at 0
     * @param power What power of exponential curve to apply. Will be clamped to the range [0, ∞)
     * @param val Discriminating value
     * @see frc.robot.controllers.BasicController#getAxisExponentialLessThan
     */
    public Trigger getAxisExponentialLessThanTrigger(int axis, double power, double val) {
        return new Trigger(() -> getAxisExponential(axis, power) < val);
    }
    
    /**
     * Get the angle in degrees of the POV stick
     * @return the angle of the currently pressed pov button, or -1 if it isn't pressed
     * @see edu.wpi.first.wpilibj.GenericHID#getPOV
     */
    public int getPOVAngle() {
        if(povBuffer != -2) {
            return povBuffer;
        }
        
        int val = hid.getPOV();
        povBuffer = val;
        return val;
    }
    /**
     * Test if the POV stick is currently pressed up
     * @see frc.robot.controllers.BasicController#getPOVAngle
     */
    public boolean getPOVUp() {
        return getPOVAngle() == 0;
    }
    /**
     * Test if the POV stick is currently pressed diagonally to the upper right
     * @see frc.robot.controllers.BasicController#getPOVAngle
     */
    public boolean getPOVUpRight() {
        return getPOVAngle() == 45;
    }
    /**
     * Test if the POV stick is currently pressed right
     * @see frc.robot.controllers.BasicController#getPOVAngle
     */
    public boolean getPOVRight() {
        return getPOVAngle() == 90;
    }
    /**
     * Test if the POV stick is currently pressed diagonally to the lower right
     * @see frc.robot.controllers.BasicController#getPOVAngle
     */
    public boolean getPOVDownRight() {
        return getPOVAngle() == 135;
    }
    /**
     * Test if the POV stick is currently pressed down
     * @see frc.robot.controllers.BasicController#getPOVAngle
     */
    public boolean getPOVDown() {
        return getPOVAngle() == 180;
    }
    /**
     * Test if the POV stick is currently pressed diagonally to the lower left
     * @see frc.robot.controllers.BasicController#getPOVAngle
     */
    public boolean getPOVDownLeft() {
        return getPOVAngle() == 225;
    }
    /**
     * Test if the POV stick is currently pressed left
     * @see frc.robot.controllers.BasicController#getPOVAngle
     */
    public boolean getPOVLeft() {
        return getPOVAngle() == 270;
    }
    /**
     * Test if the POV stick is currently pressed diagonally to the upper left
     * @see frc.robot.controllers.BasicController#getPOVAngle
     */
    public boolean getPOVUpLeft() {
        return getPOVAngle() == 315;
    }
    /**
     * Test if the POV stick is currently being pressed in any direction
     * @see frc.robot.controllers.BasicController#getPOVAngle
     */
    public boolean getPOVAny() {
        return getPOVAngle() != -1;
    }
    /**
     * Test if the POV stick isn't currently being pressed
     * @see frc.robot.controllers.BasicController#getPOVAngle
     */
    public boolean getPOVNone() {
        return getPOVAngle() == -1;
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.BasicController#getPOVUp}
     * @see frc.robot.controllers.BasicController#getPOVUp
     */
    public Trigger getPOVUpTrigger() {
        return new Trigger(() -> getPOVUp());
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.BasicController#getPOVUpRight}
     * @see frc.robot.controllers.BasicController#getPOVUpRight
     */
    public Trigger getPOVUpRightTrigger() {
        
        return new Trigger(() -> getPOVUpRight());
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.BasicController#getPOVRight}
     * @see frc.robot.controllers.BasicController#getPOVRight
     */
    public Trigger getPOVRightTrigger() {
        return new Trigger(() -> getPOVRight());
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.BasicController#getPOVDownRight}
     * @see frc.robot.controllers.BasicController#getPOVDownRight
     */
    public Trigger getPOVDownRightTrigger() {
        return new Trigger(() -> getPOVDownRight());
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.BasicController#getPOVDown}
     * @see frc.robot.controllers.BasicController#getPOVDown
     */
    public Trigger getPOVDownTrigger() {
        return new Trigger(() -> getPOVDown());
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.BasicController#getPOVDownLeft}
     * @see frc.robot.controllers.BasicController#getPOVDownLeft
     */
    public Trigger getPOVDownLeftTrigger() {
        return new Trigger(() -> getPOVDownLeft());
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.BasicController#getPOVLeft}
     * @see frc.robot.controllers.BasicController#getPOVLeft
     */
    public Trigger getPOVLeftTrigger() {
        return new Trigger(() -> getPOVLeft());
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.BasicController#getPOVUpLeft}
     * @see frc.robot.controllers.BasicController#getPOVUpLeft
     */
    public Trigger getPOVUpLeftTrigger() {
        return new Trigger(() -> getPOVUpLeft());
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.BasicController#getPOVAny}
     * @see frc.robot.controllers.BasicController#getPOVAny
     */
    public Trigger getPOVAnyTrigger() {
        return new Trigger(() -> getPOVAny());
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.BasicController#getPOVNone}
     * @see frc.robot.controllers.BasicController#getPOVNone
     */
    public Trigger getPOVNoneTrigger() {
        return new Trigger(() -> getPOVNone());
    }

    /**
     * Schedule a controller rumble. This class will handle having multiple run at the same time by choosing the rumble that has the highest strength to run on the controller
     * @param type Whether to activate the left, right, or both rumble motors
     * @param strength Magnitude of the rumble. Should be in the range of (0, 1], 1 being 100%. This value will be clamped within the acceptable range
     * @param duration How long this particular rumble should last in seconds. Should be in the range of (0, ∞). This value will be clamped within the acceptable range
     * @return Rumble ID. This can be used to cancel the rumble. This ID will be -1 if the rumble couldn't be scheduled because the rumble limit has been reached
     * @see frc.robot.controllers.BasicController#cancelRumble
     * @see frc.robot.controllers.BasicController#cancelAllRumbles
     * @see frc.robot.controllers.BasicController#rumbleEndTrigger
     * @see frc.robot.controllers.BasicController#DEFAULT_RUMBLE_LIMIT
     * @see frc.robot.controllers.BasicController#setRumbleLimit
     * @see frc.robot.controllers.BasicController#getCurrentRumbleLimit
     */
    public int scheduleRumble(int controller, RumbleType type, double strength, double duration) {
        double clampedStrength = MathUtil.clamp(strength, Double.MIN_NORMAL, 1);
        if(clampedStrength != strength) {
            ControllerLogger.warningClamp("strength", strength, "(0, 1]");
        }
        double clampedDuration = Math.max(duration, Double.MIN_NORMAL);
        if(clampedDuration != duration) {
            ControllerLogger.warningClamp("duration", duration, "(0, ∞)");
        }
        
        if(rumbles.size() == rumbleLimit) {
            ControllerLogger.warningReachedRumbleLimit(rumbleLimit);
            return -1;
        }
        
        //pick random id that isn't already being used
        int id;
        do {
            id = (int) (Math.random() * Integer.MAX_VALUE);
        }
        while(rumbles.keySet().contains(id));
        
        Rumble r = new Rumble();
        r.type = type;
        r.strength = strength;
        r.duration = duration;
        r.timer = new Timer();
        r.timer.start();
        
        rumbles.put(id, r);
        
        switch(type) {
            case LEFT:
                leftMax = Math.max(leftMax, strength);
                hid.setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kLeftRumble, leftMax);
                break;
            case RIGHT:
                rightMax = Math.max(rightMax, strength);
                hid.setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kRightRumble, rightMax);
                break;
            case BOTH:
                leftMax = Math.max(leftMax, strength);
                rightMax = Math.max(rightMax, strength);
                hid.setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kLeftRumble, leftMax);
                hid.setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kRightRumble, rightMax);
                break;
        }
        
        return id;
    }
    /**
     * Cancel an already active rumble
     * @param rumble Rumble ID returned by {@link frc.robot.controllers.BasicController#scheduleRumble}
     * @throws IllegalArgumentException If a rumble with the given ID isn't currently active
     * @see frc.robot.controllers.BasicController#scheduleRumble
     * @see frc.robot.controllers.BasicController#isRumbleIDValid
     */
    public void cancelRumble(int controller, int rumble) throws IllegalAccessException {
        if(!rumbles.containsKey(rumble)) {
            ControllerLogger.errorInvalidRumble(controller, rumble);
        }
        
        rumbles.remove(rumble);
        updateRumble(); //manually update rumble for immediate cancel
    }
    /**
     * Cancel all active rumbles on the controller
     * @see frc.robot.controllers.BasicController#scheduleRumble
     */
    public void cancelAllRumbles(int controller) {
        rumbles.clear();
        leftMax = 0;
        rightMax = 0;
        hid.setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 0);
    }
    /**
     * Check if a rumble ID is currently valid
     * @param rumble Rumble ID returned by {@link frc.robot.controllers.BasicController#scheduleRumble}
     * @return Whether the ID is valid
     * @see frc.robot.controllers.BasicController#scheduleRumble
     */
    public boolean isRumbleIDValid(int rumble) {
        return rumbles.containsKey(rumble);
    }
    /**
     * Get a Trigger that returns true when the given rumble ID is no longer valid, i.e. when the rumble ends
     * @param rumble Rumble ID returned by {@link frc.robot.controllers.BasicController#scheduleRumble}
     * @return A Trigger that tracks the value of {@link frc.robot.controllers.BasicController#isRumbleIDValid}
     * @see frc.robot.controllers.BasicController#scheduleRumble
     * @see frc.robot.controllers.BasicController#isRumbleIDValid
     */
    public Trigger rumbleEndTrigger(int rumble) {
        return new Trigger(() -> !isRumbleIDValid(rumble));
    }
}
