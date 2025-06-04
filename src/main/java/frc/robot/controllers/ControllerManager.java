package frc.robot.controllers;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;

/*
 * TODO Redo logging to have verbosity config and be more detailed -> break out into logging helper class
 * TODO Add controller binding constants -> Possibly rewrite to use controller classes rather than just one big abstract class
 * TODO Review documentation for errors, Javadocs on github?
 */
/**
 * A utility to handle the finer details of controller input for you. This is an abstract class, all available functions are called statically
 */
public abstract class ControllerManager {
    
    /**
     * For internal use only
     */
    private static class Controller {
        public GenericHID hid;
        
        public Map<Integer, Double> axisDeadzones = new HashMap<>();
        
        public Map<Integer, Boolean> buttonBuffer = new HashMap<>();
        public Map<Integer, Boolean> buttonPressedBuffer = new HashMap<>();
        public Map<Integer, Boolean> buttonReleasedBuffer = new HashMap<>();
        public Map<Integer, Double> axisBuffer = new HashMap<>();
        public int povBuffer = -2;
        
        public Map<Integer, Rumble> rumbles = new HashMap<>();
        public double leftMax = 0;
        public double rightMax = 0;
    }
    /**
     * For internal use only
     */
    private static class Rumble {
        public RumbleType type;
        public double strength;
        public double duration;
        public Timer timer;
    }
    
    private static Map<Integer, Controller> controllers = new HashMap<>();
    private static boolean hasInited = false;
    
    /**
     * Create and register a HID device
     * @param id Which port to connect the hid device to. Should be in the range of [0, 5]
     */
    public static void createController(int id) {
        if(controllers.containsKey(id)) {
            DriverStation.reportError(String.format("Cannot register multiple controllers with id %d", id), true);
            return;
        }
        if(id < 0) {
            DriverStation.reportError("Cannot create a controller with negative id", true);
            return;
        }
        if(id > 5) {
            DriverStation.reportWarning(String.format("The driver station only supports controller ids 0-5, id %d will be inaccessible", id), false);
        }
        
        if(!hasInited) init();
        
        Controller c = new Controller();
        c.hid = new GenericHID(id);
        
        //reset rumbles just in case
        c.hid.setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 0);
        
        controllers.put(id, c);
    }
    /**
     * Register an already created HID device
     * @param hid
     */
    public static void addController(GenericHID hid) {
        if(hid == null) {
            DriverStation.reportError("Cannot create a controller from a null HID device", true);
            return;
        }
        
        int id = hid.getPort();
        if(controllers.containsKey(id)) {
            DriverStation.reportError(String.format("Cannot register multiple controllers with id %d", id), true);
            return;
        }
        
        if(!hasInited) init();
        
        Controller c= new Controller();
        c.hid = hid;
        
        controllers.put(id, c);
    }
    
    /*
     * Internal tools
     */
    private static void init() {
        hasInited = true;
        
        //run periodic at 50 Hz, same cycle as robotPeriodic
        Robot.instance.addPeriodic(ControllerManager::periodic, Robot.kDefaultPeriod);
    }
    private static void periodic() {
        for(int i = 0; i < controllers.values().size(); i++) {
            forceClearBuffers(i);
            updateRumble(i);
        }
    }
    private static boolean buttonCheck(int controller, int button) {
        if(!controllers.containsKey(controller)) {
            DriverStation.reportWarning(String.format("No controller with id %d has been registered", controller), false);
            return false;
        }
        
        //button ids start at 1
        if(button <= 0) {
            DriverStation.reportWarning("Button ids must be natural numbers", false);
            return false;
        }
        if(button > controllers.get(controller).hid.getButtonCount()) {
            DriverStation.reportWarning(String.format("Controller id %d doesn't have a button with id %d", controller, button), false);
            return false;
        }
        return true;
    }
    private static boolean axisCheck(int controller, int axis) {
        if(!controllers.containsKey(controller)) {
            DriverStation.reportWarning(String.format("No controller with id %d has been registered", controller), false);
            return false;
        }
        
        //axis ids start at 0
        if(axis < 0) {
            DriverStation.reportWarning("Axis ids must be whole numbers", false);
            return false;
        }
        if(axis >= controllers.get(controller).hid.getAxisCount()) {
            DriverStation.reportWarning(String.format("Controller id %d doesn't have an axis with id %d", controller, axis), false);
            return false;
        }
        return true;
    }
    /*
     * https://www.desmos.com/calculator/07bcdud2oy
     */
    private static double applyLinearDeadzone(double val, double deadzone) {
        if(-deadzone <= val && val <= deadzone) return 0;
        
        return (val - (val > 0 ? deadzone : -deadzone)) / (1 - deadzone);
    }
    /*
     * https://www.desmos.com/calculator/07bcdud2oy
     */
    private static double applyExponentialDeadzone(double val, double deadzone, double power) {
        if(-deadzone <= val && val <= deadzone) return 0;
        
        return Math.pow(Math.abs((val - (val > 0 ? deadzone : -deadzone)) / (1 - deadzone)), power) * (val < 0 ? -1 : 1);
    }
    private static boolean povCheck(int controller) {
        if(!controllers.containsKey(controller)) {
            DriverStation.reportWarning(String.format("No controller with id %d has been registered", controller), false);
            return false;
        }
        
        return true;
    }
    private static void updateRumble(int controller) {
        //don't bother with controller validity checks since this only being used internally
        
        Controller c = controllers.get(controller);
        List<Integer> toRemove = new ArrayList<>();
        double newLeftMax = 0;
        double newRightMax = 0;
        for(int id : c.rumbles.keySet()) {
            Rumble r = c.rumbles.get(id);
            
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
        c.rumbles.keySet().removeAll(toRemove);
        
        //only send update to controller if necessary
        if(newLeftMax != c.leftMax) {
            c.leftMax = newLeftMax;
            c.hid.setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kLeftRumble, c.leftMax);
        }
        if(newRightMax != c.leftMax) {
            c.rightMax = newRightMax;
            c.hid.setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kRightRumble, c.rightMax);
        }
    }
    
    /*
     * Setters
     */
    /**
     * Forceable clear the input buffers of the given controller. Doing so will cause the input to be requeried next time an input function is called. This is already called by {@link frc.robot.controllers.ControllerManager#periodic() periodic}. This should not need to be called manually under normal circumstances
     * @param controller ID of registered controller
     */
    public static void forceClearBuffers(int controller) {
        if(!controllers.containsKey(controller)) {
            DriverStation.reportWarning(String.format("No controller with id %d has been registered", controller), false);
            return;
        }
        
        Controller c = controllers.get(controller);
        c.buttonBuffer.clear();
        c.buttonPressedBuffer.clear();
        c.buttonReleasedBuffer.clear();
        c.axisBuffer.clear();
        c.povBuffer = -2;
    }
    /**
     * Configure the deadzone for the given controller and axis. This deadzone is used when calling {@link frc.robot.controllers.ControllerManager#getAxisLinear getAxisLinear} and {@link frc.robot.controllers.ControllerManager#getAxisExponential(int, int, double) getAxisExponential}
     * @param controller ID of registered controller
     * @param axis ID of controller axis, starting at 0
     * @param deadzone The range in which if the absolute value of the axis is <= the deadzone, then it will equal 0. This value is clamped to the range [0, 1]
     */
    public static void setControllerAxisDeadzone(int controller, int axis, double deadzone) {
        if(!controllers.containsKey(controller)) {
            DriverStation.reportError(String.format("No controller with id %d has been registered", controller), false);
            return;
        }
        
        controllers.get(controller).axisDeadzones.put(axis, MathUtil.clamp(deadzone, 0, 1));
    }
    
    /*
     * Getters
     */
    /**
     * Get the interal HID device for the given controller
     * @param controller
     * @return Will return null if the given controller doesn't exist
     */
    public static GenericHID getHID(int controller) {
        if(!controllers.containsKey(controller)) {
            DriverStation.reportWarning(String.format("No controller with id %d has been registered", controller), false);
            return null;
        }
        
        return controllers.get(controller).hid;
    }
    
    /**
     * Get whether a button is currently pressed or not
     * @param controller ID of the registered controller
     * @param button ID of the controller button, starting at 1
     * @return Will return false if the given controller or button doesn't exist
     */
    public static boolean getButton(int controller, int button) {
        if(!buttonCheck(controller, button)) return false;
        
        Controller c = controllers.get(controller);
        if(c.buttonBuffer.containsKey(button)) {
            return c.buttonBuffer.get(button);
        }
        
        boolean val = controllers.get(controller).hid.getRawButton(button);
        c.buttonBuffer.put(button, val);
        return val;
    }
    /**
     * Get whether a button is currently pressed but was not the previous frame
     * @param controller ID of the registered controller
     * @param button ID of the controller button, starting at 1
     * @return Will return false if the given controller or button doesn't exist
     */
    public static boolean getButtonPressed(int controller, int button) {
        if(!buttonCheck(controller, button)) return false;
        
        Controller c = controllers.get(controller);
        if(c.buttonPressedBuffer.containsKey(button)) {
            return c.buttonPressedBuffer.get(button);
        }
        
        boolean val = controllers.get(controller).hid.getRawButtonPressed(button);
        c.buttonPressedBuffer.put(button, val);
        return val;
    }
    /**
     * Get whether a button is currently <b>not</b> pressed but was the previous frame
     * @param controller ID of the registered controller
     * @param button ID of the controller button, starting at 1
     * @return Will return false if the given controller or button doesn't exist
     */
    public static boolean getButtonReleased(int controller, int button) {
        if(!buttonCheck(controller, button)) return false;
        
        Controller c = controllers.get(controller);
        if(c.buttonReleasedBuffer.containsKey(button)) {
            return c.buttonReleasedBuffer.get(button);
        }
        
        boolean val = controllers.get(controller).hid.getRawButtonReleased(button);
        c.buttonReleasedBuffer.put(button, val);
        return val;
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.ControllerManager#getButton getButton}. Do not use this function to get the value of a button. Use {@link frc.robot.controllers.ControllerManager#getButton getButton} for that.
     * @param controller ID of the registered controller
     * @param button ID of the controller button, starting at 1
     * @return Will return a Trigger that will always evaluate to false if the given controller or button doesn't exist
     * @see https://github.com/wpilibsuite/allwpilib/issues/5903
     */
    public static Trigger getButtonTrigger(int controller, int button) {
        if(!buttonCheck(controller, button)) return new Trigger(() -> false);
        
        //directly use getRawButton rather than getButton to prevent a bunch of unneccessary checks
        return new Trigger(() -> controllers.get(controller).hid.getRawButton(button));
    }
    
    /**
     * Get the axis value without applying deadzone configurations
     * @param controller ID of registered controller
     * @param axis ID of controller axis, starting at 0
     * @return Will return 0 if the given controller or axis doesn't exist
     */
    public static double getAxisRaw(int controller, int axis) {
        if(!axisCheck(controller, axis)) return 0;
        
        Controller c = controllers.get(controller);
        if(c.axisBuffer.containsKey(axis)) {
            return c.axisBuffer.get(axis);
        }
        
        double val = c.hid.getRawAxis(axis);
        c.axisBuffer.put(axis, val);
        return val;
    }
    /**
     * Get the axis value with the configured axis deadzone
     * @param controller ID of registered controller
     * @param axis ID of controller axis, starting at 0
     * @return Will return 0 if the given controller or axis doesn't exist
     * @see https://www.desmos.com/calculator/07bcdud2oy
     */
    public static double getAxisLinear(int controller, int axis) {
        if(!axisCheck(controller, axis)) return 0;
        
        Controller c = controllers.get(controller);
        double deadzone = c.axisDeadzones.getOrDefault(axis, 0d);
        if(c.axisBuffer.containsKey(axis)) {
            return applyLinearDeadzone(c.axisBuffer.get(axis), deadzone);
        }
        
        double val = c.hid.getRawAxis(axis);
        c.axisBuffer.put(axis, val);
        return applyLinearDeadzone(val, deadzone);
    }
    /**
     * Get the axis value with the configured axis deadzone and apply an exponential curve to it
     * @param controller ID of registered controller
     * @param axis ID of controller axis, starting at 0
     * @param power What power of exponent to apply. Will be clamped to the range [0, ∞) See variable <i>s</i> in the example graph
     * @return Will return 0 if the given controller or axis doesn't exist
     * @see https://www.desmos.com/calculator/07bcdud2oy
     */
    public static double getAxisExponential(int controller, int axis, double power) {
        if(!axisCheck(controller, axis)) return 0;
        
        Controller c = controllers.get(controller);
        double deadzone = c.axisDeadzones.getOrDefault(axis, 0d);
        power = Math.max(power, 0);
        if(c.axisBuffer.containsKey(axis)) {
            return applyExponentialDeadzone(c.axisBuffer.get(axis), deadzone, power);
        }
        
        double val = c.hid.getRawAxis(axis);
        c.axisBuffer.put(axis, val);
        return applyExponentialDeadzone(val, deadzone, power);
    }
    /**
     * Get if the raw axis value is greater than the given value
     * @param controller ID of the registered controller
     * @param axis ID of the controller axis, starting at 0
     * @param val Discriminating value
     * @return Will return false if the given controller or axis doesn't exist
     * @see frc.robot.controllers.ControllerManager#getAxisRaw getAxisRaw
     */
    public static boolean getAxisRawGreaterThan(int controller, int axis, double val) {
        return getAxisRaw(controller, axis) > val;
    }
    /**
     * Get if the raw axis value is less than the given value
     * @param controller ID of the registered controller
     * @param axis ID of the controller axis, starting at 0
     * @param val Discriminating value
     * @return Will return false if the given controller or axis doesn't exist
     * @see frc.robot.controllers.ControllerManager#getAxisRaw getAxisRaw
     */
    public static boolean getAxisRawLessThan(int controller, int axis, double val) {
        if(!axisCheck(controller, axis)) return false;
        
        return getAxisRaw(controller, axis) < val;
    }
    /**
     * Get if the linearly calibrated axis value is greater than the given value
     * @param controller ID of the registered controller
     * @param axis ID of the controller axis, starting at 0
     * @param val Discriminating value
     * @return Will return false if the given controller or axis doesn't exist
     * @see frc.robot.controllers.ControllerManager#getAxisLinear getAxisLinear
     */
    public static boolean getAxisLinearGreaterThan(int controller, int axis, double val) {
        return getAxisLinear(controller, axis) > val;
    }
    /**
     * Get if the linearly calibrated axis value is less than the given value
     * @param controller ID of the registered controller
     * @param axis ID of the controller axis, starting at 0
     * @param val Discriminating value
     * @return Will return false if the given controller or axis doesn't exist
     * @see frc.robot.controllers.ControllerManager#getAxisLinear getAxisLinear
     */
    public static boolean getAxisLinearLessThan(int controller, int axis, double val) {
        if(!axisCheck(controller, axis)) return false;
        
        return getAxisLinear(controller, axis) < val;
    }
    /**
     * Get if the exponentially calibrated axis value is greater than the given value
     * @param controller ID of the registered controller
     * @param axis ID of the controller axis, starting at 0
     * @param power What power of exponential curve to apply
     * @param val Discriminating value
     * @return Will return false if the given controller or axis doesn't exist
     * @see frc.robot.controllers.ControllerManager#getAxisExponential getAxisExponential
     */
    public static boolean getAxisExponentialGreaterThan(int controller, int axis, double power, double val) {
        return getAxisExponential(controller, axis, power) > val;
    }
    /**
     * Get if the linearly calibrated axis value is less than the given value
     * @param controller ID of the registered controller
     * @param axis ID of the controller axis, starting at 0
     * @param power What power of exponential curve to apply
     * @param val Discriminating value
     * @return Will return false if the given controller or axis doesn't exist
     * @see frc.robot.controllers.ControllerManager#getAxisExponential getAxisExponential
     */
    public static boolean getAxisExponentialLessThan(int controller, int axis, double power, double val) {
        if(!axisCheck(controller, axis)) return false;
        
        return getAxisExponential(controller, axis, power) < val;
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.ControllerManager#getAxisRawGreaterThan getAxisRawGreaterThan}
     * @param controller ID of the registered controller
     * @param axis ID of the controller axis, starting at 0
     * @param val Discriminating value
     * @return Will return a Trigger that will always evaluate to false if the given controller or axis doesn't exist
     */
    public static Trigger getAxisRawGreaterThanTrigger(int controller, int axis, double val) {
        if(!axisCheck(controller, axis)) return new Trigger(() -> false);
        
        return new Trigger(() -> controllers.get(controller).hid.getRawAxis(axis) > val);
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.ControllerManager#getAxisRawLessThan getAxisRawLessThan}
     * @param controller ID of the registered controller
     * @param axis ID of the controller axis, starting at 0
     * @param val Discriminating value
     * @return Will return a Trigger that will always evaluate to false if the given controller or axis doesn't exist
     */
    public static Trigger getAxisRawLessThanTrigger(int controller, int axis, double val) {
        if(!axisCheck(controller, axis)) return new Trigger(() -> false);
        
        return new Trigger(() -> controllers.get(controller).hid.getRawAxis(axis) < val);
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.ControllerManager#getAxisLinearGreaterThan getAxisLinearGreaterThan}
     * @param controller ID of the registered controller
     * @param axis ID of the controller axis, starting at 0
     * @param val Discriminating value
     * @return Will return a Trigger that will always evaluate to false if the given controller or axis doesn't exist
     */
    public static Trigger getAxisLinearGreaterThanTrigger(int controller, int axis, double val) {
        if(!axisCheck(controller, axis)) return new Trigger(() -> false);
        
        double deadzone = controllers.get(controller).axisDeadzones.getOrDefault(controller, 0d);
        return new Trigger(() -> applyLinearDeadzone(controllers.get(controller).hid.getRawAxis(axis), deadzone) > val);
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.ControllerManager#getAxisLinearLessThan getAxisLinearLessThan}
     * @param controller ID of the registered controller
     * @param axis ID of the controller axis, starting at 0
     * @param val Discriminating value
     * @return Will return a Trigger that will always evaluate to false if the given controller or axis doesn't exist
     */
    public static Trigger getAxisLinearLessThanTrigger(int controller, int axis, double val) {
        if(!axisCheck(controller, axis)) return new Trigger(() -> false);
        
        double deadzone = controllers.get(controller).axisDeadzones.getOrDefault(controller, 0d);
        return new Trigger(() -> applyLinearDeadzone(controllers.get(controller).hid.getRawAxis(axis), deadzone) < val);
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.ControllerManager#getAxisExponentialGreaterThan getAxisExponentialGreaterThan}
     * @param controller ID of the registered controller
     * @param axis ID of the controller axis, starting at 0
     * @param power What power of exponential curve to apply. Will be clamped to the range [0, ∞)
     * @param val Discriminating value
     * @return Will return a Trigger that will always evaluate to false if the given controller or axis doesn't exist
     */
    public static Trigger getAxisExponentialGreaterThanTrigger(int controller, int axis, double power, double val) {
        if(!axisCheck(controller, axis)) return new Trigger(() -> false);
        
        double deadzone = controllers.get(controller).axisDeadzones.getOrDefault(controller, 0d);
        return new Trigger(() -> applyExponentialDeadzone(controllers.get(controller).hid.getRawAxis(axis), deadzone, power) > val);
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.ControllerManager#getAxisExponentialLessThan getAxisExponentialLessThan}
     * @param controller ID of the registered controller
     * @param axis ID of the controller axis, starting at 0
     * @param power What power of exponential curve to apply. Will be clamped to the range [0, ∞)
     * @param val Discriminating value
     * @return Will return a Trigger that will always evaluate to false if the given controller or axis doesn't exist
     */
    public static Trigger getAxisExponentialLessThanTrigger(int controller, int axis, double power, double val) {
        if(!axisCheck(controller, axis)) return new Trigger(() -> false);
        
        double deadzone = controllers.get(controller).axisDeadzones.getOrDefault(controller, 0d);
        return new Trigger(() -> applyExponentialDeadzone(controllers.get(controller).hid.getRawAxis(axis), deadzone, power) < val);
    }
    
    /**
     * Get the angle of the POV stick
     * @param controller ID of the registered controller
     * @return the angle of the currently pressed pov button, -1 if none are pressed, and -2 if the given controller doesn't exist
     * @see edu.wpi.first.wpilibj.GenericHID#getPOV GenericHID.getPOV
     */
    public static int getPOVAngle(int controller) {
        if(!povCheck(controller)) return -2;
        
        Controller c = controllers.get(controller);
        if(c.povBuffer != -2) {
            return c.povBuffer;
        }
        
        int val = controllers.get(controller).hid.getPOV();
        c.povBuffer = val;
        return val;
    }
    /**
     * Test if the POV stick is currently pressed up
     * @param controller ID of the registered controller
     * @return Will return false if the given controller doesn't exist
     */
    public static boolean getPOVUp(int controller) {
        return getPOVAngle(controller) == 0;
    }
    /**
     * Test if the POV stick is currently pressed diagonally to the upper right
     * @param controller ID of the registered controller
     * @return Will return false if the given controller doesn't exist
     */
    public static boolean getPOVUpRight(int controller) {
        return getPOVAngle(controller) == 45;
    }
    /**
     * Test if the POV stick is currently pressed right
     * @param controller ID of the registered controller
     * @return Will return false if the given controller doesn't exist
     */
    public static boolean getPOVRight(int controller) {
        return getPOVAngle(controller) == 90;
    }
    /**
     * Test if the POV stick is currently pressed diagonally to the lower right
     * @param controller ID of the registered controller
     * @return Will return false if the given controller doesn't exist
     */
    public static boolean getPOVDownRight(int controller) {
        return getPOVAngle(controller) == 135;
    }
    /**
     * Test if the POV stick is currently pressed down
     * @param controller ID of the registered controller
     * @return Will return false if the given controller doesn't exist
     */
    public static boolean getPOVDown(int controller) {
        return getPOVAngle(controller) == 180;
    }
    /**
     * Test if the POV stick is currently pressed diagonally to the lower left
     * @param controller ID of the registered controller
     * @return Will return false if the given controller doesn't exist
     */
    public static boolean getPOVDownLeft(int controller) {
        return getPOVAngle(controller) == 225;
    }
    /**
     * Test if the POV stick is currently pressed left
     * @param controller ID of the registered controller
     * @return Will return false if the given controller doesn't exist
     */
    public static boolean getPOVLeft(int controller) {
        return getPOVAngle(controller) == 270;
    }
    /**
     * Test if the POV stick is currently pressed diagonally to the upper left
     * @param controller ID of the registered controller
     * @return Will return false if the given controller doesn't exist
     */
    public static boolean getPOVUpLeft(int controller) {
        return getPOVAngle(controller) == 315;
    }
    /**
     * Test if the POV stick is currently being pressed in any direction
     * @param controller ID of the registered controller
     * @return Will return false if the given controller doesn't exist
     */
    public static boolean getPOVAny(int controller) {
        return getPOVAngle(controller) != -1;
    }
    /**
     * Test if the POV stick isn't currently being pressed
     * @param controller ID of the registered controller
     * @return Will return true if the given controller doesn't exist
     */
    public static boolean getPOVNone(int controller) {
        return getPOVAngle(controller) == -1;
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.ControllerManager#getPOVUp getPOVUp}
     * @param controller ID of the registered controller
     * @return Will return a Trigger that will always evaluate to false if the given controller doesn't exist
     */
    public static Trigger getPOVUpTrigger(int controller) {
        if(!povCheck(controller)) return new Trigger(() -> false);
        
        return new Trigger(() -> controllers.get(controller).hid.getPOV() == 0);
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.ControllerManager#getPOVUpRight getPOVUpRight}
     * @param controller ID of the registered controller
     * @return Will return a Trigger that will always evaluate to false if the given controller doesn't exist
     */
    public static Trigger getPOVUpRightTrigger(int controller) {
        if(!povCheck(controller)) return new Trigger(() -> false);
        
        return new Trigger(() -> controllers.get(controller).hid.getPOV() == 45);
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.ControllerManager#getPOVRight getPOVRight}
     * @param controller ID of the registered controller
     * @return Will return a Trigger that will always evaluate to false if the given controller doesn't exist
     */
    public static Trigger getPOVRightTrigger(int controller) {
        if(!povCheck(controller)) return new Trigger(() -> false);
        
        return new Trigger(() -> controllers.get(controller).hid.getPOV() == 90);
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.ControllerManager#getPOVDownRight getPOVDownRight}
     * @param controller ID of the registered controller
     * @return Will return a Trigger that will always evaluate to false if the given controller doesn't exist
     */
    public static Trigger getPOVDownRightTrigger(int controller) {
        if(!povCheck(controller)) return new Trigger(() -> false);
        
        return new Trigger(() -> controllers.get(controller).hid.getPOV() == 135);
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.ControllerManager#getPOVDown getPOVDown}
     * @param controller ID of the registered controller
     * @return Will return a Trigger that will always evaluate to false if the given controller doesn't exist
     */
    public static Trigger getPOVDownTrigger(int controller) {
        if(!povCheck(controller)) return new Trigger(() -> false);
        
        return new Trigger(() -> controllers.get(controller).hid.getPOV() == 180);
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.ControllerManager#getPOVDownLeft getPOVDownLeft}
     * @param controller ID of the registered controller
     * @return Will return a Trigger that will always evaluate to false if the given controller doesn't exist
     */
    public static Trigger getPOVDownLeftTrigger(int controller) {
        if(!povCheck(controller)) return new Trigger(() -> false);
        
        return new Trigger(() -> controllers.get(controller).hid.getPOV() == 225);
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.ControllerManager#getPOVLeft getPOVLeft}
     * @param controller ID of the registered controller
     * @return Will return a Trigger that will always evaluate to false if the given controller doesn't exist
     */
    public static Trigger getPOVLeftTrigger(int controller) {
        if(!povCheck(controller)) return new Trigger(() -> false);
        
        return new Trigger(() -> controllers.get(controller).hid.getPOV() == 270);
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.ControllerManager#getPOVUpLeft getPOVUpLeft}
     * @param controller ID of the registered controller
     * @return Will return a Trigger that will always evaluate to false if the given controller doesn't exist
     */
    public static Trigger getPOVUpLeftTrigger(int controller) {
        if(!povCheck(controller)) return new Trigger(() -> false);
        
        return new Trigger(() -> controllers.get(controller).hid.getPOV() == 315);
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.ControllerManager#getPOVAny getPOVAny}
     * @param controller ID of the registered controller
     * @return Will return a Trigger that will always evaluate to false if the given controller doesn't exist
     */
    public static Trigger getPOVAnyTrigger(int controller) {
        if(!povCheck(controller)) return new Trigger(() -> false);
        
        return new Trigger(() -> controllers.get(controller).hid.getPOV() != -1);
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.ControllerManager#getPOVNone getPOVNone}
     * @param controller ID of the registered controller
     * @return Will return a Trigger that will always evaluate to true if the given controller doesn't exist
     */
    public static Trigger getPOVNoneTrigger(int controller) {
        if(!povCheck(controller)) return new Trigger(() -> true);
        
        return new Trigger(() -> controllers.get(controller).hid.getPOV() == -1);
    }

    /**
     * Schedule a controller rumble. ControllerManager will handle having multiple run at the same time by choosing the rumble that has the highest strength to run on the controller
     * @param controller ID of the registered controller
     * @param type Whether to activate the left, right, or both rumble motors
     * @param strength Magnitude of the rumble. Should be in the range of (0, 1], 1 being 100%. This value will be clamped within an acceptable range
     * @param duration How long this particular rumble should last. Should be in the range of (0, ∞)
     * @return Rumble ID. This can be used to cancel the rumble. This ID will be -1 if the given controller doesn't exist or if an invalid rumble is created
     */
    public static int scheduleRumble(int controller, RumbleType type, double strength, double duration) {
        if(!controllers.containsKey(controller)) {
            DriverStation.reportError(String.format("No controller with id %d has been registered", controller), false);
            return -1;
        }
        
        strength = Math.min(strength, 1);
        if(strength <= 0) {
            return -1; //return quietly
        }
        if(duration <= 0) {
            return -1; //return quietly
        }
        
        Controller c = controllers.get(controller);
        
        //probably could be better but it works
        //will get super slow if a ton of rumbles are active -> TODO add a limit on how many rumbles can be active at a time per controller, how to handle limit (either cancel old rumbles or cancel incoming)
        //also will break if 2147483647 rumbles are already active when scheduling a new one
        int id;
        do {
            id = (int) (Math.random() * Integer.MAX_VALUE);
        }
        while(c.rumbles.keySet().contains(id));
        
        Rumble r = new Rumble();
        r.type = type;
        r.strength = strength;
        r.duration = duration;
        r.timer = new Timer();
        r.timer.start();
        
        c.rumbles.put(id, r);
        
        switch(type) {
            case LEFT:
                c.leftMax = Math.max(c.leftMax, strength);
                c.hid.setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kLeftRumble, c.leftMax);
                break;
            case RIGHT:
                c.rightMax = Math.max(c.rightMax, strength);
                c.hid.setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kRightRumble, c.rightMax);
                break;
            case BOTH:
                c.leftMax = Math.max(c.leftMax, strength);
                c.rightMax = Math.max(c.rightMax, strength);
                c.hid.setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kLeftRumble, c.leftMax);
                c.hid.setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kRightRumble, c.rightMax);
                break;
        }
        
        return id;
    }
    /**
     * Cancel an already scheduled rumble
     * @param controller ID of the registered controller
     * @param id Rumble ID returned by {@link frc.robot.controllers.ControllerManager#scheduleRumble scheduleRumble}
     */
    public static void cancelRumble(int controller, int id) {
        if(!controllers.containsKey(controller)) {
            DriverStation.reportError(String.format("No controller with id %d has been registered", controller), false);
            return;
        }
        
        Controller c = controllers.get(controller);
        if(!c.rumbles.containsKey(id)) {
            DriverStation.reportError(String.format("No rumble with id %d is currently registered for controller %d", id, controller), false);
            return;
        }
        
        c.rumbles.remove(id);
    }
    /**
     * Cancel all active rumbles on the controller
     * @param controller ID of the registered controller
     */
    public static void cancelAllRumbles(int controller) {
        if(!controllers.containsKey(controller)) {
            DriverStation.reportError(String.format("No controller with id %d has been registered", controller), false);
            return;
        }
        
        Controller c = controllers.get(controller);
        c.rumbles.clear();
        c.leftMax = 0;
        c.rightMax = 0;
        c.hid.setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 0);
    }
    /**
     * Check if a rumble ID is currently valid
     * @param controller ID of the registered controller
     * @param id Rumble ID returned by {@link frc.robot.controllers.ControllerManager#scheduleRumble scheduleRumble}
     * @return Whether the ID is valid. Will return false if the given controller doesn't exist
     */
    public static boolean isRumbleIDValid(int controller, int id) {
        if(!controllers.containsKey(controller)) {
            DriverStation.reportError(String.format("No controller with id %d has been registered", controller), false);
            return false;
        }
        
        return controllers.get(controller).rumbles.containsKey(id);
    }
    /**
     * Get a Trigger that returns true when the given rumble ID is no longer valid, i.e. when the rumble ends
     * @param controller ID of the registered controller
     * @param id Rumble ID returned by {@link frc.robot.controllers.ControllerManager#scheduleRumble scheduleRumble}
     * @return Will return a Trigger that always evaluates to true if the given controller doesn't exist
     */
    public static Trigger rumbleEndTrigger(int controller, int id) {
        if(!controllers.containsKey(controller)) {
            DriverStation.reportError(String.format("No controller with id %d has been registered", controller), false);
            return new Trigger(() -> true);
        }
        
        return new Trigger(() -> !controllers.get(controller).rumbles.containsKey(id));
    }
}
