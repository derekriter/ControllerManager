package frc.robot.controllers;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
    }
    
    private static Map<Integer, Controller> controllers = new HashMap<>();
    
    /**
     * Create and register a HID device
     * @param id Which port to connect the hid device to. Should be in the range of 0-5
     */
    public static void createController(int id) {
        if(controllers.containsKey(id)) {
            DriverStation.reportError(String.format("Cannot create multiple controllers with id %d", id), false);
            return;
        }
        if(id < 0) {
            DriverStation.reportError("Cannot create a controller with negative id", false);
            return;
        }
        if(id > 5) {
            DriverStation.reportWarning(String.format("The driver station only supports controller ids 0-5, id %d will be inaccessible", id), false);
        }
        
        Controller c = new Controller();
        c.hid = new GenericHID(id);
        
        controllers.put(id, c);
    }
    /**
     * Register an already created HID device
     * @param hid
     */
    public static void addController(GenericHID hid) {
        int id = hid.getPort();
        if(controllers.containsKey(id)) {
            DriverStation.reportError(String.format("Cannot create multiple controllers with id %d", id), false);
            return;
        }
        
        Controller c= new Controller();
        c.hid = hid;
        
        controllers.put(id, c);
    }
    /**
     * Critical to call in robotPeriodic
     */
    public static void periodic() {
        forceClearBuffers();
    }
    
    /*
     * Internal tools
     */
    private static boolean getButtonCheck(int controller, int button) {
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
    private static boolean getAxisCheck(int controller, int axis) {
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
     * https://www.desmos.com/calculator/xwmslpzd1a
     */
    public static double applyLinearDeadzone(double val, double deadzone) {
        if(-deadzone <= val && val <= deadzone) return 0;
        
        return (val - (val > 0 ? deadzone : -deadzone)) / (1 - deadzone);
    }
    /*
     * https://www.desmos.com/calculator/xwmslpzd1a
     */
    public static double applyExponentialDeadzone(double val, double deadzone, double power) {
        if(-deadzone <= val && val <= deadzone) return 0;
        
        return Math.pow(Math.abs((val - (val > 0 ? deadzone : -deadzone)) / (1 - deadzone)), power) * (val < 0 ? -1 : 1);
    }
    
    /*
     * Setters
     */
    public static void forceClearBuffers() {
        for(Controller c : controllers.values()) {
            c.buttonBuffer.clear();
            c.buttonPressedBuffer.clear();
            c.buttonReleasedBuffer.clear();
            c.axisBuffer.clear();
        }
    }
    public static void setControllerAxisDeadzone(int controller, int axis, double deadzone) {
        if(!controllers.containsKey(controller)) {
            DriverStation.reportError(String.format("No controller with id %d has been registered", controller), false);
            return;
        }
        if(deadzone < 0) {
            DriverStation.reportError(String.format("Cannot apply negative deadzone to axis %d of controller %d", axis, controller), false);
            return;
        }
        if(deadzone >= 1) {
            DriverStation.reportWarning(String.format("A deadzone of >= 1 on axis %d of controller %d will disable that axis", axis, controller), false);
        }
        
        controllers.get(controller).axisDeadzones.put(axis, deadzone);
    }
    
    /*
     * Getters
     */
    public static boolean getButton(int controller, int button) {
        if(!getButtonCheck(controller, button)) return false;
        
        Controller c = controllers.get(controller);
        if(c.buttonBuffer.containsKey(button)) {
            return c.buttonBuffer.get(button);
        }
        
        boolean val = controllers.get(controller).hid.getRawButton(button);
        c.buttonBuffer.put(button, val);
        return val;
    }
    public static boolean getButtonPressed(int controller, int button) {
        if(!getButtonCheck(controller, button)) return false;
        
        Controller c = controllers.get(controller);
        if(c.buttonPressedBuffer.containsKey(button)) {
            return c.buttonPressedBuffer.get(button);
        }
        
        boolean val = controllers.get(controller).hid.getRawButtonPressed(button);
        c.buttonPressedBuffer.put(button, val);
        return val;
    }
    public static boolean getButtonReleased(int controller, int button) {
        if(!getButtonCheck(controller, button)) return false;
        
        Controller c = controllers.get(controller);
        if(c.buttonReleasedBuffer.containsKey(button)) {
            return c.buttonReleasedBuffer.get(button);
        }
        
        boolean val = controllers.get(controller).hid.getRawButtonReleased(button);
        c.buttonReleasedBuffer.put(button, val);
        return val;
    }
    /**
     * Get a trigger that tracks the value of getButton. Do not use this function to get the value of a button. Use getButton for that.
     * @param controller ID of the registered controller
     * @param button ID of the controller button, starting at 0
     * @return Will return an axis that will always evaluate to false if the given controller or button doesn't exist
     * @see https://github.com/wpilibsuite/allwpilib/issues/5903
     */
    public static Trigger getButtonTrigger(int controller, int button) {
        if(!getButtonCheck(controller, button)) return new Trigger(() -> false);
        
        //directly use getRawButton rather than getButton to prevent a bunch of unneccessary checks
        return new Trigger(() -> controllers.get(controller).hid.getRawButton(button));
    }
    
    /**
     * Get the axis value without applying deadzone configurations
     * @param controller ID of registered controller
     * @param axis ID of controller axis, starting at 1
     * @return Will return 0 if the given controller or axis doesn't exist
     */
    public static double getAxisRaw(int controller, int axis) {
        if(!getAxisCheck(controller, axis)) return 0;
        
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
     * @param axis ID of controller axis, starting at 1
     * @return Will return 0 if the given controller or axis doesn't exist
     * @see https://www.desmos.com/calculator/07bcdud2oy
     */
    public static double getAxisLinear(int controller, int axis) {
        if(!getAxisCheck(controller, axis)) return 0;
        
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
     * @param axis ID of controller axis, starting at 1
     * @param power What power of exponential curve to apply. See variable <i>s</i> in the example graph
     * @return Will return 0 if the given controller or axis doesn't exist
     * @see https://www.desmos.com/calculator/07bcdud2oy
     */
    public static double getAxisExponential(int controller, int axis, double power) {
        if(!getAxisCheck(controller, axis)) return 0;
        
        if(power < 0) {
            DriverStation.reportWarning("Using a negative exponent on an axis will result in weird behaviour", false);
        }
        
        Controller c = controllers.get(controller);
        double deadzone = c.axisDeadzones.getOrDefault(axis, 0d);
        if(c.axisBuffer.containsKey(axis)) {
            return applyExponentialDeadzone(c.axisBuffer.get(axis), deadzone, power);
        }
        
        double val = c.hid.getRawAxis(axis);
        c.axisBuffer.put(axis, val);
        return applyExponentialDeadzone(val, deadzone, power);
    }
}
