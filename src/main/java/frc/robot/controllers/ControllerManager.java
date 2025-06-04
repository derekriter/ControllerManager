package frc.robot.controllers;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
    }
    
    private static Map<Integer, Controller> controllers = new HashMap<>();
    
    /**
     * Create and register a HID device
     * @param id Which port to connect the hid device to. Should be in the range of 0-5
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
        
        Controller c = new Controller();
        c.hid = new GenericHID(id);
        
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
        
        Controller c= new Controller();
        c.hid = hid;
        
        controllers.put(id, c);
    }
    /**
     * Critical to call in {@link frc.robot.Robot#robotPeriodic() robotPeriodic}
     */
    public static void periodic() {
        forceClearBuffers();
    }
    
    /*
     * Internal tools
     */
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
    public static double applyLinearDeadzone(double val, double deadzone) {
        if(-deadzone <= val && val <= deadzone) return 0;
        
        return (val - (val > 0 ? deadzone : -deadzone)) / (1 - deadzone);
    }
    /*
     * https://www.desmos.com/calculator/07bcdud2oy
     */
    public static double applyExponentialDeadzone(double val, double deadzone, double power) {
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
    
    /*
     * Setters
     */
    /**
     * Forceable clear the input buffers. Doing so will cause the input to be requeried next time an input function is called. This is already called by {@link frc.robot.controllers.ControllerManager#periodic() periodic}. This should not need to be called manually under normal circumstances
     */
    public static void forceClearBuffers() {
        for(Controller c : controllers.values()) {
            c.buttonBuffer.clear();
            c.buttonPressedBuffer.clear();
            c.buttonReleasedBuffer.clear();
            c.axisBuffer.clear();
            c.povBuffer = -2;
        }
    }
    /**
     * Configure the deadzone for the given controller and axis. This deadzone is used when calling {@link frc.robot.controllers.ControllerManager#getAxisLinear getAxisLinear} and {@link frc.robot.controllers.ControllerManager#getAxisExponential(int, int, double) getAxisExponential}
     * @param controller ID of registered controller
     * @param axis ID of controller axis, starting at 0
     * @param deadzone The range in which if the absolute value of the axis is <= the deadzone, then it will equal 0
     */
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
     * @param power What power of exponential curve to apply. See variable <i>s</i> in the example graph
     * @return Will return 0 if the given controller or axis doesn't exist
     * @see https://www.desmos.com/calculator/07bcdud2oy
     */
    public static double getAxisExponential(int controller, int axis, double power) {
        if(!axisCheck(controller, axis)) return 0;
        
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
     * @param power What power of exponential curve to apply
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
     * @param power What power of exponential curve to apply
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
}
