package frc.robot.controllers;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;

/*
 * TODO Add controller binding constants -> Possibly rewrite to use controller classes rather than just one big abstract class
 * TODO Javadocs on github?
 */
/**
 * A utility to handle the finer details of controller input for you. This is an abstract class, all available functions are called statically
 */
public abstract class ControllerManager {
    
    /**
     * The default number of rumbles that are allowed to be active at once per controller
     * @see frc.robot.controllers.ControllerManager#setRumbleLimit
     * @see frc.robot.controllers.ControllerManager#MAX_RUMBLE_LIMIT
     */
    public static final int DEFAULT_RUMBLE_LIMIT = 10;
    /**
     * Max value for the rumble limit
     * @see frc.robot.controllers.ControllerManager#DEFAULT_RUMBLE_LIMIT
     * @see frc.robot.controllers.ControllerManager#setRumbleLimit
     */
    public static final int MAX_RUMBLE_LIMIT = 30;
    
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
    private static int rumbleLimit = DEFAULT_RUMBLE_LIMIT;
    
    /*
     * Internal
     */
    private static void periodic() {
        for(int i = 0; i < controllers.values().size(); i++) {
            forceClearBuffers(i);
            updateRumble(i);
        }
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
    
    private static void errorControllerCheck(int controller) throws IllegalArgumentException {
        if(controllers.containsKey(controller)) return;
        
        ControllerLogger.errorInvalidController(controller);
    }
    private static boolean warningControllerCheck(int controller) {
        if(controllers.containsKey(controller)) return true;
        
        ControllerLogger.warningInvalidController(controller);
        return false;
    }
    
    private static boolean buttonCheck(int controller, int button) {
        if(!warningControllerCheck(controller)) return false;
        
        Controller c = controllers.get(controller);
        //button ids start at 1
        if(button <= 0) {
            ControllerLogger.warningButtonUnderMin(button);
            return false;
        }
        if(button > c.hid.getButtonCount()) {
            ControllerLogger.warningButtonOverMax(controller, button, c.hid.getButtonCount());
            return false;
        }
        
        return true;
    }
    private static boolean axisCheck(int controller, int axis) {
        if(!warningControllerCheck(controller)) return false;
        
        Controller c = controllers.get(controller);
        //axis ids start at 0
        if(axis < 0) {
            ControllerLogger.warningAxisUnderMin(axis);
            return false;
        }
        if(axis >= c.hid.getAxisCount()) {
            ControllerLogger.warningAxisOverMax(controller, axis, c.hid.getAxisCount());
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
        if(!warningControllerCheck(controller)) return false;
        
        return true;
    }
    
    /*
     * Setters
     */
    /**
     * Setup the manager. This should be the first thing you call, before creating controllers
     * @param robot Your {@link frc.robot.Robot} instance
     * @throws IllegalArgumentException If robot is null
     */
    public static void init(Robot robot) throws IllegalArgumentException {
        if(robot == null) {
            ControllerLogger.errorNullArgument("robot");
        }
        
        //maybe shouldn't be assert, IDK
        assert DEFAULT_RUMBLE_LIMIT <= MAX_RUMBLE_LIMIT;
        
        hasInited = true;
        
        //run periodic at 50 Hz, same cycle as robotPeriodic
        robot.addPeriodic(ControllerManager::periodic, Robot.kDefaultPeriod);
    }
    /**
     * Create and register a HID device
     * @param id Which port to connect the hid device to. Should be in the range of [0, 5]
     * @throws IllegalStateException If ControllerManager has not been initialized
     * @throws IllegalArgumentException If a controller has already been registered with the given ID
     * @throws IllegalArgumentException If id is not within the allowed range
     */
    public static void registerController(int id) throws IllegalArgumentException, IllegalStateException {
        if(!hasInited) {
            ControllerLogger.errorHasNotInited();
        }
        if(controllers.containsKey(id)) {
            ControllerLogger.errorControllerAlreadyExists(id);
        }
        if(id < 0 || id > 5) {
            ControllerLogger.errorOutOfRange("id", id, "[0, 5]");
        }
        
        Controller c = new Controller();
        c.hid = new GenericHID(id);
        
        //reset rumbles just in case
        c.hid.setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 0);
        
        controllers.put(id, c);
    }
    /**
     * Register an already created HID device
     * @param hid An initialized GenericHID or GenericHID child such as {@link edu.wpi.first.wpilibj.Joystick}
     * @throws IllegalStateException If ControllerManager has not been initialized
     * @throws IllegalArgumentException If hid is null
     * @throws IllegalArgumentException If a controller has already been registered with the same ID as the given HID
     */
    public static void registerController(GenericHID hid) throws IllegalArgumentException, IllegalStateException {
        if(!hasInited) {
            ControllerLogger.errorHasNotInited();
        }
        if(hid == null) {
            ControllerLogger.errorNullArgument("hid");
        }
        
        int id = hid.getPort();
        if(controllers.containsKey(id)) {
            ControllerLogger.errorControllerAlreadyExists(id);
        }
        
        Controller c = new Controller();
        c.hid = hid;
        
        controllers.put(id, c);
    }
     /**
     * Forceably clear the input buffers of the given controller. Doing so will cause the input to be requeried next time an input function is called. This is already called in a 50 Hz periodic loop. This should not need to be called manually under normal circumstances
     * @param controller ID of a registered controller
     * @throws IllegalArgumentException If a controller with the given ID hasn't been registered
     */
    public static void forceClearBuffers(int controller) throws IllegalArgumentException {
        errorControllerCheck(controller);
        
        Controller c = controllers.get(controller);
        c.buttonBuffer.clear();
        c.buttonPressedBuffer.clear();
        c.buttonReleasedBuffer.clear();
        c.axisBuffer.clear();
        c.povBuffer = -2;
    }
    /**
     * Configure the deadzone for the given controller and axis. This deadzone is used when calling {@link frc.robot.controllers.ControllerManager#getAxisLinear} and {@link frc.robot.controllers.ControllerManager#getAxisExponential}
     * @param controller ID of a registered controller
     * @param axis ID of a controller axis, starting at 0
     * @param deadzone The range in which if the absolute value of the axis is <= the deadzone, then the axis will evaluate to 0. This value is clamped to the range [0, 1]
     * @throws IllegalArgumentException If a controller with the given ID hasn't been registered
     * @see frc.robot.controllers.ControllerManager#getAxisLinear
     * @see frc.robot.controllers.ControllerManager#getAxisExponential
     */
    public static void setControllerAxisDeadzone(int controller, int axis, double deadzone) throws IllegalArgumentException {
        errorControllerCheck(controller);
        
        double clampedDeadzone = MathUtil.clamp(deadzone, 0, 1);
        if(clampedDeadzone != deadzone) {
            ControllerLogger.warningClamp("deadzone", deadzone, "[0, 1]");
        }
        controllers.get(controller).axisDeadzones.put(axis, clampedDeadzone);
    }
    /**
     * This will configure how many scheduled rumbles ControllerManager will allow to be active at once on a per-controller basis. This can be set before initialization of ControllerManager. This defaults to {@link frc.robot.controllers.ControllerManager#DEFAULT_RUMBLE_LIMIT}. The rumble limit only affects newly scheduled rumbles, meaing decreasing this value below what is currently active will not cancel any rumbles. e.g. If 15 rumbles are currently active and you set the rumble limit to 10, then the extra 5 will continue to operate normally, as they were scheduled before the limit was decreased
     * @param maxRumbles Maximum amount of rumbles allowed to be active at once per controller. This value will be clamped to the range [0, {@link frc.robot.controllers.ControllerManager#MAX_RUMBLE_LIMIT}]
     * @see frc.robot.controllers.ControllerManager#DEFAULT_RUMBLE_LIMIT
     * @see frc.robot.controllers.ControllerManager#MAX_RUMBLE_LIMIT
     * @see frc.robot.controllers.ControllerManager#getCurrentRumbleLimit
     */
    public static void setRumbleLimit(int maxRumbles) {
        rumbleLimit = MathUtil.clamp(maxRumbles, 0, MAX_RUMBLE_LIMIT);
        
        if(rumbleLimit != maxRumbles) {
            ControllerLogger.warningClamp("maxRumbles", maxRumbles, String.format("[0, %d]", MAX_RUMBLE_LIMIT));
        }
    }
    /**
     * Change the level of logging verbosity. This can be set before initialization of ControllerManager
     * @param verbosity Amount of logging to be put into the console
     */
    public static void setVerbosity(LoggingVerbosity verbosity) {
        ControllerLogger.setVerbosity(verbosity);
    }
    
    /*
     * Getters
     */
    /**
     * Get the interal HID device for the given controller
     * @param controller ID of a registered controller
     * @return Will return null if the given controller doesn't exist
     * @throws IllegalArgumentException If a controller with the given ID hasn't been registered
     */
    public static GenericHID getHID(int controller) throws IllegalArgumentException {
        errorControllerCheck(controller);
        
        return controllers.get(controller).hid;
    }
    /**
     * Get the currently configured maximum rumbles. This will default to {@link frc.robot.controllers.ControllerManager#DEFAULT_RUMBLE_LIMIT ControllerManager.DEFAULT_RUMBLE_LIMIT}
     * @see frc.robot.controllers.ControllerManager#DEFAULT_RUMBLE_LIMIT ControllerManager.DEFAULT_RUMBLE_LIMIT
     * @see frc.robot.controllers.ControllerManager#setRumbleLimit ControllerManager.setRumbleLimit
     */
    public static int getCurrentRumbleLimit() {
        return rumbleLimit;
    }
    /**
     * Get the currently active level of logging verbosity
     */
    public static LoggingVerbosity getVerbosity() {
        return ControllerLogger.getVerbosity();
    }
    
    /**
     * Get whether a button is currently pressed or not
     * @param controller ID of a registered controller
     * @param button ID of a controller button, starting at 1
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
     * @param controller ID of a registered controller
     * @param button ID of a controller button, starting at 1
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
     * @param controller ID of a registered controller
     * @param button ID of a controller button, starting at 1
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
     * Get a trigger that tracks the value of {@link frc.robot.controllers.ControllerManager#getButton}. <b>Do not</b> use this function to get the value of a button as a boolean. Use {@link frc.robot.controllers.ControllerManager#getButton} for that.
     * @param controller ID of a registered controller
     * @param button ID of a controller button, starting at 1
     * @return Will return a Trigger that will always evaluate to false if the given controller or button doesn't exist
     * @see https://github.com/wpilibsuite/allwpilib/issues/5903
     * @see frc.robot.controllers.ControllerManager#getButton
     */
    public static Trigger getButtonTrigger(int controller, int button) {
        if(!buttonCheck(controller, button)) return new Trigger(() -> false);
        
        //directly use getRawButton rather than getButton to prevent a bunch of unneccessary checks
        //don't use buffer because it might just cause more problems
        return new Trigger(() -> controllers.get(controller).hid.getRawButton(button));
    }
    
    /**
     * Get the axis value without applying deadzone configurations
     * @param controller ID of a registered controller
     * @param axis ID of a controller axis, starting at 0
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
     * @param controller ID of a registered controller
     * @param axis ID of a controller axis, starting at 0
     * @return Will return 0 if the given controller or axis doesn't exist
     * @see https://www.desmos.com/calculator/07bcdud2oy
     * @see frc.robot.controllers.ControllerManager#setControllerAxisDeadzone
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
     * @param controller ID of a registered controller
     * @param axis ID of a controller axis, starting at 0
     * @param power What power of exponent to apply. Will be clamped to the range [0, ∞) See variable <i>s</i> in the example graph
     * @return Will return 0 if the given controller or axis doesn't exist
     * @see https://www.desmos.com/calculator/07bcdud2oy
     * @see frc.robot.controllers.ControllerManager#setControllerAxisDeadzone
     */
    public static double getAxisExponential(int controller, int axis, double power) {
        if(!axisCheck(controller, axis)) return 0;
        
        Controller c = controllers.get(controller);
        double deadzone = c.axisDeadzones.getOrDefault(axis, 0d);
        double clampedPower = Math.max(power, 0);
        if(clampedPower != power) {
            ControllerLogger.warningClamp("power", power, "[0, ∞)");
        }
        
        if(c.axisBuffer.containsKey(axis)) {
            return applyExponentialDeadzone(c.axisBuffer.get(axis), deadzone, clampedPower);
        }
        
        double val = c.hid.getRawAxis(axis);
        c.axisBuffer.put(axis, val);
        return applyExponentialDeadzone(val, deadzone, clampedPower);
    }
    /**
     * Get if the raw axis value is greater than the given value
     * @param controller ID of a registered controller
     * @param axis ID of a controller axis, starting at 0
     * @param val Discriminating value
     * @return Will return false if the given controller or axis doesn't exist
     * @see frc.robot.controllers.ControllerManager#getAxisRaw
     */
    public static boolean getAxisRawGreaterThan(int controller, int axis, double val) {
        return getAxisRaw(controller, axis) > val;
    }
    /**
     * Get if the raw axis value is less than the given value
     * @param controller ID of a registered controller
     * @param axis ID of a controller axis, starting at 0
     * @param val Discriminating value
     * @return Will return false if the given controller or axis doesn't exist
     * @see frc.robot.controllers.ControllerManager#getAxisRaw
     */
    public static boolean getAxisRawLessThan(int controller, int axis, double val) {
        if(!axisCheck(controller, axis)) return false;
        
        return getAxisRaw(controller, axis) < val;
    }
    /**
     * Get if the linearly calibrated axis value is greater than the given value
     * @param controller ID of a registered controller
     * @param axis ID of a controller axis, starting at 0
     * @param val Discriminating value
     * @return Will return false if the given controller or axis doesn't exist
     * @see frc.robot.controllers.ControllerManager#getAxisLinear
     */
    public static boolean getAxisLinearGreaterThan(int controller, int axis, double val) {
        return getAxisLinear(controller, axis) > val;
    }
    /**
     * Get if the linearly calibrated axis value is less than the given value
     * @param controller ID of a registered controller
     * @param axis ID of a controller axis, starting at 0
     * @param val Discriminating value
     * @return Will return false if the given controller or axis doesn't exist
     * @see frc.robot.controllers.ControllerManager#getAxisLinear
     */
    public static boolean getAxisLinearLessThan(int controller, int axis, double val) {
        if(!axisCheck(controller, axis)) return false;
        
        return getAxisLinear(controller, axis) < val;
    }
    /**
     * Get if the exponentially calibrated axis value is greater than the given value
     * @param controller ID of a registered controller
     * @param axis ID of a controller axis, starting at 0
     * @param power What power of exponential curve to apply
     * @param val Discriminating value
     * @return Will return false if the given controller or axis doesn't exist
     * @see frc.robot.controllers.ControllerManager#getAxisExponential
     */
    public static boolean getAxisExponentialGreaterThan(int controller, int axis, double power, double val) {
        return getAxisExponential(controller, axis, power) > val;
    }
    /**
     * Get if the linearly calibrated axis value is less than the given value
     * @param controller ID of a registered controller
     * @param axis ID of a controller axis, starting at 0
     * @param power What power of exponential curve to apply
     * @param val Discriminating value
     * @return Will return false if the given controller or axis doesn't exist
     * @see frc.robot.controllers.ControllerManager#getAxisExponential
     */
    public static boolean getAxisExponentialLessThan(int controller, int axis, double power, double val) {
        if(!axisCheck(controller, axis)) return false;
        
        return getAxisExponential(controller, axis, power) < val;
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.ControllerManager#getAxisRawGreaterThan}
     * @param controller ID of a registered controller
     * @param axis ID of a controller axis, starting at 0
     * @param val Discriminating value
     * @return Will return a Trigger that will always evaluate to false if the given controller or axis doesn't exist
     * @see frc.robot.controllers.ControllerManager#getAxisRawGreaterThan
     */
    public static Trigger getAxisRawGreaterThanTrigger(int controller, int axis, double val) {
        if(!axisCheck(controller, axis)) return new Trigger(() -> false);
        
        return new Trigger(() -> controllers.get(controller).hid.getRawAxis(axis) > val);
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.ControllerManager#getAxisRawLessThan}
     * @param controller ID of a registered controller
     * @param axis ID of a controller axis, starting at 0
     * @param val Discriminating value
     * @return Will return a Trigger that will always evaluate to false if the given controller or axis doesn't exist
     * @see frc.robot.controllers.ControllerManager#getAxisRawLessThan
     */
    public static Trigger getAxisRawLessThanTrigger(int controller, int axis, double val) {
        if(!axisCheck(controller, axis)) return new Trigger(() -> false);
        
        return new Trigger(() -> controllers.get(controller).hid.getRawAxis(axis) < val);
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.ControllerManager#getAxisLinearGreaterThan}
     * @param controller ID of a registered controller
     * @param axis ID of a controller axis, starting at 0
     * @param val Discriminating value
     * @return Will return a Trigger that will always evaluate to false if the given controller or axis doesn't exist
     * @see frc.robot.controllers.ControllerManager#getAxisLinearGreaterThan
     */
    public static Trigger getAxisLinearGreaterThanTrigger(int controller, int axis, double val) {
        if(!axisCheck(controller, axis)) return new Trigger(() -> false);
        
        double deadzone = controllers.get(controller).axisDeadzones.getOrDefault(controller, 0d);
        return new Trigger(() -> applyLinearDeadzone(controllers.get(controller).hid.getRawAxis(axis), deadzone) > val);
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.ControllerManager#getAxisLinearLessThan}
     * @param controller ID of a registered controller
     * @param axis ID of a controller axis, starting at 0
     * @param val Discriminating value
     * @return Will return a Trigger that will always evaluate to false if the given controller or axis doesn't exist
     * @see frc.robot.controllers.ControllerManager#getAxisLinearLessThan
     */
    public static Trigger getAxisLinearLessThanTrigger(int controller, int axis, double val) {
        if(!axisCheck(controller, axis)) return new Trigger(() -> false);
        
        double deadzone = controllers.get(controller).axisDeadzones.getOrDefault(controller, 0d);
        return new Trigger(() -> applyLinearDeadzone(controllers.get(controller).hid.getRawAxis(axis), deadzone) < val);
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.ControllerManager#getAxisExponentialGreaterThan}
     * @param controller ID of a registered controller
     * @param axis ID of a controller axis, starting at 0
     * @param power What power of exponential curve to apply. Will be clamped to the range [0, ∞)
     * @param val Discriminating value
     * @return Will return a Trigger that will always evaluate to false if the given controller or axis doesn't exist
     * @see frc.robot.controllers.ControllerManager#getAxisExponentialGreaterThan
     */
    public static Trigger getAxisExponentialGreaterThanTrigger(int controller, int axis, double power, double val) {
        if(!axisCheck(controller, axis)) return new Trigger(() -> false);
        
        double deadzone = controllers.get(controller).axisDeadzones.getOrDefault(controller, 0d);
        return new Trigger(() -> applyExponentialDeadzone(controllers.get(controller).hid.getRawAxis(axis), deadzone, power) > val);
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.ControllerManager#getAxisExponentialLessThan}
     * @param controller ID of a registered controller
     * @param axis ID of a controller axis, starting at 0
     * @param power What power of exponential curve to apply. Will be clamped to the range [0, ∞)
     * @param val Discriminating value
     * @return Will return a Trigger that will always evaluate to false if the given controller or axis doesn't exist
     * @see frc.robot.controllers.ControllerManager#getAxisExponentialLessThan
     */
    public static Trigger getAxisExponentialLessThanTrigger(int controller, int axis, double power, double val) {
        if(!axisCheck(controller, axis)) return new Trigger(() -> false);
        
        double deadzone = controllers.get(controller).axisDeadzones.getOrDefault(controller, 0d);
        return new Trigger(() -> applyExponentialDeadzone(controllers.get(controller).hid.getRawAxis(axis), deadzone, power) < val);
    }
    
    /**
     * Get the angle in degrees of the POV stick
     * @param controller ID of a registered controller
     * @return the angle of the currently pressed pov button, or -1 if it isn't pressed or if the given controller doesn't exist
     * @see edu.wpi.first.wpilibj.GenericHID#getPOV
     */
    public static int getPOVAngle(int controller) {
        if(!povCheck(controller)) return -1;
        
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
     * @param controller ID of a registered controller
     * @return Will return false if the given controller doesn't exist
     * @see frc.robot.controllers.ControllerManager#getPOVAngle
     */
    public static boolean getPOVUp(int controller) {
        return getPOVAngle(controller) == 0;
    }
    /**
     * Test if the POV stick is currently pressed diagonally to the upper right
     * @param controller ID of a registered controller
     * @return Will return false if the given controller doesn't exist
     * @see frc.robot.controllers.ControllerManager#getPOVAngle
     */
    public static boolean getPOVUpRight(int controller) {
        return getPOVAngle(controller) == 45;
    }
    /**
     * Test if the POV stick is currently pressed right
     * @param controller ID of a registered controller
     * @return Will return false if the given controller doesn't exist
     * @see frc.robot.controllers.ControllerManager#getPOVAngle
     */
    public static boolean getPOVRight(int controller) {
        return getPOVAngle(controller) == 90;
    }
    /**
     * Test if the POV stick is currently pressed diagonally to the lower right
     * @param controller ID of a registered controller
     * @return Will return false if the given controller doesn't exist
     * @see frc.robot.controllers.ControllerManager#getPOVAngle
     */
    public static boolean getPOVDownRight(int controller) {
        return getPOVAngle(controller) == 135;
    }
    /**
     * Test if the POV stick is currently pressed down
     * @param controller ID of a registered controller
     * @return Will return false if the given controller doesn't exist
     * @see frc.robot.controllers.ControllerManager#getPOVAngle
     */
    public static boolean getPOVDown(int controller) {
        return getPOVAngle(controller) == 180;
    }
    /**
     * Test if the POV stick is currently pressed diagonally to the lower left
     * @param controller ID of a registered controller
     * @return Will return false if the given controller doesn't exist
     * @see frc.robot.controllers.ControllerManager#getPOVAngle
     */
    public static boolean getPOVDownLeft(int controller) {
        return getPOVAngle(controller) == 225;
    }
    /**
     * Test if the POV stick is currently pressed left
     * @param controller ID of a registered controller
     * @return Will return false if the given controller doesn't exist
     * @see frc.robot.controllers.ControllerManager#getPOVAngle
     */
    public static boolean getPOVLeft(int controller) {
        return getPOVAngle(controller) == 270;
    }
    /**
     * Test if the POV stick is currently pressed diagonally to the upper left
     * @param controller ID of a registered controller
     * @return Will return false if the given controller doesn't exist
     * @see frc.robot.controllers.ControllerManager#getPOVAngle
     */
    public static boolean getPOVUpLeft(int controller) {
        return getPOVAngle(controller) == 315;
    }
    /**
     * Test if the POV stick is currently being pressed in any direction
     * @param controller ID of a registered controller
     * @return Will return false if the given controller doesn't exist
     * @see frc.robot.controllers.ControllerManager#getPOVAngle
     */
    public static boolean getPOVAny(int controller) {
        return getPOVAngle(controller) != -1;
    }
    /**
     * Test if the POV stick isn't currently being pressed
     * @param controller ID of a registered controller
     * @return Will return true if the given controller doesn't exist
     * @see frc.robot.controllers.ControllerManager#getPOVAngle
     */
    public static boolean getPOVNone(int controller) {
        return getPOVAngle(controller) == -1;
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.ControllerManager#getPOVUp}
     * @param controller ID of a registered controller
     * @return Will return a Trigger that will always evaluate to false if the given controller doesn't exist
     * @see frc.robot.controllers.ControllerManager#getPOVUp
     */
    public static Trigger getPOVUpTrigger(int controller) {
        if(!povCheck(controller)) return new Trigger(() -> false);
        
        return new Trigger(() -> controllers.get(controller).hid.getPOV() == 0);
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.ControllerManager#getPOVUpRight}
     * @param controller ID of a registered controller
     * @return Will return a Trigger that will always evaluate to false if the given controller doesn't exist
     * @see frc.robot.controllers.ControllerManager#getPOVUpRight
     */
    public static Trigger getPOVUpRightTrigger(int controller) {
        if(!povCheck(controller)) return new Trigger(() -> false);
        
        return new Trigger(() -> controllers.get(controller).hid.getPOV() == 45);
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.ControllerManager#getPOVRight}
     * @param controller ID of a registered controller
     * @return Will return a Trigger that will always evaluate to false if the given controller doesn't exist
     * @see frc.robot.controllers.ControllerManager#getPOVRight
     */
    public static Trigger getPOVRightTrigger(int controller) {
        if(!povCheck(controller)) return new Trigger(() -> false);
        
        return new Trigger(() -> controllers.get(controller).hid.getPOV() == 90);
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.ControllerManager#getPOVDownRight}
     * @param controller ID of a registered controller
     * @return Will return a Trigger that will always evaluate to false if the given controller doesn't exist
     * @see frc.robot.controllers.ControllerManager#getPOVDownRight
     */
    public static Trigger getPOVDownRightTrigger(int controller) {
        if(!povCheck(controller)) return new Trigger(() -> false);
        
        return new Trigger(() -> controllers.get(controller).hid.getPOV() == 135);
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.ControllerManager#getPOVDown}
     * @param controller ID of a registered controller
     * @return Will return a Trigger that will always evaluate to false if the given controller doesn't exist
     * @see frc.robot.controllers.ControllerManager#getPOVDown
     */
    public static Trigger getPOVDownTrigger(int controller) {
        if(!povCheck(controller)) return new Trigger(() -> false);
        
        return new Trigger(() -> controllers.get(controller).hid.getPOV() == 180);
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.ControllerManager#getPOVDownLeft}
     * @param controller ID of a registered controller
     * @return Will return a Trigger that will always evaluate to false if the given controller doesn't exist
     * @see frc.robot.controllers.ControllerManager#getPOVDownLeft
     */
    public static Trigger getPOVDownLeftTrigger(int controller) {
        if(!povCheck(controller)) return new Trigger(() -> false);
        
        return new Trigger(() -> controllers.get(controller).hid.getPOV() == 225);
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.ControllerManager#getPOVLeft}
     * @param controller ID of a registered controller
     * @return Will return a Trigger that will always evaluate to false if the given controller doesn't exist
     * @see frc.robot.controllers.ControllerManager#getPOVLeft
     */
    public static Trigger getPOVLeftTrigger(int controller) {
        if(!povCheck(controller)) return new Trigger(() -> false);
        
        return new Trigger(() -> controllers.get(controller).hid.getPOV() == 270);
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.ControllerManager#getPOVUpLeft}
     * @param controller ID of a registered controller
     * @return Will return a Trigger that will always evaluate to false if the given controller doesn't exist
     * @see frc.robot.controllers.ControllerManager#getPOVUpLeft
     */
    public static Trigger getPOVUpLeftTrigger(int controller) {
        if(!povCheck(controller)) return new Trigger(() -> false);
        
        return new Trigger(() -> controllers.get(controller).hid.getPOV() == 315);
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.ControllerManager#getPOVAny}
     * @param controller ID of a registered controller
     * @return Will return a Trigger that will always evaluate to false if the given controller doesn't exist
     * @see frc.robot.controllers.ControllerManager#getPOVAny
     */
    public static Trigger getPOVAnyTrigger(int controller) {
        if(!povCheck(controller)) return new Trigger(() -> false);
        
        return new Trigger(() -> controllers.get(controller).hid.getPOV() != -1);
    }
    /**
     * Get a trigger that tracks the value of {@link frc.robot.controllers.ControllerManager#getPOVNone}
     * @param controller ID of a registered controller
     * @return Will return a Trigger that will always evaluate to true if the given controller doesn't exist
     * @see frc.robot.controllers.ControllerManager#getPOVNone
     */
    public static Trigger getPOVNoneTrigger(int controller) {
        if(!povCheck(controller)) return new Trigger(() -> true);
        
        return new Trigger(() -> controllers.get(controller).hid.getPOV() == -1);
    }

    /**
     * Schedule a controller rumble. ControllerManager will handle having multiple run at the same time by choosing the rumble that has the highest strength to run on the controller
     * @param controller ID of a registered controller
     * @param type Whether to activate the left, right, or both rumble motors
     * @param strength Magnitude of the rumble. Should be in the range of (0, 1], 1 being 100%. This value will be clamped within the acceptable range
     * @param duration How long this particular rumble should last in seconds. Should be in the range of (0, ∞). This value will be clamped within the acceptable range
     * @return Rumble ID. This can be used to cancel the rumble. This ID will be -1 if the rumble couldn't be scheduled because the rumble limit has been reached
     * @throws IllegalArgumentException If a controller with the given ID hasn't been registered
     * @see frc.robot.controllers.ControllerManager#cancelRumble
     * @see frc.robot.controllers.ControllerManager#cancelAllRumbles
     * @see frc.robot.controllers.ControllerManager#rumbleEndTrigger
     * @see frc.robot.controllers.ControllerManager#DEFAULT_RUMBLE_LIMIT
     * @see frc.robot.controllers.ControllerManager#setRumbleLimit
     * @see frc.robot.controllers.ControllerManager#getCurrentRumbleLimit
     */
    public static int scheduleRumble(int controller, RumbleType type, double strength, double duration) throws IllegalArgumentException {
        errorControllerCheck(controller);
        
        double clampedStrength = MathUtil.clamp(strength, Double.MIN_NORMAL, 1);
        if(clampedStrength != strength) {
            ControllerLogger.warningClamp("strength", strength, "(0, 1]");
        }
        double clampedDuration = Math.max(duration, Double.MIN_NORMAL);
        if(clampedDuration != duration) {
            ControllerLogger.warningClamp("duration", duration, "(0, ∞)");
        }
        
        Controller c = controllers.get(controller);
        
        if(c.rumbles.size() == rumbleLimit) {
            ControllerLogger.warningReachedRumbleLimit(rumbleLimit);
            return -1;
        }
        
        //pick random id that isn't already being used
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
     * Cancel an already active rumble
     * @param controller ID of a registered controller
     * @param rumble Rumble ID returned by {@link frc.robot.controllers.ControllerManager#scheduleRumble}
     * @throws IllegalArgumentException If a controller with the given ID hasn't been registered
     * @throws IllegalArgumentException If a rumble with the given ID isn't currently active
     * @see frc.robot.controllers.ControllerManager#scheduleRumble
     * @see frc.robot.controllers.ControllerManager#isRumbleIDValid
     */
    public static void cancelRumble(int controller, int rumble) throws IllegalAccessException {
        errorControllerCheck(controller);
    
        Controller c = controllers.get(controller);
        if(!c.rumbles.containsKey(rumble)) {
            ControllerLogger.errorInvalidRumble(controller, rumble);
        }
        
        c.rumbles.remove(rumble);
        updateRumble(controller); //manually update rumble for immediate cancel
    }
    /**
     * Cancel all active rumbles on the controller
     * @param controller ID of a registered controller
     * @throws IllegalArgumentException If a controller with the given ID hasn't been registered
     * @see frc.robot.controllers.ControllerManager#scheduleRumble
     */
    public static void cancelAllRumbles(int controller) throws IllegalArgumentException {
        errorControllerCheck(controller);
        
        Controller c = controllers.get(controller);
        c.rumbles.clear();
        c.leftMax = 0;
        c.rightMax = 0;
        c.hid.setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 0);
    }
    /**
     * Check if a rumble ID is currently valid
     * @param controller ID of the registered controller
     * @param rumble Rumble ID returned by {@link frc.robot.controllers.ControllerManager#scheduleRumble}
     * @return Whether the ID is valid
     * @throws IllegalArgumentException If a controller with the given ID hasn't been registered
     * @see frc.robot.controllers.ControllerManager#scheduleRumble
     */
    public static boolean isRumbleIDValid(int controller, int rumble) throws IllegalArgumentException {
        errorControllerCheck(controller);
        
        return controllers.get(controller).rumbles.containsKey(rumble);
    }
    /**
     * Get a Trigger that returns true when the given rumble ID is no longer valid, i.e. when the rumble ends
     * @param controller ID of the registered controller
     * @param rumble Rumble ID returned by {@link frc.robot.controllers.ControllerManager#scheduleRumble}
     * @return A Trigger that tracks the value of {@link frc.robot.controllers.ControllerManager#isRumbleIDValid}
     * @throws IllegalArgumentException If a controller with the given ID hasn't been registered
     * @see frc.robot.controllers.ControllerManager#scheduleRumble
     * @see frc.robot.controllers.ControllerManager#isRumbleIDValid
     */
    public static Trigger rumbleEndTrigger(int controller, int rumble) throws IllegalArgumentException {
        errorControllerCheck(controller);
        
        return new Trigger(() -> !controllers.get(controller).rumbles.containsKey(rumble));
    }
}
