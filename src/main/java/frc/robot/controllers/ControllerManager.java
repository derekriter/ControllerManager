package frc.robot.controllers;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public abstract class ControllerManager {
    
    private static class Controller {
        public GenericHID hid;
        public Map<Integer, Double> axisDeadzones = new HashMap<>();
        
        public Map<Integer, Boolean> getButtonBuffer = new HashMap<>();
        public Map<Integer, Boolean> getButtonPressedBuffer = new HashMap<>();
        public Map<Integer, Boolean> getButtonReleasedBuffer = new HashMap<>();
    }
    
    private static Map<Integer, Controller> controllers = new HashMap<>();
    
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
     * Critical to call. Failing to do so will break input buffers
     */
    public static void periodic() {
        for(Controller c : controllers.values()) {
            c.getButtonBuffer.clear();
            c.getButtonPressedBuffer.clear();
            c.getButtonReleasedBuffer.clear();
        }
    }
    
    /*
     * Internal tools
     */
    private static boolean getButtonCheck(int controller, int button) {
        if(!controllers.containsKey(controller)) {
            DriverStation.reportWarning(String.format("No controller with id %d has been created", controller), false);
            return false;
        }
        
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
    
    /*
     * Setters
     */
    public static void setControllerAxisDeadzone(int controller, int axis, double deadzone) {
        if(!controllers.containsKey(controller)) {
            DriverStation.reportError(String.format("No controller with id %d has been created", controller), false);
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
        if(c.getButtonBuffer.containsKey(button)) {
            return c.getButtonBuffer.get(button);
        }
        
        boolean val = controllers.get(controller).hid.getRawButton(button);
        c.getButtonBuffer.put(button, val);
        return val;
    }
    public static boolean getButtonPressed(int controller, int button) {
        if(!getButtonCheck(controller, button)) return false;
        
        Controller c = controllers.get(controller);
        if(c.getButtonPressedBuffer.containsKey(button)) {
            return c.getButtonPressedBuffer.get(button);
        }
        
        boolean val = controllers.get(controller).hid.getRawButtonPressed(button);
        c.getButtonPressedBuffer.put(button, val);
        return val;
    }
    public static boolean getButtonReleased(int controller, int button) {
        if(!getButtonCheck(controller, button)) return false;
        
        Controller c = controllers.get(controller);
        if(c.getButtonReleasedBuffer.containsKey(button)) {
            return c.getButtonReleasedBuffer.get(button);
        }
        
        boolean val = controllers.get(controller).hid.getRawButtonReleased(button);
        c.getButtonReleasedBuffer.put(button, val);
        return val;
    }
    /**
     * Do not use this function to get the value of a button. Use getButton for that.
     */
    public static Trigger getButtonTrigger(int controller, int button) {
        if(!getButtonCheck(controller, button)) return new Trigger(() -> false);
        
        //directly use getRawButton rather than getButton to prevent a bunch of unneccessary checks
        return new Trigger(() -> controllers.get(controller).hid.getRawButton(button));
    }
}
