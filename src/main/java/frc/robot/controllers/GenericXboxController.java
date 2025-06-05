package frc.robot.controllers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * A controller class for generic Xbox controllers. It is better to use more specific class if a suitable one is available
 */
public class GenericXboxController extends BasicController {
    
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
}
