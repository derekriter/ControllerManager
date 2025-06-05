// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.controllers.BasicController;
import frc.robot.controllers.GenericXboxController;

public class RobotContainer {
    
    private final Robot robot;
    private final BasicController driver1;
    private final GenericXboxController driver2;
    
    public RobotContainer(Robot _robot) {
        robot = _robot;
        
        DriverStation.silenceJoystickConnectionWarning(true);
        driver1 = new BasicController(0, robot);
        driver2 = new GenericXboxController(1, robot);
    }
}
