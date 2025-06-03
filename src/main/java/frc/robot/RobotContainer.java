// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.controllers.ControllerManager;

public class RobotContainer {
    
    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);
        ControllerManager.createController(0); //driver 1
        ControllerManager.createController(1); //driver 2
    }
}
