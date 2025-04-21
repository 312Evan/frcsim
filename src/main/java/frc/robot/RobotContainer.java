// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.utility.IO;

public class RobotContainer {
  public final IO io = new IO();

  public RobotContainer() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
