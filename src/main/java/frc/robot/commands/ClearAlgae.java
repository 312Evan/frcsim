// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.utility.IO;
import frc.robot.utility.Util;

public class ClearAlgae extends SequentialCommandGroup {

  public ClearAlgae(IO io) {
    addCommands(
      io.elevator.moveCommand(0),
      new WaitUntilCommand(.2),
      Util.Do(() -> {
        io.shooter.angle(0);
        io.shooter.intakeSpeed(-1);
      }, io.shooter),
      io.elevator.moveCommand(4),
      new WaitUntilCommand(.5),
      Util.Do(() -> {
        io.shooter.intakeSpeed(0);
      }, io.shooter),
      io.elevator.moveCommand(0)
    );
  }
}