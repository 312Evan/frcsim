package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.utility.IO;
import frc.robot.utility.Util;

public class ScoreReef extends SequentialCommandGroup {

  public ScoreReef(IO io, int level) {
    addCommands(
      io.elevator.moveCommand(level),
      new WaitUntilCommand(io.elevator::atPosition),
      Util.Do(() -> io.shooter.angle(level), io.shooter),
      new Intake(io, true),
      io.elevator.moveCommand(0)
    );
  }
}