package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utility.IO;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Intake extends Command {

  Runnable intake;
  Runnable stop;
  BooleanSupplier holding;
  double angle;
  boolean release;

  public Intake(IO io, boolean release) {
    holding = () -> (release) ? io.shooter.coral() :  !io.shooter.coral();
    
    intake = () -> {
      io.shooter.intakeSpeed((release) ? 1 : .4);
    };

    stop = () -> {
      io.shooter.stopIntake();
    };

    this.release = release;
  }


  @Override
  public void initialize() {
    intake.run();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    stop.run();
  }

  @Override
  public boolean isFinished() {
    return holding.getAsBoolean();
  }
}