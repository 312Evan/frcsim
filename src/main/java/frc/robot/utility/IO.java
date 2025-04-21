package frc.robot.utility;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.Intake;
import frc.robot.commands.ScoreReef;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class IO extends SubsystemBase {
  public final Elevator elevator = new Elevator();
  public final Shooter shooter = new Shooter();
  public final Swerve chassis = new Swerve();

  public final CommandXboxController controller = new CommandXboxController(0);

  public IO() {
    DriverStation.silenceJoystickConnectionWarning(true);
    configure();
  }

  public void configure() {
    chassis.setDefaultCommand(new DefaultDrive(this, controller));

    controller.y().and(() -> shooter.coral()).onTrue(Util.Do(() -> new ScoreReef(this, 4)));
    controller.x().and(() -> shooter.coral()).onTrue(Util.Do(() -> new ScoreReef(this, 3)));
    controller.b().and(() -> shooter.coral()).onTrue(Util.Do(() -> new ScoreReef(this, 2)));
    controller.a().and(() -> shooter.coral()).onTrue(Util.Do(() -> new ScoreReef(this, 1)));
    controller.a().and(() -> shooter.coral()).onTrue(new Intake(this, false));


    controller.start().onTrue(Util.Do(() -> elevator.move(0)));
  }

  @Override
  public void periodic() {

  }
}
