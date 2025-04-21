package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  public TalonFX lead = new TalonFX(11);
  public TalonFX follow = new TalonFX(12);
  private final ElevatorSim motorSim;
  TrapezoidProfile profile = new TrapezoidProfile(new Constraints(100, 500));

  private final DCMotor gearbox = DCMotor.getKrakenX60(2);
  public final double gearing = 1.17;
  private final double mass = 15;
  private final double minHeight = 0.0;
  private final double maxHeight = 70.0;
  private final double radius = 2.0;

  PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);

  public boolean stopped = true;
  Timer time = new Timer();
  public double target = 0;
  public double position = 0;

  final String[] levelLayout = { "Rest", "L1", "L2", "L3", "L4", "Barge", "Low Algae", "High Algae" };
  public final double[] Level = { 0, 25, 43.5, 76, 110 };
  public final double Rest = 0;
  public final double L1 = 25;
  public final double L2 = 43.5;
  public final double L3 = 76;
  public final double L4 = 110;

  public Elevator() {
    var plant = LinearSystemId.createElevatorSystem(
        gearbox,
        mass,
        radius,
        gearing);

    motorSim = new ElevatorSim(plant, gearbox, minHeight, maxHeight, true, minHeight);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = 0.3;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.1;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    lead.getConfigurator().apply(config);
    follow.setControl(new Follower(lead.getDeviceID(), true));
  }

  public void speed(double speed) {
    lead.set(speed);
  }

  public void volts(double volts) {
    lead.setVoltage(volts);
  }

  public Voltage voltage() {
    return lead.getMotorVoltage().getValue();
  }

  public void stop() {
    stopped = true;
    lead.stopMotor();
    follow.stopMotor();
  }

  public void move(double height) {
    stopped = false;
    target = height;
    time.restart();
  }

  public void move(int level) {
    move(Level[Math.max(0, Math.min(4, level))]);
    SmartDashboard.putString("Target Elevator Level", levelLayout[level]);
  }

  public InstantCommand moveCommand(int level) {
    return new InstantCommand(() -> this.move(level), this);
  }

  public boolean atPosition() {
    return profile.isFinished(time.get());
  }

  public LinearVelocity velocity() {
    return MetersPerSecond.of(lead.getVelocity().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    motorSim.update(0.02);

    double motorRotations = (motorSim.getPositionMeters() / (2 * Math.PI * radius)) * gearing;
    double motorVelocityRPS = (motorSim.getVelocityMetersPerSecond() / (2 * Math.PI * radius)) * gearing;

    lead.getSimState().setRawRotorPosition(motorRotations);
    lead.getSimState().setRotorVelocity(motorVelocityRPS);

    double cTime = time.get();

    if (stopped)
      return;

    State out = profile.calculate(cTime,
        new State(position, motorSim.getVelocityMetersPerSecond()),
        new State(target, 0));

    motorSim.setInputVoltage(lead.getSimState().getMotorVoltage());


    double targetRotations = (out.position / (2 * Math.PI * radius)) * gearing;
    lead.setControl(positionRequest.withPosition(targetRotations).withEnableFOC(true));

    stopped = profile.isFinished(cTime);

    SmartDashboard.putNumber("Elevator Height", out.position);
    position = out.position;
    SmartDashboard.putNumber("Elevator Target Height", target);
    SmartDashboard.putNumber("Elevator Velocity", velocity().magnitude());
    SmartDashboard.putNumber("Elevator cTarget Velocity", out.velocity);
    SmartDashboard.putNumber("Elevator Voltage", voltage().magnitude());
  }
}
