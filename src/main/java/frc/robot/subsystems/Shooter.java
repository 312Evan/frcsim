package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  public SparkMax intake = new SparkMax(13, MotorType.kBrushless);
  public SparkMax pivot = new SparkMax(14, MotorType.kBrushless);

  private final SingleJointedArmSim pivotSim;
  private static final DCMotor pivotGearbox = DCMotor.getNEO(1);
  private final double pivotGearing = 17.0;
  private final double armLength = Units.inchesToMeters(15);
  private final double armMass = Units.lbsToKilograms(12);
  private final double armMOI = SingleJointedArmSim.estimateMOI(armLength, armMass);
  private final double minAngle = Units.degreesToRadians(0);
  private final double maxAngle = Units.degreesToRadians(90);

  private final DCMotor intakeGearbox = DCMotor.getNEO(1);
  private final double intakeGearing = 7.0;
  private final DCMotorSim intakeSim;

  private boolean beamBroken = false;
  private double hoodAngle = 0.0;

  private final TrapezoidProfile profile = new TrapezoidProfile(new Constraints(100, 500));
  private final Timer time = new Timer();
  private double target = 0.0;
  private boolean stopped = true;

  public Shooter() {
    var pivotPlant = LinearSystemId.createSingleJointedArmSystem(pivotGearbox, armMOI, pivotGearing);
    pivotSim = new SingleJointedArmSim(pivotPlant, pivotGearbox, pivotGearing, armLength, minAngle, maxAngle, true,
        minAngle);

    var intakePlant = LinearSystemId.createDCMotorSystem(intakeGearbox, armMOI, intakeGearing);
    intakeSim = new DCMotorSim(intakePlant, intakeGearbox);


    SparkMaxConfig config = new SparkMaxConfig();

    config.idleMode(SparkMaxConfig.IdleMode.kBrake);
    intake.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    pivotConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
    pivotConfig.closedLoop.pidf(0.0, 0.0, 0.0, 0.0, ClosedLoopSlot.kSlot0);

    pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    time.start();
  }

  public void pivotVolts(double volts) {
    pivot.setVoltage(volts);
  }

  public void pivotSpeed(double speed) {
    pivot.set(speed);
  }

  public void stopPivot() {
    pivot.stopMotor();
  }

  public void intakeVolts(double volts) {
    intake.setVoltage(volts);
  }

  public void intakeSpeed(double speed) {
    intake.set(speed);
  }

  public void stopIntake() {
    intake.stopMotor();
  }

  public boolean coral() {
    return beamBroken;
  }

  public void setBeamBroken(boolean broken) {
    beamBroken = broken;
  }

  public void hoodAngle(double angle) {
    hoodAngle = angle;
  }

  public void angle(double targetAngle) {
    target = targetAngle;
    stopped = false;
    time.restart();
  }

  public double angle() {
    return pivotSim.getAngleRads();
  }

  @Override
  public void simulationPeriodic() {
    intakeSim.setInputVoltage(intake.getBusVoltage());

    pivotSim.update(0.02);
    intakeSim.update(0.02);

    SmartDashboard.putBoolean("Coral", coral());

    SmartDashboard.putNumber("Pivot Angle", angle());
    SmartDashboard.putNumber("Hood Angle", hoodAngle);
    SmartDashboard.putNumber("Intake Volts", intake.getAppliedOutput());

    double cTime = time.get();
    if (stopped)
      return;

    State out = profile.calculate(cTime, new State(angle(), 0), new State(target, 0));
    pivotSim.setInputVoltage(12 * out.position);
    stopped = profile.isFinished(cTime);
  }
}
