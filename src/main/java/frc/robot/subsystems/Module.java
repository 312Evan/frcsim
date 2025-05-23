package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.subsystems.Swerve.SwerveConstants;
import frc.robot.utility.Util;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.sim.SparkMaxSim;

public class Module {
    public final TalonFX drive;
    public final SparkMax steer;

    private final SimEncoder encoderSim;

    private final TalonFXSimState driveSim;
    private final SparkMaxSim steerSim;
    private final DCMotorSim driveMotorSim;
    private final DCMotorSim steerMotorSim;

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    private static final double STEER_REDUCTION = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    private static final double GEAR_RATIO = 8.14;
    private static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    private static final double DRIVE_KV = 0.025;
    private static final double DRIVE_KA = 0.005;
    private static final double STEER_KV = 0.045;
    private static final double STEER_KA = 0.003;

    private final DCMotor driveGearbox = DCMotor.getKrakenX60(1);
    private final DCMotor steerGearbox = DCMotor.getNEO(1);

    double desiredAngle;

    public Module(ShuffleboardLayout tab, int driveID, int steerID, int encoderID) {
        var drivePlant = LinearSystemId.createDCMotorSystem(DRIVE_KV, DRIVE_KA);
        var steerPlant = LinearSystemId.createDCMotorSystem(STEER_KV, STEER_KA);

        drive = new TalonFX(driveID, "rio");
        steer = new SparkMax(steerID, MotorType.kBrushless);

        encoderSim = new SimEncoder();

        driveSim = drive.getSimState();
        steerSim = new SparkMaxSim(steer, driveGearbox);

        driveMotorSim = new DCMotorSim(drivePlant, driveGearbox);
        steerMotorSim = new DCMotorSim(steerPlant, steerGearbox);

        SparkMaxConfig steerConfig = new SparkMaxConfig();
        steerConfig
                .smartCurrentLimit(20)
                .idleMode(IdleMode.kBrake)
                .inverted(true);

        steerConfig.encoder
                .positionConversionFactor(Math.PI * STEER_REDUCTION)
                .velocityConversionFactor(Math.PI * STEER_REDUCTION / 60);

        steerConfig.closedLoop
                .positionWrappingEnabled(true)
                .positionWrappingMaxInput(SwerveConstants.PI2)
                .pid(0.2, 0.0, 0.0);

        steerConfig.signals.primaryEncoderPositionAlwaysOn(false);
        steerConfig.signals.primaryEncoderPositionPeriodMs(20);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimit = 80;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 20;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        steer.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        steer.getEncoder().setPosition(angle());
        drive.getConfigurator().apply(config);

        tab.addDouble("Absolute Angle", () -> Math.toDegrees(angle()));
        tab.addDouble("Current Angle", () -> Math.toDegrees(steer.getEncoder().getPosition()));
        tab.addDouble("Angle Difference", () -> Math.toDegrees(angle() - steer.getEncoder().getPosition()));
        tab.addDouble("Target Angle", () -> Math.toDegrees(desiredAngle));
    }

    public void periodicSimulation() {
        driveSim.setSupplyVoltage(12.0);
        driveMotorSim.setInputVoltage(driveSim.getMotorVoltage());
        driveMotorSim.update(0.02);

        double driveMotorPosRot = driveMotorSim.getAngularPositionRotations();
        double driveMotorVelRps = driveMotorSim.getAngularVelocityRPM() / 60.0;

        double driveRotorPosition = driveMotorPosRot * GEAR_RATIO;
        double driveRotorVelocity = driveMotorVelRps * GEAR_RATIO;

        driveSim.setRawRotorPosition(driveRotorPosition);
        driveSim.setRotorVelocity(driveRotorVelocity);

        steerSim.setBusVoltage(12.0);
        double appliedVoltage = steerSim.getAppliedOutput() * steerSim.getBusVoltage();
        steerMotorSim.setInputVoltage(appliedVoltage);
        steerMotorSim.update(0.02);

        double steerMotorPosRad = steerMotorSim.getAngularPositionRad();
        double steerMotorVelRadPerSec = steerMotorSim.getAngularVelocityRadPerSec();

        double steerMechanismPosition = steerMotorPosRad * STEER_REDUCTION;
        double steerMechanismVelocity = steerMotorVelRadPerSec * STEER_REDUCTION;

        steerSim.getRelativeEncoderSim().setPosition(steerMechanismPosition);
        steerSim.getRelativeEncoderSim().setVelocity(steerMechanismVelocity);

        encoderSim.update(steerMechanismPosition, steerMechanismVelocity);
    }

    public void resetDrivePosition() {
        drive.setPosition(0.0);
        driveSim.setRawRotorPosition(0.0);
        driveMotorSim.setState(0.0, 0.0);
    }

    public void syncEncoders() {
        double angleRad = Util.wrapAngleRad(angle());
        steer.getEncoder().setPosition(angleRad);
        steerSim.getRelativeEncoderSim().setPosition(angleRad);
        encoderSim.setAbsPosition(angleRad);
    }

    public void zeroAbsolute() {
        zero();
    }

    public void zero() {
        encoderSim.setAbsPosition(0);
        steer.getEncoder().setPosition(0);
        steerSim.getRelativeEncoderSim().setPosition(0);
        steerMotorSim.setState(0.0, 0.0);
    }

    public double drivePosition() {
        return drive.getPosition().getValueAsDouble() * GEAR_RATIO * WHEEL_CIRCUMFERENCE;
    }

    public LinearVelocity velocity() {
        return MetersPerSecond
                .of(drive.getVelocity().getValueAsDouble() * (2 * Math.PI) * GEAR_RATIO * WHEEL_CIRCUMFERENCE);
    }

    public Voltage voltage() {
        return drive.getMotorVoltage().getValue();
    }

    public double angle() {
        return (encoderSim.getAbsPosition() % SwerveConstants.PI2 + SwerveConstants.PI2) % SwerveConstants.PI2;
    }

    public AngularVelocity steerVelocity() {
        return RadiansPerSecond.of(encoderSim.getVelocity());
    }

    public Voltage steerVoltage() {
        return Volts.of(steerSim.getBusVoltage());
    }

    public void stop() {
        drive.stopMotor();
        steer.stopMotor();
        steerSim.setAppliedOutput(0.0);
    }

    public void set(double driveVolts, double targetAngle) {
        syncEncoders();
        this.desiredAngle = targetAngle;
        drive.set(driveVolts / 12.0);
        steer.getClosedLoopController().setReference(targetAngle, ControlType.kPosition);
        encoderSim.setAbsPosition(targetAngle);
    }

    public void setSteer(double steerVolts) {
        syncEncoders();
        drive.set(0);
        steer.set(steerVolts / 12.0);
    }

    private static class SimEncoder {
        private double absPosition;
        private double velocity;

        public void setAbsPosition(double positionRadians) {
            this.absPosition = positionRadians;
        }

        public double getAbsPosition() {
            return absPosition;
        }

        public double getVelocity() {
            return velocity;
        }

        public void update(double pos, double vel) {
            this.absPosition = pos;
            this.velocity = vel;
        }
    }
}
