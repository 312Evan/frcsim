package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

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
import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.reduxrobotics.sensors.canandmag.CanandmagSettings;
import com.revrobotics.sim.SparkMaxSim;

public class Module {
    public final TalonFX drive;
    public final SparkMax steer;
    public final Canandmag encoder;

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double STEER_REDUCTION = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);

    private final TalonFXSimState driveSim;
    private final SparkMaxSim steerSim;
    private final CanandmagSim encoderSim;
    private final DCMotorSim driveMotorSim;
    private final DCMotorSim steerMotorSim;

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
        encoder = new Canandmag(encoderID);

        driveSim = drive.getSimState();
        steerSim = new SparkMaxSim(steer, driveGearbox);
        encoderSim = new CanandmagSim();

        driveMotorSim = new DCMotorSim(drivePlant, driveGearbox);
        steerMotorSim = new DCMotorSim(steerPlant, steerGearbox);

        CanandmagSettings settings = new CanandmagSettings();
        settings.setInvertDirection(true);
        encoder.clearStickyFaults();
        encoder.setSettings(settings);

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
    
        double driveAngularVelRadPerSec = driveMotorSim.getAngularVelocityRadPerSec();
        double driveAngularPosRad = driveMotorSim.getAngularPositionRad();
        double driveRotorPosRotations = driveAngularPosRad / (2 * Math.PI) * GEAR_RATIO;
        double driveRotorVelRps = driveAngularVelRadPerSec / (2 * Math.PI) * GEAR_RATIO;
        driveSim.setRawRotorPosition(driveRotorPosRotations);
        driveSim.setRotorVelocity(driveRotorVelRps);
    
        steerSim.setBusVoltage(12.0);
        steerMotorSim.setInputVoltage(steerSim.getAppliedOutput() * steerSim.getBusVoltage());
        steerMotorSim.update(0.02);
    
        double steerAngularPosRad = steerMotorSim.getAngularPositionRad();
        double steerAngularVelRadPerSec = steerMotorSim.getAngularVelocityRadPerSec();
        double steerAngleRad = steerAngularPosRad * STEER_REDUCTION;
        double steerVelRadPerSec = steerAngularVelRadPerSec * STEER_REDUCTION;
        steerSim.getRelativeEncoderSim().setPosition(steerAngleRad);
        steerSim.getRelativeEncoderSim().setVelocity(steerVelRadPerSec * 60 / (2 * Math.PI)); // Convert rad/s to RPM
        encoderSim.setAbsPosition(steerAngleRad / (2 * Math.PI));
        encoderSim.setVelocity(steerVelRadPerSec / (2 * Math.PI));
    }

    public void resetDrivePosition() {
        drive.setPosition(0.0);
        driveSim.setRawRotorPosition(0.0);
        driveMotorSim.setState(0.0, 0.0);
    }

    public void syncEncoders() {
        double angleRad = Util.wrapAngleRad(angle()); // Ensure angle is in [-π, π]
        steer.getEncoder().setPosition(angleRad);
        steerSim.getRelativeEncoderSim().setPosition(angleRad);
        encoderSim.setAbsPosition(angleRad / (2 * Math.PI));
    }

    public void zeroAbsolute() {
        zero();
    }

    public void zero() {
        encoder.setAbsPosition(0, 250);
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
        return (encoderSim.getAbsPosition() * SwerveConstants.PI2) % SwerveConstants.PI2;
    }

    public AngularVelocity steerVelocity() {
        return RadiansPerSecond.of(encoderSim.getVelocity() * SwerveConstants.PI2);
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
    }

    public void setSteer(double steerVolts) {
        syncEncoders();
        drive.set(0);
        steer.set(steerVolts / 12.0);
    }

    private static class CanandmagSim {
        private double absPosition;
        private double velocity;

        public void setAbsPosition(double positionRotations) {
            this.absPosition = positionRotations;
        }

        public double getAbsPosition() {
            return absPosition;
        }

        public void setVelocity(double velocityRotationsPerSec) {
            this.velocity = velocityRotationsPerSec;
        }

        public double getVelocity() {
            return velocity;
        }
    }
}