package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.Util;

public class Swerve extends SubsystemBase {

    public boolean field_oritented = true;
    private final SwerveDriveKinematics kinematics;

    public final Pigeon2 pigeon2 = new Pigeon2(SwerveConstants.PIGEON_ID);
    private Pigeon2SimState pigeonSim = pigeon2.getSimState();

    StructArrayPublisher<SwerveModuleState> current_states = Util.table
            .getStructArrayTopic("Current Module States", SwerveModuleState.struct).publish();
    StructArrayPublisher<SwerveModuleState> target_states = Util.table
            .getStructArrayTopic("Target Module States", SwerveModuleState.struct).publish();
    StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault().getTable("Debug")
            .getStructTopic("Current pose", Pose2d.struct).publish();
    StructPublisher<Pose2d> estimatedPosePublisher = Util.table
            .getStructTopic("Estimated Pose", Pose2d.struct).publish();

    SwerveDrivePoseEstimator estimator;

    final SwerveDriveOdometry odometry;
    final Module[] modules = new Module[4];
    ChassisSpeeds speeds = new ChassisSpeeds();

    public boolean active = true;

    public Swerve() {
        kinematics = new SwerveDriveKinematics(
                createTranslation(SwerveConstants.TRACKWIDTH / 2.0, SwerveConstants.WHEELBASE / 2.0),
                createTranslation(SwerveConstants.TRACKWIDTH / 2.0, -SwerveConstants.WHEELBASE / 2.0),
                createTranslation(-SwerveConstants.TRACKWIDTH / 2.0, SwerveConstants.WHEELBASE / 2.0),
                createTranslation(-SwerveConstants.TRACKWIDTH / 2.0, -SwerveConstants.WHEELBASE / 2.0));

        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        for (int i = 0; i < modules.length; i++) {
            modules[i] = new Module(
                    tab.getLayout(SwerveConstants.LAYOUT_TITLE[i], BuiltInLayouts.kList)
                            .withSize(2, 4)
                            .withPosition(i * 2, 0),
                    SwerveConstants.CHASSIS_ID[i],
                    SwerveConstants.CHASSIS_ID[i],
                    SwerveConstants.ENCODER_ID[i]);
        }

        odometry = new SwerveDriveOdometry(kinematics, rotation(), modulePositions(),
                new Pose2d(0, 0, new Rotation2d()));

        estimator = new SwerveDrivePoseEstimator(kinematics, rotation(), modulePositions(), odometry.getPoseMeters());
    }

    private Translation2d createTranslation(double x, double y) {
        return new Translation2d(x, y);
    }

    public void resetAngle() {
        pigeon2.setYaw(0);
    }

    public Rotation2d rotation() {
        double rotation = pigeon2.getYaw().getValueAsDouble() % 360;
        rotation += (rotation < 0) ? 360 : 0;
        return new Rotation2d(Degree.of(rotation));
    }

    public void adjustRotation() {
        pigeon2.setYaw((rotation().getDegrees() + 180) % 360);
    }

    public void drive(ChassisSpeeds speeds) {
        this.speeds = speeds;
    }

    public void stop() {
        speeds = new ChassisSpeeds();
    }

    public double distance(Pose2d reference_point) {
        return pose().getTranslation().getDistance(reference_point.getTranslation());
    }

    public SwerveModulePosition[] modulePositions() {
        SwerveModulePosition[] pos = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++)
            pos[i] = new SwerveModulePosition(modules[i].drivePosition(), Rotation2d.fromRadians(modules[i].angle()));
        return pos;
    }

    public double getYaw() {
        return pigeon2.getYaw().getValueAsDouble();
    }

    public SwerveModuleState[] moduleStates(Module[] modules) {
        SwerveModuleState[] state = new SwerveModuleState[4];
        for (int i = 0; i < modules.length; i++)
            state[i] = new SwerveModuleState(modules[i].velocity(), new Rotation2d(modules[i].angle()));
        return state;
    }

    public Pose2d pose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry() {
        resetOdometry(new Pose2d(new Translation2d(), new Rotation2d(0)));
    }

    public void resetOdometry(Pose2d pose) {
        pigeon2.setYaw(pose.getRotation().getDegrees());
        resetModulePositions();

        odometry.resetPosition(rotation(), modulePositions(), pose);
    }

    public void resetModulePositions() {
        for (Module mod : modules)
            mod.resetDrivePosition();
    }

    public void syncEncoders() {
        for (Module mod : modules)
            mod.syncEncoders();
    }

    public void zeroAbsolute() {
        for (Module mod : modules)
            mod.zeroAbsolute();
    }

    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_VELOCITY);
        for (int i = 0; i < modules.length; i++) {
            states[i].optimize(new Rotation2d(modules[i].angle()));
            modules[i].set((states[i].speedMetersPerSecond / SwerveConstants.MAX_VELOCITY) * .8,
                    states[i].angle.getRadians());
        }
    }

    final MutDistance[] distance = { Meters.mutable(0), Meters.mutable(0), Meters.mutable(0), Meters.mutable(0) };
    final MutAngle[] angle = { Radians.mutable(0), Radians.mutable(0), Radians.mutable(0), Radians.mutable(0) };

    public double getRoll() {
        return pigeon2.getRoll().getValueAsDouble();
    }

    public void toggle() {
        active = !active;

        for (Module mod : modules)
            mod.stop();
    }

    public void simulationPeriodic() {
        for (int i = 0; i < modules.length; i++) {
            modules[i].periodicSimulation();
        }

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        if (active && speeds != new ChassisSpeeds())
            setModuleStates(states);

        current_states.set(moduleStates(modules));
        target_states.set(states);

        Pose2d pose = odometry.update(rotation(), modulePositions());
        posePublisher.set(pose);

        SmartDashboard.putNumber("X position", pose.getX());
        SmartDashboard.putNumber("Y position", pose.getY());

        SmartDashboard.putNumber("Odometry rotation", rotation().getDegrees());
        SmartDashboard.putNumber("Pigeon Yaw", pigeon2.getYaw().getValueAsDouble());
        SmartDashboard.putNumber("Pigeon Pitch",
                pigeon2.getPitch().getValueAsDouble());
        SmartDashboard.putNumber("Pigeon Roll",
                pigeon2.getRoll().getValueAsDouble());

        SmartDashboard.putString("Drive Mode", (field_oritented) ? "Field-Oriented" : "Robot-Oriented");
    }

    public static class SwerveConstants {
        public static final double TRACKWIDTH = 30;
        public static final double WHEELBASE = 30;

        public static final double PI2 = 2 * Math.PI;

        public static final double MAX_VELOCITY = 5.4;
        public static final double MAX_ANGULAR_VELOCITY = Math.PI / 6;
        public static final String[] LAYOUT_TITLE = { "Front Left", "Front Right", "Back Left", "Back Right" };
        public static final int[] CHASSIS_ID = { 2, 3, 4, 5 }; // FL, FR, BL, BR
        public static final int[] ENCODER_ID = { 7, 8, 9, 10 }; // FL, FR, BL, BR
        public static double[] ENCODER_OFFSETS = { -0.87890625, -0.996337890625, -0.638427734375, -0.892822265625 };
        public static final int PIGEON_ID = 6;

        public static double transFactor = 1.0;
        public static double rotFactor = .30;
    }
}