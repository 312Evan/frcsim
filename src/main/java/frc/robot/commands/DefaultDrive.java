package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Swerve.SwerveConstants;
import frc.robot.utility.IO;

import java.util.function.DoubleSupplier;

public class DefaultDrive extends Command {

    private final IO io;
    private CommandXboxController controller;
    private final DoubleSupplier x_supplier;
    private final DoubleSupplier y_supplier;
    private final DoubleSupplier rotation_supplier;

    public DefaultDrive(IO io, ChassisSpeeds chassisSpeeds) {
        this(io, () -> chassisSpeeds.vxMetersPerSecond, () -> chassisSpeeds.vyMetersPerSecond, () -> chassisSpeeds.omegaRadiansPerSecond);
    }

    public DefaultDrive(IO io, CommandXboxController controller) {
        this(io, () -> -modifyAxis(controller.getLeftY()) * SwerveConstants.MAX_VELOCITY,
        () -> -modifyAxis(controller.getLeftX()) * SwerveConstants.MAX_VELOCITY,
        () -> -modifyAxis(controller.getRightX()) * SwerveConstants.MAX_ANGULAR_VELOCITY);
        this.controller = controller;
    }
  
    public DefaultDrive(IO io,
        DoubleSupplier translationXSupplier,
        DoubleSupplier translationYSupplier,
        DoubleSupplier rotationSupplier) {
        
        this.io = io;
        this.x_supplier = translationXSupplier;
        this.y_supplier = translationYSupplier;
        this.rotation_supplier = rotationSupplier;

        addRequirements(io.chassis);
    }
    
    @Override
    public void execute() {
        double down_scale = 1.25 - modifyAxis(controller.getLeftTriggerAxis());
        double up_scale = (SwerveConstants.MAX_VELOCITY * .2) * modifyAxis(controller.getRightTriggerAxis());

        double scale = SwerveConstants.transFactor * down_scale + up_scale;
        double rot_scale = SwerveConstants.rotFactor * down_scale;

        double xSpeed = x_supplier.getAsDouble() * scale;
        double ySpeed = y_supplier.getAsDouble() * scale;
        double rotationSpeed = rotation_supplier.getAsDouble() * down_scale * rot_scale;

        ChassisSpeeds output = new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed);
    
        if (io.chassis.field_oritented)
            output = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, io.chassis.rotation());

        io.chassis.drive(output);
    }

    @Override
    public void end(boolean interrupted) {
        io.chassis.stop();
    }

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) <= deadband) return 0.0;
        deadband *= (value > 0.0) ? 1 : -1;
        return (value + deadband) / (1.0 + deadband);
    }

    private static double modifyAxis(double value) {
        value = deadband(value, 0.1); // Deadband
        value = Math.copySign(value * value, value); // Square the axis
        return value;
    }
}