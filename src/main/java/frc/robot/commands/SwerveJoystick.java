package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.Constants;

public class SwerveJoystick extends CommandBase{
    
    private final Swerve swerve;
    private final Supplier<Double> xSpeedFunction, ySpeedFunction, omegaSpeedFunction;
    private final Supplier<Boolean> robotOrientedFunction;

    public SwerveJoystick(
        Supplier<Double> xSpeedFunction,
        Supplier<Double> ySpeedFunction,
        Supplier<Double> omegaSpeedFunction,
        Supplier<Boolean> fieldOrientedFunction,
        Swerve swerve
    ) {
        this.xSpeedFunction = xSpeedFunction;
        this.ySpeedFunction = ySpeedFunction;
        this.omegaSpeedFunction = omegaSpeedFunction;
        this.robotOrientedFunction = fieldOrientedFunction;
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        double xSpeed = xSpeedFunction.get();
        double ySpeed = ySpeedFunction.get();
        double omegaSpeed = omegaSpeedFunction.get();

        xSpeed = Math.abs(xSpeed) > Constants.IO.deadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > Constants.IO.deadband ? ySpeed : 0.0;
        omegaSpeed = Math.abs(omegaSpeed) > Constants.IO.deadband ? omegaSpeed : 0.0;

        xSpeed = xSpeed * Constants.Swerve.throttleLimitedMaxStrafeSpeed;
        ySpeed = ySpeed * Constants.Swerve.throttleLimitedMaxStrafeSpeed;
        omegaSpeed = omegaSpeed * Constants.Swerve.throttleLimitedMaxTurnSpeed;

        ChassisSpeeds chassisSpeeds;
        if (robotOrientedFunction.get()) {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, omegaSpeed);
        } else {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, swerve.getRotation2d());
        }

        SwerveModuleState[] states = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);

        swerve.setModuleStates(states);
    }

    @Override
    public void end(boolean isInterrupted) {
        swerve.stopModules();
    }
}
