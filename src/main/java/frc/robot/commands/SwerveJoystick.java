package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.Constants;

public class SwerveJoystick extends CommandBase{
    
    private final Swerve swerve;
    private final Supplier<Double> xSpeedFunction, ySpeedFunction, omegaSpeedFunction;
    private final Supplier<Boolean> robotOriented, boost;

    public SwerveJoystick(
        Supplier<Double> xSpeedFunction,
        Supplier<Double> ySpeedFunction,
        Supplier<Double> omegaSpeedFunction,
        Supplier<Boolean> robotOriented,
        Supplier<Boolean> boost,
        Swerve swerve
    ) {
        this.xSpeedFunction = xSpeedFunction;
        this.ySpeedFunction = ySpeedFunction;
        this.omegaSpeedFunction = omegaSpeedFunction;
        this.robotOriented = robotOriented;
        this.boost = boost;
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

        xSpeed = boost.get() ? xSpeed * Constants.Swerve.throttleLimitedBoostedMaxSpeed : xSpeed * Constants.Swerve.throttleLimitedMaxSpeed;
        ySpeed = boost.get() ? ySpeed * Constants.Swerve.throttleLimitedBoostedMaxSpeed : ySpeed * Constants.Swerve.throttleLimitedMaxSpeed;
        omegaSpeed = boost.get() ? omegaSpeed * Constants.Swerve.throttleLimitedBoostedMaxTurnSpeed : omegaSpeed * Constants.Swerve.throttleLimitedMaxTurnSpeed;

        ChassisSpeeds chassisSpeeds;
        if (robotOriented.get()) {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, omegaSpeed);
        } else {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, swerve.getRotation2d());
        }

        SwerveModuleState[] states = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
        

        swerve.setModuleStates(states);

        SmartDashboard.putString("Chassis Speeds", chassisSpeeds.toString());
    }

    @Override
    public void end(boolean isInterrupted) {
        swerve.stopModules();
    }
}
