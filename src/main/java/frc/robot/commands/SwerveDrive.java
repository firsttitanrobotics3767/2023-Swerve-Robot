package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class SwerveDrive extends CommandBase {
    
    private final Swerve swerve;
    private double xSpeed, ySpeed, omegaSpeed;

    public SwerveDrive(double xSpeed, double ySpeed, double omegaSpeed, Swerve swerve) {
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.omegaSpeed = omegaSpeed;
        this.swerve = swerve;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        
    }
}
