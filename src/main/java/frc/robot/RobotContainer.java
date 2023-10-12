package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.SwerveJoystick;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.IO;

public class RobotContainer {

  private final Swerve swerve;

  private final CommandJoystick driver;

  private boolean robotOriented = false;

  public RobotContainer() {
  
    swerve = new Swerve();

    driver = new CommandJoystick(Constants.IO.driverPort);

    configureBindings();

    swerve.setDefaultCommand(new SwerveJoystick(
      () -> -driver.getRawAxis(IO.driveXAxis),
      () -> -driver.getRawAxis(IO.driveYAxis),
      () -> -driver.getRawAxis(IO.driveOmegaAxis),
      this::isRobotOriented,
      () -> driver.button(IO.boostButton).getAsBoolean(),
      swerve
      )
    );

    //TESTING
    // swerve.setDefaultCommand(new RunCommand(() -> {swerve.setDriveSpeeds(-driver.getRawAxis(1)); swerve.setTurnSpeeds(driver.getRawAxis(2));}, swerve));
  }

  private void configureBindings() {
    driver.button(IO.resetGyroButton).onTrue(new InstantCommand(swerve::resetGyro));
    driver.button(2).onTrue(new InstantCommand(swerve::resetEncoders));
    driver.button(IO.robotOrientedButton).onTrue(new InstantCommand(() -> robotOriented = true));
    driver.button(IO.fieldOrientedButton).onTrue(new InstantCommand(() -> robotOriented = false));
    driver.button(IO.resetOdometryButton).onTrue(new InstantCommand(() -> swerve.setOdometry(new Pose2d())));
  }

  public Command getAutonomousCommand() {
    // return Commands.print("No autonomous command configured");
    return new SwerveJoystick(() -> 0.0, () -> 0.07, () -> 0.0, () -> false, () -> false, swerve).withTimeout(1);
  }

  public boolean isRobotOriented() {
    return robotOriented;
  }
}
