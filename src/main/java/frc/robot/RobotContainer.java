package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
      () -> -driver.getRawAxis(Constants.IO.driveXAxis),
      () -> driver.getRawAxis(Constants.IO.driveYAxis),
      () -> driver.getRawAxis(Constants.IO.driveOmegaAxis),
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
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public boolean isRobotOriented() {
    return robotOriented;
  }
}
