// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.SwerveJoystick;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.Constants;

public class RobotContainer {

  private final Swerve swerve;

  private final CommandJoystick driver;

  private boolean isFieldOriented = false;

  public RobotContainer() {
    swerve = new Swerve();

    driver = new CommandJoystick(Constants.IO.driverPort);

    configureBindings();

    // swerve.setDefaultCommand(new SwerveJoystick(
    //   () -> -driver.getRawAxis(Constants.IO.driveXAxis),
    //   () -> driver.getRawAxis(Constants.IO.driveYAxis),
    //   () -> driver.getRawAxis(Constants.IO.driveOmegaAxis),
    //   // () -> driver.button(Constants.IO.robotOrientedButton).getAsBoolean(),
    //   () -> isFieldOriented,
    //   swerve
    //   )
    // );

    //TESTING
    swerve.setDefaultCommand(new RunCommand(() -> {swerve.setDriveSpeeds(-driver.getRawAxis(1)); swerve.setTurnSpeeds(driver.getRawAxis(2));}, swerve));
  }

  private void configureBindings() {
    driver.button(1).onTrue(new InstantCommand(swerve::resetGyro));
    driver.button(2).onTrue(new InstantCommand(swerve::resetEncoders));
    driver.button(5).onTrue(new InstantCommand(() -> isFieldOriented = true));
    driver.button(7).onTrue(new InstantCommand(() -> isFieldOriented = false));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
