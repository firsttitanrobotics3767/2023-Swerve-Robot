package frc.robot;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.SupplyElevator;
import frc.robot.commands.SwerveJoystick;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.IO;

public class RobotContainer {

  private final Swerve swerve;
  private final Elevator elevator;

  private final CommandJoystick driver;

  private boolean robotOriented = false;

  private PathConstraints constraints;
  private PathPlannerTrajectory path;
  // private List<PathPlannerTrajectory

  public RobotContainer() {
  
    swerve = new Swerve();
    elevator = new Elevator();

    driver = new CommandJoystick(Constants.IO.driverPort);

    constraints = new PathConstraints(2, 10);
    path = PathPlanner.loadPath("testPath", constraints);

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

    // driver.button(2).whileTrue(new SupplyElevator(() -> 0.2, elevator));
    // driver.button(5).whileTrue(new SupplyElevator(() -> 0.2, elevator));
    driver.povDown().whileTrue(new SupplyElevator(() -> -0.4, elevator));
    driver.povUp().whileTrue(new SupplyElevator(() -> 0.4, elevator));
    driver.povCenter().whileTrue(new SupplyElevator(() -> 0.0, elevator));
  }

  public Command getAutonomousCommand() {
    return swerve.getTrajectoryCommand(path, robotOriented);
  }

  public boolean isRobotOriented() {
    return robotOriented;
  }
}
