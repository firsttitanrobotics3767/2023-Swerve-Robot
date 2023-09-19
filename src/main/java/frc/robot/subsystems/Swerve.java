package frc.robot.subsystems;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class Swerve extends SubsystemBase{


    private final List<SwerveModule> modules;

    private final AHRS gyro;

    public Swerve() {

        modules = List.of(
            new SwerveModule(0),
            new SwerveModule(1),
            new SwerveModule(2),
            new SwerveModule(3)
        );

        gyro = new AHRS(SerialPort.Port.kMXP);
        // Give gyro 1 second to start up before reset
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                resetGyro();
            } catch (Exception e) {}
        }).start();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Heading", getHeading());
        for (SwerveModule module : modules) {
            SmartDashboard.putString("Module [" + module.getModuleID() + "] actual state", module.getStateString());
        }
    }

    public void resetGyro() {
        gyro.reset();
    }

    public void resetEncoders() {
        for (SwerveModule module : modules) {
            module.resetEncoders();
        }
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public void stopModules() {
        for (SwerveModule module : modules) {
            module.stop();
        }
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        for (SwerveModule module : modules) {
            module.setDesiredState(desiredStates[modules.indexOf(module)]);
        }
    }

    //TESTING
    public void setDriveSpeeds(double speed) {
        for  (SwerveModule module : modules) {
            module.setDriveSpeed(speed);
        }
    }

    public void setTurnSpeeds(double speed) {
        for  (SwerveModule module : modules) {
            module.setTurnPosition(speed);
        }
    }
}
