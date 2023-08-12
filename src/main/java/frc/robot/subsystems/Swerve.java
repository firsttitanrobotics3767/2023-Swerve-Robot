package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class Swerve extends SubsystemBase{

    private final SwerveModule[] modules = {
        new SwerveModule(0),
        new SwerveModule(1),
        new SwerveModule(2),
        new SwerveModule(3)
    };

    private final AHRS gyro;

    public Swerve() {
        gyro = new AHRS(SerialPort.Port.kMXP);

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
    }

    public void resetGyro() {
        gyro.reset();
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
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeedMetersPerSecond);
        for (int i = 0; i < 4; i ++) {
            modules[i].setDesiredState(desiredStates[i]);
        }
    }

    //TESTING
    public void setDriveSpeeds(double speed) {
        for (int i = 0; i < 4; i ++) {
            modules[i].setDriveSpeed(speed);
        }
    }

    public void setTurnSpeeds(double speed) {
        for (int i = 0; i < 4; i ++) {
            modules[i].setTurnSpeed(speed);
        }
    }
}
