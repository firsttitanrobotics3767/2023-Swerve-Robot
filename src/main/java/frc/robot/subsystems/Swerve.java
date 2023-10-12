package frc.robot.subsystems;

import java.util.List;

// import org.photonvision.PhotonCamera;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class Swerve extends SubsystemBase{


    private final List<SwerveModule> modules;

    private final SwerveDriveOdometry odometry;
    private Pose2d pose, correctedPose;
    private final Field2d field;
    private AprilTagFieldLayout fieldLayout;

    private final AHRS gyro;
    // private final PhotonCamera camera;

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

        // camera = new PhotonCamera(Constants.IO.cameraName);

        pose = new Pose2d();
        odometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getRotation2d(), getModulePositions(), pose);
        // correctedPose = new Pose2d(pose.getX() * -1, pose.getY(), pose.getRotation());
        field = new Field2d();
        SmartDashboard.putData(field);
        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Heading", getHeading());
        for (SwerveModule module : modules) {
            SmartDashboard.putString("Module [" + module.getModuleID() + "] actual state", module.getStateString());
        }

        pose = odometry.update(getRotation2d(), getModulePositions());
        // the odometry had the x inverted, and this is a quick and dirty solution
        //TODO: find and fix the root cause
        // correctedPose = new Pose2d(pose.getX() * -1, pose.getY(), pose.getRotation());
        field.setRobotPose(pose);
    }

    public void resetGyro() {
        gyro.reset();
    }

    public void resetEncoders() {
        for (SwerveModule module : modules) {
            module.resetEncoders();
        }
    }

    public void resetOdometry() {
        odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    public void setOdometry(Pose2d newPose) {
        odometry.resetPosition(getRotation2d(), getModulePositions(), newPose);
    }

    public double getHeading() {
        return Math.IEEEremainder(-gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    // public Pose2d getPose() {
    //     return new 
    // }

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

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i <= 3; i++) {
            positions[i] = modules.get(i).getPosition();
        }
        return positions;
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
