package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final class IO {
        public static final int driverPort = 0;
        public static final int driveXAxis = 1;
        public static final int driveYAxis = 0;
        public static final int driveOmegaAxis = 2;
        public static final int resetGyroButton = 1;
        public static final int fieldOrientedButton = 7;
        public static final int robotOrientedButton = 5;
        public static final int boostButton = 8;
        public static final int resetOdometryButton = 3;

        public static final double deadband = 0.02;

        public static final double throttleLimiter = 0.6;
        public static final double turnThrottleLimiter = 0.6;
        public static final double boostedThrottleLimiter = 1.0;
        public static final double boostedTurnThrottleLimiter = 1.0;

        public static final String cameraName = "OV5647";
    }

    public static final class Swerve {

        public static final double maxSpeed = 5.15; //4.8768m
        public static final double maxTurnSpeed = 13.0857; // 4.8768 / (0.745363m * pi) * 2 * pi
        public static final double throttleLimitedMaxSpeed = maxSpeed * IO.throttleLimiter;
        public static final double throttleLimitedMaxTurnSpeed = maxTurnSpeed * IO.turnThrottleLimiter;
        public static final double throttleLimitedBoostedMaxSpeed = maxSpeed * IO.boostedThrottleLimiter;
        public static final double throttleLimitedBoostedMaxTurnSpeed = maxTurnSpeed * IO.boostedTurnThrottleLimiter;

        public static final double trackWidth = Units.inchesToMeters(20.75);
        public static final double wheelBase = Units.inchesToMeters(20.75);
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0)
        );

        public static final class Module {
            public static final double wheelDiameter = Units.inchesToMeters(4);

            public static final double driveGearRatio = 1.0 / 6.12;
            public static final double driveRotationsToMeters = driveGearRatio * Math.PI * wheelDiameter;
            public static final double driveRPMToMetersPerSecond = driveRotationsToMeters / 60.0;
            
            public static final double maxTurningSpeedRadiansPerMinute = 1.0;
            public static final double turnGearRatio = 1 / 21.42857143;
            public static final double turnRotationsToRadians = turnGearRatio * Math.PI * 2.0;
            public static final double turnRPMToRadiansPerSecond = maxTurningSpeedRadiansPerMinute / 60.0;


        }


    }
}
