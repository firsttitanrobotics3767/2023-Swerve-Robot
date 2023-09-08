package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Constants;

public class SwerveModule {

    private final int moduleID;
    
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    private final boolean[] isDriveInverted = {false, false, false, false};
    private final boolean[] isTurnInverted  = {false, false, false, false};

    private final CANCoder absoluteEncoder;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;
    
    // private final SparkMaxPIDController driveController;
    // private final SparkMaxPIDController turnController;

    private final int[] driveMotorIDs      = {1, 2, 3, 4}; 
    private final int[] turnMotorIDs       = {11, 21, 31, 41};
    private final int[] absoluteEncoderIDs = {12, 22, 32, 42};

    // 0 : frontLeft, 1 : frontRight, 2 : backLeft, 3 : backRight
    public SwerveModule(int moduleID) {

        this.moduleID = moduleID;

        System.out.println("Module " + moduleID);
        driveMotor = new CANSparkMax(driveMotorIDs[moduleID], MotorType.kBrushless);
        System.out.println(driveMotor.restoreFactoryDefaults().name());
        System.out.println(driveMotor.clearFaults().name());
        driveMotor.setInverted(isDriveInverted[moduleID]);
        System.out.println(driveMotor.setIdleMode(IdleMode.kCoast).name());
        System.out.println(driveMotor.burnFlash().name());
        System.out.println("Module " + moduleID + " drive motor configured");

        turnMotor = new CANSparkMax(turnMotorIDs[moduleID], MotorType.kBrushless);
        System.out.println(turnMotor.restoreFactoryDefaults().name());
        System.out.println(turnMotor.clearFaults().name());
        turnMotor.setInverted(isTurnInverted[moduleID]);
        System.out.println(turnMotor.setIdleMode(IdleMode.kCoast).name());
        System.out.println(turnMotor.burnFlash().name());
        System.out.println("Module " + moduleID + " turn motor configured");

        absoluteEncoder = new CANCoder(absoluteEncoderIDs[moduleID]);
        // absoluteEncoder.configMagnetOffset(absoluteEncoder.configGetMagnetOffset() - (90 * moduleID));
        // absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        System.out.println("Module " + moduleID + " absolute encoder configured");

        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(Constants.Swerve.Module.driveRotationsToMeters);
        driveEncoder.setVelocityConversionFactor(Constants.Swerve.Module.driveRPMToMetersPerSecond);
        System.out.println("Module " + moduleID + " drive encoder configured");

        turnEncoder = turnMotor.getEncoder();
        turnEncoder.setPositionConversionFactor(Constants.Swerve.Module.turnRotationsToRadians);
        turnEncoder.setVelocityConversionFactor(Constants.Swerve.Module.turnRPMToRadiansPerSecond);      
        System.out.println("Module " + moduleID + " turn encoder configured");

        // driveController = driveMotor.getPIDController();
        // driveController.setFeedbackDevice(driveEncoder);
        // driveController.setP(0.0020645);
        // driveController.setI(0);
        // driveController.setD(0);
        // driveController.setFF(0);
        // driveController.setIZone(0);

        // turnController = turnMotor.getPIDController();
        // turnController.setFeedbackDevice(turnEncoder);
        // turnController.setP(0.01);
        // turnController.setI(0);
        // turnController.setD(0);
        // turnController.setFF(0);
        // turnController.setIZone(0);
        // turnController.setPositionPIDWrappingEnabled(true);
        // turnController.setPositionPIDWrappingMaxInput(moduleID);
        // turnController.setPositionPIDWrappingMinInput(moduleID);

        resetEncoders();

    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurnPosition() {
        return turnEncoder.getPosition();
    }

    public double getTurnVelocity() {
        return turnEncoder.getVelocity();
    }

    public double getAbsoluteEncoderPositionRadians() {
        return (absoluteEncoder.getAbsolutePosition() / 180) * Math.PI;
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turnEncoder.setPosition(getAbsoluteEncoderPositionRadians() - (moduleID - 1) * 0.5 * Math.PI);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPosition()));
        // return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAbsoluteEncoderPositionRadians()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (state.speedMetersPerSecond < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        // driveController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
        // turnController.setReference(state.angle.getRadians(), ControlType.kPosition);
        SmartDashboard.putString("Module [" + moduleID + "] desired state", state.toString());
    }

    public void stop() {
        // driveController.setReference(0, ControlType.kVelocity);
        // turnController.setReference(getTurnPosition(), ControlType.kPosition);
    }

    public double getModuleID() {
        return moduleID;
    }

    //TESTING
    public void setDriveSpeed(double speed) {
        driveMotor.set(speed);
    }

    public void setTurnSpeed(double speed) {
        turnMotor.set(speed);
    }
    
}
