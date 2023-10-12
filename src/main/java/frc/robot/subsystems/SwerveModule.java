package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Constants;

public class SwerveModule {

    private final int moduleID;
    
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    private final boolean[] isDriveInverted = {false, false, false, false};
    // private final boolean[] isTurnInverted  = {false, false, false, false};
    private final boolean[] isTurnInverted  = {true, true, true, true};

    private final CANCoder absoluteEncoder;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;
    
    private final SparkMaxPIDController driveController;
    private final SparkMaxPIDController turnController;

    private final int[] driveMotorIDs      = {1, 2, 3, 4}; 
    private final int[] turnMotorIDs       = {11, 21, 31, 41};
    private final int[] absoluteEncoderIDs = {12, 22, 32, 42};

    // 0 : frontLeft, 1 : frontRight, 2 : backLeft, 3 : backRight
    public SwerveModule(int moduleID) {

        this.moduleID = moduleID;

        driveMotor = new CANSparkMax(driveMotorIDs[moduleID], MotorType.kBrushless);
        driveMotor.restoreFactoryDefaults();
        driveMotor.clearFaults();
        driveMotor.setInverted(isDriveInverted[moduleID]);
        driveMotor.setIdleMode(IdleMode.kCoast);
        driveMotor.burnFlash();
        System.out.println("Module " + moduleID + " drive motor configured");

        turnMotor = new CANSparkMax(turnMotorIDs[moduleID], MotorType.kBrushless);
        turnMotor.restoreFactoryDefaults();
        turnMotor.clearFaults();
        turnMotor.setInverted(isTurnInverted[moduleID]);
        turnMotor.setIdleMode(IdleMode.kCoast);
        turnMotor.burnFlash();
        System.out.println("Module " + moduleID + " turn motor configured");

        absoluteEncoder = new CANCoder(absoluteEncoderIDs[moduleID]);
        absoluteEncoder.configSensorDirection(false);

        try {Thread.sleep(200);} catch (Exception e) {}
        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(Constants.Swerve.Module.driveRotationsToMeters);
        driveEncoder.setVelocityConversionFactor(Constants.Swerve.Module.driveRPMToMetersPerSecond);
        System.out.println("Module " + moduleID + " drive encoder configured");

        turnEncoder = turnMotor.getEncoder();
        turnEncoder.setPositionConversionFactor(Constants.Swerve.Module.turnRotationsToRadians);
        turnEncoder.setVelocityConversionFactor(Constants.Swerve.Module.turnRPMToRadiansPerSecond);
        System.out.println("Module " + moduleID + " turn encoder configured");

        driveController = driveMotor.getPIDController();
        driveController.setP(0.06);
        driveController.setI(0);
        driveController.setD(0);
        driveController.setFF(0.195);
        driveController.setIZone(0);

        turnController = turnMotor.getPIDController();
        // turnController.setFeedbackDevice((AbsoluteEncoder) absoluteEncoder);
        turnController.setP(0.5);
        turnController.setI(0);
        turnController.setD(0);
        // turnController.setFF(0.005);
        turnController.setIZone(0);
        turnController.setPositionPIDWrappingEnabled(true);
        turnController.setPositionPIDWrappingMaxInput(Math.PI);
        turnController.setPositionPIDWrappingMinInput(-Math.PI);

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

    public double getAbsoluteEncoderPosition() {
        return absoluteEncoder.getAbsolutePosition();
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turnEncoder.setPosition(getAbsoluteEncoderPosition());
    }

    public SwerveModuleState getState() {
        // return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromRadians(getTurnPosition()));
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAbsoluteEncoderPosition()));
    }

    public String getStateString() {
        return String.format("Speed: %.2f m/s, Angle: %.2f", getDriveVelocity(), getState().angle.getDegrees());
    }

    public void setDesiredState(SwerveModuleState state) {
        if (state.speedMetersPerSecond < 0.05) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, new Rotation2d(getAbsoluteEncoderPosition()));
        driveController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
        turnController.setReference(state.angle.getRadians(), ControlType.kPosition);
        SmartDashboard.putString("Module [" + getModuleID() + "] desired state",
            String.format("Speed: %.2f m/s, Angle: %.2f", state.speedMetersPerSecond, state.angle.getDegrees()));
        
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), getState().angle);
    }

    public void stop() {
        driveController.setReference(0, ControlType.kVelocity);
        turnController.setReference(getTurnPosition(), ControlType.kPosition);
    }

    public double getModuleID() {
        return moduleID;
    }

    //TESTING
    public void setDriveSpeed(double speed) {
        // driveMotor.set(speed);
        speed = speed * Constants.Swerve.maxSpeed;
        if (speed < 0.05 && speed > -0.05) {
            return;
        }
        driveController.setReference(speed, ControlType.kVelocity);
        SmartDashboard.putNumber("Drive speed setpoint", speed);

    }

    public void setTurnPosition(double position) {
        double positionDegrees = position * 180;
        position = position * Math.PI;
        turnController.setReference(position, ControlType.kPosition);
        SmartDashboard.putNumber("Turn Position Setpoint", positionDegrees);
    }
    
}
