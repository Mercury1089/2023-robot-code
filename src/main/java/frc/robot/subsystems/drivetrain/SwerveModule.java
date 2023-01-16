package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SWERVE;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

public class SwerveModule {
    private final CANSparkMax drivingSparkMax;
    private final CANSparkMax turningSparkMax;

    private final RelativeEncoder drivingEncoder;
    private final AbsoluteEncoder turningEncoder;

    private final SparkMaxPIDController drivingPIDController;
    private final SparkMaxPIDController turningPIDController;

    private double currchassisAngularOffset = 0;
    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

    public SwerveModule(int driveCAN, int turnCad, double chassisAngularOffset) {
        drivingSparkMax = new CANSparkMax(driveCAN, MotorType.kBrushless);
        turningSparkMax = new CANSparkMax(turnCad, MotorType.kBrushless);

        // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
        drivingSparkMax.restoreFactoryDefaults();
        turningSparkMax.restoreFactoryDefaults();

        // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
        drivingEncoder = drivingSparkMax.getEncoder();
        turningEncoder = turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
        drivingPIDController = drivingSparkMax.getPIDController();
        turningPIDController = turningSparkMax.getPIDController();
        drivingPIDController.setFeedbackDevice(drivingEncoder);
        turningPIDController.setFeedbackDevice(turningEncoder);

        /*
         * WPILib needs: 
         * METERS for distance
         * RADIANS for rotation
         */
        // rotations --> meters
        drivingEncoder.setPositionConversionFactor(SWERVE.METERS_CONVERSION); 
        // rpm --> m/s
        drivingEncoder.setVelocityConversionFactor(SWERVE.VELOCITY_CONVERSION);

        // deg --> radians
        turningEncoder.setPositionConversionFactor(SWERVE.RADIANS_CONVERSION);
        // dps --> rad/sec
        turningEncoder.setVelocityConversionFactor(SWERVE.RADIANS_VELOCITY_CONVERSION);

        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        turningEncoder.setInverted(true);

        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.
        turningPIDController.setPositionPIDWrappingEnabled(true);
        // m_turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
        // m_turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

        // Set the PID gains for the driving motor. Note these are example gains, and you
        // may need to tune them for your own robot!
        // m_drivingPIDController.setP(ModuleConstants.kDrivingP);
        // m_drivingPIDController.setI(ModuleConstants.kDrivingI);
        // m_drivingPIDController.setD(ModuleConstants.kDrivingD);
        // m_drivingPIDController.setFF(ModuleConstants.kDrivingFF);
        // m_drivingPIDController.setOutputRange(ModuleConstants.kDrivingMinOutput,
        //     ModuleConstants.kDrivingMaxOutput);

        // Set the PID gains for the turning motor. Note these are example gains, and you
        // may need to tune them for your own robot!
        // m_turningPIDController.setP(ModuleConstants.kTurningP);
        // m_turningPIDController.setI(ModuleConstants.kTurningI);
        // m_turningPIDController.setD(ModuleConstants.kTurningD);
        // m_turningPIDController.setFF(ModuleConstants.kTurningFF);
        // m_turningPIDController.setOutputRange(ModuleConstants.kTurningMinOutput,
        //     ModuleConstants.kTurningMaxOutput);

        // m_drivingSparkMax.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
        // m_turningSparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
        // m_drivingSparkMax.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
        // m_turningSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        drivingSparkMax.burnFlash();
        turningSparkMax.burnFlash();

        currchassisAngularOffset = chassisAngularOffset;
        desiredState.angle = new Rotation2d(turningEncoder.getPosition());
        drivingEncoder.setPosition(0);
    }

    public SwerveModulePosition getPosition() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModulePosition(
            drivingEncoder.getPosition(),
            new Rotation2d(turningEncoder.getPosition() - currchassisAngularOffset));
    }

    public void resetEncoders() {
        drivingEncoder.setPosition(0);
    }
}