
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.AbsoluteEncoder;

public class SwerveModule {
  // motors
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  // encoders
  final RelativeEncoder m_driveEncoder;
  final RelativeEncoder m_turningEncoder;
  final AbsoluteEncoder m_absoluteEncoder;

  // final AbsoluteEncoder absoluteEncoder;

  // steering pid
  private SparkMaxPIDController m_pidController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  // drive pid
  // private CANPIDController m_drivepidController;
  // public double dkP, dkI, dkD, dkIz, dkFF, dkMaxOutput, dkMinOutput;

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel   ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModule(int driveMotorChannel, int turningMotorChannel, int absoluteEncoderChannel,
      boolean turningEncoderReversed, double angleOffset) {

    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

    m_driveMotor.restoreFactoryDefaults();
    m_turningMotor.restoreFactoryDefaults();

    m_driveEncoder = m_driveMotor.getEncoder();
    m_turningEncoder = m_turningMotor.getEncoder();
    m_absoluteEncoder = new AbsoluteEncoder(absoluteEncoderChannel, angleOffset);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution (ratio of distance traveled by wheel to distance traveled by
    // encoder shaft)
    m_driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderDistancePerPulse);
    m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderDistancePerPulse / 60.);

    // Set whether drive encoder should be reversed or not

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * wpi::math::pi)
    // divided by the encoder resolution.
    m_turningEncoder.setPositionConversionFactor(ModuleConstants.kSteerEncoderDistancePerPulse);
    m_turningEncoder.setVelocityConversionFactor(ModuleConstants.kSteerEncoderDistancePerPulse / 60.);
    // m_absoluteEncoder.setPositionConversionFactor(encoderCPR);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    // m_turningPIDController.enableContinuousInput(0, 2*Math.PI);

    // PID coefficients
    kP = 2; // 0.5
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 1;
    kMinOutput = -1;

    m_pidController = m_turningMotor.getPIDController();
    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    m_driveMotor.setSmartCurrentLimit(40);
    m_turningMotor.setSmartCurrentLimit(40);
    m_driveMotor.setIdleMode(IdleMode.kBrake);
    m_turningMotor.setIdleMode(IdleMode.kBrake);

    m_turningMotor.setInverted(true);

    m_driveMotor.burnFlash();
    m_turningMotor.burnFlash();
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getVelocity() + m_turningEncoder.getVelocity() * ModuleConstants.kWheelDiameterMeters / 2,
        new Rotation2d(m_turningEncoder.getPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param state Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState state) {

    double desiredDrive = state.speedMetersPerSecond / Constants.SwerveConstants.kMaxSpeedMetersPerSecond;

    if (Math.abs(desiredDrive) < 0.05) {
      m_driveMotor.set(0);
      return;
    }
    double desiredSteering = state.angle.getRadians();
    double currentSteering = m_turningEncoder.getPosition();

    // calculate shortest path to angle with forward drive (error -pi to pi)
    double steeringError = Math.IEEEremainder(desiredSteering - currentSteering, 2 * Math.PI);

    // reverse drive if error is larger than 90 degrees
    if (steeringError > Math.PI / 2) {
      steeringError -= Math.PI;
      desiredDrive *= -1;
    } else if (steeringError < -Math.PI / 2) {
      steeringError += Math.PI;
      desiredDrive *= -1;
    }

    double steeringSetpoint = currentSteering + steeringError;

    // m_driveMotor.set(desiredDrive + Math.cos(steeringError));
    m_driveMotor.set(desiredDrive);
    m_pidController.setReference(steeringSetpoint, ControlType.kPosition);
  }

  /**
   * Zeros all the SwerveModule encoders.
   */

  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
    m_turningEncoder.setPosition(m_absoluteEncoder.getAngle());
  }

  /**
   * Physically zeroes wheel. (i hope)
   */
  public void resetWheel() {
    m_driveMotor.set(0);
    m_pidController.setReference(0, ControlType.kPosition);

  }
  
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(), new Rotation2d(m_turningEncoder.getPosition()));
  }
}