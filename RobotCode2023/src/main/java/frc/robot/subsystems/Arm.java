// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Electrical;

public class Arm extends SubsystemBase {
  private final CANSparkMax shoulderJoint;
  private final CANSparkMax elbowJoint;
  private final CANSparkMax wristJoint;

  private final DutyCycleEncoder shoulderEncoder;
  private final DutyCycleEncoder elbowEncoder;
  private final DutyCycleEncoder wristEncoder;

  public final ProfiledPIDController shoulderPID;
  public final ProfiledPIDController elbowPID;
  public final ProfiledPIDController wristPID;

  private double shoulderSetpoint;
  private double elbowSetpoint;
  private double wristSetpoint;

  private double shoulderPower;
  private double elbowPower;
  private double wristPower;

  private boolean shoulderPIDEnabled;
  private boolean elbowPIDEnabled;
  private boolean wristPIDEnabled;

  /** Creates a new Arm. */
  public Arm() {
    super();
    shoulderJoint = new CANSparkMax(Electrical.SHOULDER, MotorType.kBrushless);
    elbowJoint = new CANSparkMax(Electrical.ELBOW, MotorType.kBrushless);
    wristJoint = new CANSparkMax(Electrical.WRIST, MotorType.kBrushless);
    // shoulderJoint.restoreFactoryDefaults();
    // elbowJoint.restoreFactoryDefaults();
    // wristJoint.restoreFactoryDefaults();

    shoulderJoint.setInverted(true);
    elbowJoint.setInverted(true);
    wristJoint.setInverted(true);

    // encoder to determine offset angle
    shoulderEncoder = new DutyCycleEncoder(Constants.ArmConstants.SHOULDER_ENC);
    elbowEncoder = new DutyCycleEncoder(Constants.ArmConstants.ELBOW_ENC);
    wristEncoder = new DutyCycleEncoder(Constants.ArmConstants.WRIST_ENC);

    // Old method without abs encoders
    // shoulderInit = Math.PI / 2;
    // elbowInit = Math.toRadians(-71.657202);
    // wristInit = 0;

    shoulderJoint.setSmartCurrentLimit(40);
    elbowJoint.setSmartCurrentLimit(40);
    wristJoint.setSmartCurrentLimit(20);

    shoulderPID = new ProfiledPIDController(1, 0, 0, new Constraints(0.5, 0.5)); // 2, 2.25
    elbowPID = new ProfiledPIDController(0.4, 0, 0.03, new Constraints(0.5, 0.5)); // 2.5, 2.5
    wristPID = new ProfiledPIDController(0.5, 0, 0, new Constraints(1, 0.5)); // 2, 2

    shoulderJoint.setIdleMode(IdleMode.kBrake);
    elbowJoint.setIdleMode(IdleMode.kBrake);
    wristJoint.setIdleMode(IdleMode.kBrake);

    shoulderJoint.burnFlash();
    elbowJoint.burnFlash();
    wristJoint.burnFlash();

  }

  public void moveArm(double motorPowerShoulder, double motorPowerElbow, double motorPowerWrist) {
    if (((getArmPosition().getX() > Units.inchesToMeters(48 + 18) && motorPowerShoulder < 0))
        || (getArmPosition().getX() > 18 - 9) && (getShoulder() > Math.toRadians(97) && motorPowerShoulder > 0)) {
      motorPowerShoulder = 0;
    }
    if ((getArmPosition().getX() > Units.inchesToMeters(48 + 18))
        && ((motorPowerElbow > 0 && getElbow() < 0) || (motorPowerElbow < 0 && getElbow() > 0))) {
      motorPowerElbow = 0;
    }
    if (getArmPosition().getY() > Units.inchesToMeters(78 - 12.25) && motorPowerElbow > 0) {
      motorPowerElbow = 0;
    }
    if (getArmPosition().getY() > Units.inchesToMeters(78 - 12.25)
        && ((motorPowerShoulder > 0 && getShoulder() < Math.PI / 2)
            || (motorPowerShoulder < 0 && getShoulder() > Math.PI / 2))) {
      motorPowerShoulder = 0;
    }

    if (Math.abs(motorPowerShoulder) < 0.05) {
      shoulderPIDEnabled = true;
    } else {
      shoulderPower = motorPowerShoulder;
      shoulderJoint.set(shoulderPower);
      shoulderSetpoint = getShoulder();
      shoulderPID.reset(shoulderSetpoint);
      shoulderPIDEnabled = false;
    }

    if (Math.abs(motorPowerElbow) < 0.05) {
      elbowPIDEnabled = true;
    } else {
      elbowPower = motorPowerElbow + 0.05 * Math.cos(getShoulder() + getElbow() + Math.PI / 2);
      elbowJoint.set(elbowPower);
      elbowSetpoint = getElbow();
      elbowPID.reset(elbowSetpoint);
      elbowPIDEnabled = false;
    }

    if (Math.abs(motorPowerWrist) < 0.05) {
      wristPIDEnabled = true;
    } else {
      wristPower = motorPowerWrist;
      wristJoint.set(wristPower);
      wristSetpoint = getWrist();
      wristPID.reset(wristSetpoint);
      wristPIDEnabled = false;
    }

  }

  public void setShoulder(double angle, double ffSpeed) {
    shoulderPIDEnabled = true;
    shoulderSetpoint = angle;
  }

  public double getShoulder() {
    return MathUtil
        .angleModulus(2 * Math.PI * shoulderEncoder.getAbsolutePosition() - Constants.ArmConstants.SHOULDER_OFFSET);
  }

  /*
   * Sets angle between humerus and radius according to shoulder joint angle
   * Angle 0 = radius is on top of humerus
   */
  public void setElbow(double angle, double ffSpeed) {
    elbowPIDEnabled = true;
    elbowSetpoint = angle;
  }

  public double getElbow() {
    return MathUtil
        .angleModulus(2 * Math.PI * elbowEncoder.getAbsolutePosition() - Constants.ArmConstants.ELBOW_OFFSET);
  }

  /*
   * Sets angle between metacarpals and radius
   * Angle 0 = metacarpals is on top of radius
   */
  public void setWrist(double angle, double ffSpeed) {
    wristPIDEnabled = true;
    wristSetpoint = angle;
  }

  public double getWrist() {
    return MathUtil
        .angleModulus(2 * Math.PI * wristEncoder.getAbsolutePosition() - Constants.ArmConstants.WRIST_OFFSET);
  }

  /*
   * PARAMETERS
   * width: horizontal distance between shoulder pivot point and where wrist pivot
   * point should be
   * height: vertical distance between shoulder pivot point and where wrist pivot
   * point should be
   * 
   * c = (a^2 + b^2 - 2abcosC)^(1/2)
   * a^2 + b^2 - c^2 = 2abcosC
   * sinA/a = sinB/b = sinC/c
   */

  public static class JointAngles {

    public double shoulderAngle;
    public double elbowAngle;
    public double wristAngle;

    public JointAngles(double shoulderAngle, double elbowAngle, double wristAngle) {

      this.shoulderAngle = shoulderAngle;
      this.elbowAngle = elbowAngle;
      this.wristAngle = wristAngle;

    }

    public static JointAngles anglesFrom2D(double width, double height, double wristAngle) {
      // math checked with CAD
      // Proof: https://imgur.com/3QSfHY5
      double a = Constants.ArmConstants.SHOULDER_LENGTH;
      double b = Constants.ArmConstants.ELBOW_LENGTH;
      double c = Math.hypot(width, height);

      double angle1 = Math.acos((Math.pow(a, 2) + Math.pow(b, 2) - Math.pow(c, 2)) / (2 * a * b));

      // figure out how to calculate shoulder angle
      double angle2 = Math.asin((Math.sin(angle1) / c) * b);
      if (a + b < c) {
        angle1 = Math.PI;
        angle2 = 0;
      }
      // from horizontal
      double shoulderAngle = Math.atan2(height, width) + angle2;

      double elbowAngle = angle1 + shoulderAngle - Math.PI;

      JointAngles angles = new JointAngles(shoulderAngle, elbowAngle, wristAngle);
      return angles;

    }

  }

  public Vector<N2> angleVelocity(Vector<N2> velocity) {
    double sAngle = shoulderEncoder.getAbsolutePosition();
    double eAngle = elbowEncoder.getAbsolutePosition();
    Matrix<N2, N2> MAngularToLinear = new MatBuilder<>(N2.instance, N2.instance).fill(
        -Constants.ArmConstants.SHOULDER_LENGTH * Math.sin(sAngle),
        -Constants.ArmConstants.ELBOW_LENGTH * Math.sin(eAngle),
        Constants.ArmConstants.SHOULDER_LENGTH * Math.cos(sAngle),
        Constants.ArmConstants.ELBOW_LENGTH * Math.cos(eAngle)

    );

    return new Vector<>(MAngularToLinear.inv().times(velocity));
  }

  public Translation2d getArmPosition() {
    double sAngle = shoulderEncoder.getAbsolutePosition();
    double eAngle = elbowEncoder.getAbsolutePosition();

    double intakeX = Constants.ArmConstants.ELBOW_LENGTH * Math.cos(eAngle)
        + Constants.ArmConstants.SHOULDER_LENGTH * Math.cos(sAngle);
    double intakeY = Constants.ArmConstants.ELBOW_LENGTH * Math.sin(eAngle)
        + Constants.ArmConstants.SHOULDER_LENGTH * Math.sin(sAngle);

    return new Translation2d(intakeX, intakeY);
  }

  public void resetPIDs() {
    shoulderSetpoint = getShoulder();
    elbowSetpoint = getElbow();
    wristSetpoint = getWrist();
    shoulderPID.reset(shoulderSetpoint);
    elbowPID.reset(elbowSetpoint);
    wristPID.reset(wristSetpoint);
    shoulderPIDEnabled = true;
    elbowPIDEnabled = true;
    wristPIDEnabled = true;
  }

  public void updateSmartDashBoard() {
    SmartDashboard.putNumber("shoulderDeg", Math.toDegrees(getShoulder()));
    SmartDashboard.putNumber("elbowDeg", Math.toDegrees(getElbow()));
    SmartDashboard.putNumber("wristDeg", Math.toDegrees(getWrist()));

    SmartDashboard.putNumber("lastShoulderAngle", Math.toDegrees(shoulderSetpoint));
    SmartDashboard.putNumber("lastElbowAngle", Math.toDegrees(elbowSetpoint));
    SmartDashboard.putNumber("lastWristAngle", Math.toDegrees(wristSetpoint));

    SmartDashboard.putBoolean("shoulderEnabled", shoulderPIDEnabled);
    SmartDashboard.putBoolean("elbowEnabled", elbowPIDEnabled);
    SmartDashboard.putBoolean("wristEnabled", wristPIDEnabled);

    SmartDashboard.putNumber("shoulderPower", shoulderPower);
    SmartDashboard.putNumber("elbowPower", elbowPower);
    SmartDashboard.putNumber("wristPower", wristPower);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (shoulderPIDEnabled) {
      shoulderPower = shoulderPID.calculate(getShoulder(), shoulderSetpoint);
      if (!shoulderEncoder.isConnected()) {
        shoulderPower = 0;
      }
      shoulderJoint.set(shoulderPower);
      // shoulderJoint.set(0);
    }
    if (elbowPIDEnabled) {
      elbowPower = elbowPID.calculate(getElbow(), elbowSetpoint);
      if (!elbowEncoder.isConnected()) {
        elbowPower = 0;
      }
      elbowJoint.set(elbowPower);
      // elbowJoint.set(0);
    }
    if (wristPIDEnabled) {
      wristPower = wristPID.calculate(getWrist(), wristSetpoint);
      if (!wristEncoder.isConnected()) {
        wristPower = 0;
      }
      wristJoint.set(wristPower);
      // wristJoint.set(0);
    }

  }

}
