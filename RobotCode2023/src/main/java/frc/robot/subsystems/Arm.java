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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Electrical;

public class Arm extends SubsystemBase {
  private final CANSparkMax shoulderJoint;
  private final CANSparkMax elbowJoint;
  private final CANSparkMax wristJoint;

  private final AbsoluteEncoder shoulderEncoder;
  private final AbsoluteEncoder elbowEncoder;
  private final AbsoluteEncoder wristEncoder;

  private final RelativeEncoder shoulderRelativeEncoder;
  private final RelativeEncoder elbowRelativeEncoder;
  private final RelativeEncoder wristRelativeEncoder;

  private final SparkMaxPIDController shoulderPID;
  private final SparkMaxPIDController elbowPID;
  private final SparkMaxPIDController wristPID;

  private double lastShoulderAngle;
  private double lastElbowAngle;
  private double lastWristAngle;

  private double shoulderInit;
  private double elbowInit;
  private double wristInit;

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

    // encoder to determine offset angle
    shoulderEncoder = new AbsoluteEncoder(7, 152.678402, true);
    elbowEncoder = new AbsoluteEncoder(6, 226.560079);
    wristEncoder = new AbsoluteEncoder(5, 158.522452);

    shoulderRelativeEncoder = shoulderJoint.getEncoder();
    elbowRelativeEncoder = elbowJoint.getEncoder();
    wristRelativeEncoder = wristJoint.getEncoder();

    shoulderRelativeEncoder.setPositionConversionFactor(2 * Math.PI / Constants.armConstants.GR_SHOULDER);
    elbowRelativeEncoder.setPositionConversionFactor(2 * Math.PI / Constants.armConstants.GR_ELBOW);
    wristRelativeEncoder.setPositionConversionFactor(2 * Math.PI / Constants.armConstants.GR_WRIST);

    // Old method without abs encoders
    // shoulderInit = Math.PI / 2;
    // elbowInit = Math.toRadians(-71.657202);
    // wristInit = 0;

    // With abs encoders
    shoulderInit = (Math.PI / 2) + MathUtil.angleModulus(shoulderEncoder.getAngle()) * Constants.armConstants.ENC_GR_SHOULDER;
    elbowInit = Math.toRadians(-71.657202) + MathUtil.angleModulus(elbowEncoder.getAngle()) * Constants.armConstants.ENC_GR_SHOULDER;
    wristInit = 0;
    
    shoulderRelativeEncoder.setPosition(shoulderInit);
    elbowRelativeEncoder.setPosition(elbowInit);
    wristRelativeEncoder.setPosition(wristInit - shoulderInit * 38 / 26 + elbowInit * 38 / 26);

    shoulderJoint.setSmartCurrentLimit(40);
    elbowJoint.setSmartCurrentLimit(40);
    wristJoint.setSmartCurrentLimit(20);

    shoulderPID = shoulderJoint.getPIDController();
    elbowPID = elbowJoint.getPIDController();
    wristPID = wristJoint.getPIDController();

    shoulderJoint.setIdleMode(IdleMode.kBrake);
    elbowJoint.setIdleMode(IdleMode.kBrake);
    wristJoint.setIdleMode(IdleMode.kBrake);

    initializePID(shoulderPID);
    initializePID(elbowPID);
    initializePID(wristPID);

    shoulderJoint.burnFlash();
    elbowJoint.burnFlash();
    wristJoint.burnFlash();
  }
  public void moveArm(double motorPowerShoulder, double motorPowerElbow, double motorPowerWrist)  {
    if (((getArmPosition().getX() > Units.inchesToMeters(48 + 18) && motorPowerShoulder < 0)) || (getArmPosition().getX() > 18 - 9) && (getShoulder() > Math.toRadians(97) && motorPowerShoulder > 0)) {
      motorPowerShoulder = 0;
    }
    if ((getArmPosition().getX() > Units.inchesToMeters(48 + 18)) && ((motorPowerElbow > 0 && getElbow() < 0) || (motorPowerElbow < 0 && getElbow() > 0))) {
      motorPowerElbow = 0;
    }
    if (getArmPosition().getY() > Units.inchesToMeters(78 - 12.25) && motorPowerElbow > 0) {
      motorPowerElbow = 0;
    }
    if (getArmPosition().getY() > Units.inchesToMeters(78 - 12.25) && ((motorPowerShoulder > 0 && getShoulder() < Math.PI / 2) || (motorPowerShoulder < 0 && getShoulder() > Math.PI / 2)))  {
      motorPowerShoulder = 0;
    }

    if (Math.abs(motorPowerShoulder) < 0.05)  {
      shoulderPID.setReference(lastShoulderAngle, ControlType.kPosition);
    } else {
      shoulderJoint.set(motorPowerShoulder);
      lastShoulderAngle = getShoulder();
    }

    if (Math.abs(motorPowerElbow) < 0.05 )  {
      elbowPID.setReference(lastElbowAngle, ControlType.kPosition);
    } else {
      elbowJoint.set(motorPowerElbow + 0.05*Math.cos(getElbow()));
      lastElbowAngle = getElbow();
    }

    if (Math.abs(motorPowerWrist) < 0.05 )  {
      // wristPID.setReference(lastWristAngle, ControlType.kPosition);
      setWrist(lastWristAngle, 0);
    } else {
      wristJoint.set(motorPowerWrist);
      lastWristAngle = getWrist();
    }
    
   }
   public void setShoulder(double angle, double ffSpeed) {
    shoulderPID.setReference(angle, ControlType.kPosition, 
    0, ffSpeed/Constants.armConstants.FREE_SPEED_SHOULDER);
    lastShoulderAngle = angle;

  }
    public double getShoulder() {
    return shoulderRelativeEncoder.getPosition();
  }
  
  /*
   * Sets angle between humerus and radius according to shoulder joint angle
   * Angle 0 = radius is on top of humerus
   */
  public void setElbow(double angle, double ffSpeed) {
    elbowPID.setReference(angle, ControlType.kPosition,
        0, ffSpeed / Constants.armConstants.FREE_SPEED_ELBOW);
    lastElbowAngle = angle;
  }

  public double getElbow() {
    return elbowRelativeEncoder.getPosition();
  }
  /*
   * Sets angle between metacarpals and radius
   * Angle 0 = metacarpals is on top of radius
   */
  public void setWrist(double angle, double ffSpeed) {
    wristPID.setReference(angle - getShoulder() * 38 / 26 + getElbow() * 38 / 26, ControlType.kPosition, 
    0, ffSpeed/Constants.armConstants.FREE_SPEED_WRIST);
    lastWristAngle = angle;
  }

  public double getWrist() {
    return wristRelativeEncoder.getPosition() + getShoulder() * 38 / 26 - getElbow() * 38 / 26;
  }

  public void initializePID(SparkMaxPIDController controller) {
    int kP = 7;
    int kI = 0; 
    int kD = 0;
    double kMinOutput = -0.5;
    double kMaxOutput = 0.5;

    controller.setP(kP);
    controller.setI(kI);
    controller.setD(kD);
    controller.setOutputRange(kMinOutput, kMaxOutput);
  }

  /*
   * PARAMETERS
   *  width: horizontal distance between shoulder pivot point and where wrist pivot point should be
   *  height: vertical distance between shoulder pivot point and where wrist pivot point should be
   * 
   * c = (a^2 + b^2 - 2abcosC)^(1/2)
   * a^2 + b^2 - c^2 = 2abcosC
   * sinA/a = sinB/b = sinC/c
   */
  

public static class JointAngles  {

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
    double a = Constants.armConstants.SHOULDER_LENGTH;
    double b = Constants.armConstants.ELBOW_LENGTH;
    double c = Math.hypot(width, height);

    double angle1 = Math.acos((Math.pow(a, 2) + Math.pow(b, 2) - Math.pow(c, 2))/(2 * a * b));
    
    // figure out how to calculate shoulder angle
    double angle2 = Math.asin((Math.sin(angle1)/c) * b);
    if (a + b < c)  {
      angle1 = Math.PI;
      angle2 = 0;
    }
// from horizontal
    double shoulderAngle = Math.atan2(height,width) + angle2;

    double elbowAngle = angle1+shoulderAngle - Math.PI;

    JointAngles angles = new JointAngles(shoulderAngle, elbowAngle, wristAngle);
    return angles;

  }

  
}

public Vector<N2> angleVelocity(Vector <N2> velocity) {
  double sAngle = shoulderRelativeEncoder.getPosition();
  double eAngle = elbowRelativeEncoder.getPosition();
  Matrix<N2,N2> MAngularToLinear = new MatBuilder<>(N2.instance, N2.instance).fill(
    -Constants.armConstants.SHOULDER_LENGTH * Math.sin(sAngle),
    -Constants.armConstants.ELBOW_LENGTH * Math.sin(eAngle),
    Constants.armConstants.SHOULDER_LENGTH * Math.cos(sAngle),
    Constants.armConstants.ELBOW_LENGTH * Math.cos(eAngle)
    
  );

  return new Vector <> (MAngularToLinear.inv().times(velocity));
}

public Translation2d getArmPosition()  {
  double sAngle = shoulderRelativeEncoder.getPosition();
  double eAngle = elbowRelativeEncoder.getPosition();

  double intakeX = Constants.armConstants.ELBOW_LENGTH * Math.cos(eAngle) 
  + Constants.armConstants.SHOULDER_LENGTH * Math.cos(sAngle);
  double intakeY = Constants.armConstants.ELBOW_LENGTH * Math.sin(eAngle)
  + Constants.armConstants.SHOULDER_LENGTH * Math.sin(sAngle);

  return new Translation2d(intakeX, intakeY);
}

public void setArmAngles()  {

  lastShoulderAngle = getShoulder();
  lastElbowAngle = getElbow();
  lastWristAngle = getWrist();
}

public void resetArmEncoders()  {
  shoulderRelativeEncoder.setPosition(shoulderInit);
  elbowRelativeEncoder.setPosition(elbowInit);
  wristRelativeEncoder.setPosition(wristInit - shoulderInit * 38 / 26 + elbowInit * 38 / 26);

  lastShoulderAngle = shoulderInit;
  lastElbowAngle = elbowInit;
  lastWristAngle = wristInit;
}

public void updateSmartDashBoard()  {
    SmartDashboard.putNumber("shoulderDeg", Math.toDegrees(getShoulder()));
    SmartDashboard.putNumber("elbowDeg", Math.toDegrees(getElbow()));
    SmartDashboard.putNumber("wristDeg", Math.toDegrees(getWrist()));
    Translation2d currentPos = getArmPosition();
    SmartDashboard.putNumber("armX", currentPos.getX());
    SmartDashboard.putNumber("armY", currentPos.getY());

    SmartDashboard.putNumber("lastShoulderAngle", lastShoulderAngle);
    SmartDashboard.putNumber("lastElbowAngle", lastElbowAngle);
    SmartDashboard.putNumber("lastWristAngle", lastWristAngle);

    SmartDashboard.putNumber("shoulderPositionOffset", Units.radiansToDegrees(shoulderEncoder.getAngle()));
    SmartDashboard.putNumber("elbowPositionOffset", Units.radiansToDegrees(elbowEncoder.getAngle()));
    SmartDashboard.putNumber("wristPositionOffset", Units.radiansToDegrees(wristEncoder.getAngle()));

    SmartDashboard.putNumber("Shoulder", shoulderInit);
    SmartDashboard.putNumber("Elbow Angle", elbowInit);
    SmartDashboard.putNumber("Wrist", wristInit);

}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   
  }
}
