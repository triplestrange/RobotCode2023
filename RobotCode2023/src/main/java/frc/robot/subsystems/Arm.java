// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Electrical;

public class Arm extends SubsystemBase {
  private final CANSparkMax shoulderJoint;
  private final CANSparkMax elbowJoint;
  private final CANSparkMax wristJoint;

  private final AbsoluteEncoder shoulderEncoder;
  private final AbsoluteEncoder elbowEncoder;
  private final AbsoluteEncoder wristEncoder;

  private final SparkMaxPIDController shoulderPID;
  private final SparkMaxPIDController elbowPID;
  private final SparkMaxPIDController wristPID;

  /** Creates a new Arm. */
  public Arm() {
    super();
    shoulderJoint = new CANSparkMax(Electrical.SHOULDER, MotorType.kBrushless);
    elbowJoint = new CANSparkMax(Electrical.ELBOW, MotorType.kBrushless);
    wristJoint = new CANSparkMax(Electrical.WRIST, MotorType.kBrushless);

    // TODO: determine channel & call getCalibration() on each absolute
    // encoder to determine offset angle
    shoulderEncoder = new AbsoluteEncoder(4, 0);
    elbowEncoder = new AbsoluteEncoder(7, 0);
    wristEncoder = new AbsoluteEncoder(6, 0);

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
    shoulderJoint.set(motorPowerShoulder);
    elbowJoint.set(motorPowerElbow);
    wristJoint.set(motorPowerWrist);
  }
   public void setShoulder(double angle) {
    shoulderPID.setReference(angle, ControlType.kPosition);
   }
    public double getShoulder() {
    return shoulderJoint.get();
  }
  
  /*
   * Sets angle between humerus and radius according to shoulder joint angle
   * Angle 0 = radius is on top of humerus
   */
  public void setElbow(double angle) {
    elbowPID.setReference(angle, ControlType.kPosition);
  }

  public double getElbow() {
    return elbowJoint.get();
  }
  /*
   * Sets angle between metacarpals and radius
   * Angle 0 = metacarpals is on top of radius
   */
  public void setWrist(double angle) {
    wristPID.setReference(angle, ControlType.kPosition);
  }

  public double getWrist() {
    return wristJoint.get();
  }

  public void initializePID(SparkMaxPIDController controller) {
    int kP = 1;
    int kI = 0; 
    int kD = 0;
    int kMinOutput = -1;
    int kMaxOutput = 1;

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
    double a = 38;
    double b = 34;
    double c = Math.hypot(width, height);

    double angle1 = Math.acos((Math.pow(a, 2) + Math.pow(b, 2) - Math.pow(c, 2))/(2 * a * b));
    
    // figure out how to calculate shoulder angle
    double angle2 = Math.asin((Math.sin(angle1)/c) * b);
    
// from horizontal
    double shoulderAngle = Math.atan2(height,width) + angle2;

    double elbowAngle = angle1+shoulderAngle - Math.PI;

    JointAngles angles = new JointAngles(shoulderAngle, elbowAngle, wristAngle);
    return angles;

  }
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
