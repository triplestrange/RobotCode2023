// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
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
    shoulderEncoder = new AbsoluteEncoder(0, 0);
    elbowEncoder = new AbsoluteEncoder(0, 0);
    wristEncoder = new AbsoluteEncoder(0, 0);

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

  public void setShoulder(double pos) {
    shoulderPID.setReference(pos, ControlType.kPosition);
  }

  public void setElbow(double pos) {
    elbowPID.setReference(pos, ControlType.kPosition);
  }

  public void setWrist(double pos) {
    wristPID.setReference(pos, ControlType.kPosition);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
