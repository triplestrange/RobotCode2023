// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Electrical;

public class Intake extends SubsystemBase {
  private final CANSparkMax fingerJoint;
  private final CANSparkMax rollers;

  private final AbsoluteEncoder fingerEncoder;

  private final SparkMaxPIDController fingerPID;
  double intakeSpeed = 1;
  /** Creates a new Intake. */
  public Intake() {
    super();
    fingerJoint = new CANSparkMax(Electrical.FINGER, MotorType.kBrushless);
    rollers = new CANSparkMax(Electrical.ROLLERS, MotorType.kBrushless);
    
    // TODO: determine channel & call getCalibration() on each absolute
    // encoder to determine offset angle
    fingerEncoder = new AbsoluteEncoder(5, 0);

    fingerPID = fingerJoint.getPIDController();

    
  }
  public void moveFinger(double motorPowerFinger)  {
    fingerJoint.set(motorPowerFinger);
  }
  public void runIntake()  {
      rollers.set(intakeSpeed);
  }
  public void runOutake() {
    rollers.set(-intakeSpeed);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
