// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Electrical;

public class Intake extends SubsystemBase {

  private final CANSparkMax rollers;
  double intakeSpeed = 1;

  /** Creates a new Intake. */
  public Intake() {
    super();
    rollers = new CANSparkMax(Electrical.ROLLERS, MotorType.kBrushless);

    rollers.setSmartCurrentLimit(30);
    rollers.setIdleMode(IdleMode.kBrake);
    // encoder to determine offset angle
  }

  public void runIntake() {
    rollers.set(-intakeSpeed);
  }

  public void runOutake() {
    rollers.set(intakeSpeed);
  }

  public void intakeOff() {
    rollers.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
