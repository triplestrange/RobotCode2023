// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gameplay.automations;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.Constants.JoystickButtons;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.JointAngles;

public class ArmPositions extends CommandBase {
  private final Arm arm;
  private double sAngle;
  private double eAngle;
  private double wAngle;

  public ArmPositions(JointAngles jointAngles, Arm arm) {
    addRequirements(arm);
    this.arm = arm;
    sAngle = jointAngles.shoulderAngle;
    eAngle = jointAngles.elbowAngle;
    wAngle = jointAngles.wristAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setShoulder(sAngle, 0);
    arm.setElbow(eAngle, 0);
    arm.setWrist(wAngle, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((arm.shoulderPID.getSetpoint().equals(arm.shoulderPID.getGoal()) &&
        arm.elbowPID.getSetpoint().equals(arm.elbowPID.getGoal()) &&
        arm.wristPID.getSetpoint().equals(arm.wristPID.getGoal()))) {
      JoystickButtons.m_operatorController.setRumble(RumbleType.kBothRumble, 1);
      return true;
    }
    return false;
  }

}
