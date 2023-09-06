// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.testing;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveDrive;

public class WhateverYouWant extends CommandBase {
    /*
     * make it go side to side in nfront of a retroreflective target
     * 
     */
    private final SwerveDrive m_SwerveDrive;
    PIDController robotGyro = new PIDController(0.0525, 0, 0.015);
    private double speed;
    private Timer timer;
    double tx;

    public WhateverYouWant(SwerveDrive swerveDrive) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerveDrive);
        this.m_SwerveDrive = swerveDrive;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.start();
        Timer timer = new Timer();

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        if (tx > 25) {
            speed = 1.0;
        } else if (tx < 25) {
            speed = -1.0;
        } else {
            speed = 0;
        }

        m_SwerveDrive.drive(0, speed * 0.5, 0, true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_SwerveDrive.setXWheels();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(4);
    }
}
