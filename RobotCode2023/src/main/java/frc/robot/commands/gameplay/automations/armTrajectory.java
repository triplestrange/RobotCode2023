package frc.robot.commands.gameplay.automations;

import edu.wpi.first.wpilibj2.command.CommandBase;


import java.util.List;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.JoystickButtons;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.JointAngles;

public class armTrajectory extends CommandBase{
    private final Arm arm;
    private final Pose2d targetPose2d;
    private Trajectory trajectory;
    private final Timer timer = new Timer();
    private double initialWristAngle;
    private double finalWristAngle; 
    public armTrajectory(Pose2d targetPose2d, Arm arm, double finalWristAngle) {
        addRequirements(arm);
        this.arm = arm;
        this.targetPose2d = targetPose2d;
        this.finalWristAngle = finalWristAngle;
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        Translation2d currentPos  = arm.getArmPosition();
        double initialAngle;
        double endingAngle;
        initialAngle = arm.getWrist();
        if (currentPos.getX() < (13 * 0.0254))    {
            initialAngle = Math.PI / 2; 
        }

        else if (currentPos.getY() > (7 * 0.0254)) {
            initialAngle = Math.PI / 2;
        }

        else {

            initialAngle = Math.PI; 
        }

        if (targetPose2d.getX() < (13 * 0.0254))    {
            endingAngle = Math.PI / -2; 
        }

        else if (targetPose2d.getY() > (7 * 0.0254)) {
            endingAngle = Math.PI / -2;
        }

        else {

            endingAngle = 0; 
        }

        trajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(currentPos, new Rotation2d(initialAngle)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(targetPose2d.getX(), targetPose2d.getY(), new Rotation2d(endingAngle)),
            Constants.armConstants.config);
        
        timer.reset();
        timer.start();
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Trajectory.State targetState = trajectory.sample(timer.get());

        double wristAngle = 0;

        JointAngles targetAngles = JointAngles.anglesFrom2D(targetState.poseMeters.getX(), targetState.poseMeters.getY(), wristAngle);
        Rotation2d direction = targetState.poseMeters.getRotation();
        Vector<N2> linearVelocity = VecBuilder.fill(direction.getCos(), direction.getSin()).times(targetState.velocityMetersPerSecond);
         

        Vector <N2> angleVector = arm.angleVelocity(linearVelocity);
        arm.setShoulder(targetAngles.shoulderAngle, angleVector.get(0, 0));
        arm.setElbow(targetAngles.elbowAngle, angleVector.get(1, 0));
        arm.setWrist(initialWristAngle + (finalWristAngle - initialWristAngle) * timer.get()/trajectory.getTotalTimeSeconds(), 
        finalWristAngle - initialWristAngle / trajectory.getTotalTimeSeconds());
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
  
  
  }

