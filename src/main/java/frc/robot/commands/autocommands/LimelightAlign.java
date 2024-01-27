package frc.robot.commands.autocommands;

import java.lang.invoke.ConstantCallSite;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Limelight;
import frc.robot.util.Limelight.LightMode;


public class LimelightAlign extends Command{
  private Swerve swerve;
  private Timer timer;
  private double angleToPole, distanceToBase, lengthToPole, distanceFromTarget, strafeOffset, rotation, savedRotation;

  
  public LimelightAlign(Swerve swerve) {
    timer = new Timer();
    this.swerve = swerve;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.''
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Limelight X", Limelight.getTx());
    rotation = -Limelight.getTx(); 

    // angleToPole = 0;
    // distanceToBase = 0;
    // lengthToPole = 0;
    // angleToPole = Math.toRadians(Constants.Snake.limelightAngle + Limelight.getTy());
    // distanceToBase = (Constants.Snake.midPoleHeight - Constants.Snake.limelightHeight) / Math.tan(Constants.Snake.limelightAngle);
    // lengthToPole = Constants.Snake.lengthToMidPole;

    


    //Calculates distance robot has to drive forward to be on the edge of the scoring station
    // distanceFromTarget = distanceToBase - lengthToPole;

    //calculates how far left or right we have to strafe to align with the pole we are scoring on. 
    // strafeOffset = distanceToBase * Math.tan(Math.toRadians(-Limelight.getTx()));


    // SmartDashboard.putNumber("distanceFromTarget", distanceFromTarget);
    // SmartDashboard.putNumber("strafeOffset", strafeOffset);
    SmartDashboard.putNumber("Rotation Offset", rotation);

    // PathPlannerTrajectory AlignToScore = PathPlanner.generatePath(
    //   new PathConstraints(4.0, 4.0),
    //   new PathPoint(swerve.get, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)), // position, heading(direction of travel), holonomic rotation
    //   new PathPoint(new Translation2d(distanceFromTarget, strafeOffset), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)) // position, heading(direction of travel), holonomic rotation
    //   );

    // swerve.followTrajectoryCommand(AlignToScore);
    
    //   swerve.driveStraight(new Translation2d(distanceFromTarget, strafeOffset).times(2.0), 0, true, true);
    // swerve.drive(new Translation2d(0, 0), (Rotation2d.fromRadians(rotation)).getRadians(), true, true);
      if(rotation != 0){
        savedRotation = rotation;
      }

        if(Math.abs(savedRotation) >= 12){
            swerve.driveSlow(new Translation2d(0, 0), savedRotation, true, true, 0.5);
        }
        else if(Math.abs(savedRotation) > 1){
            swerve.driveSlow(new Translation2d(0, 0), savedRotation, true, true, 0.1);
        }
        else{
            swerve.driveSlow(new Translation2d(0, 0), savedRotation, true, true, 0.05);
        }
  

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
    swerve.drive(new Translation2d(), 0, true, true);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if((Limelight.getTx() != 0 && Math.abs(Limelight.getTx()) < 0.1) || timer.hasElapsed(3)){
      return true;
    } else {
      return false;
    }
    // // return (Math.abs(rotation - swerve.getPose().getRotation().getDegrees()) <= 5 || timer.hasElapsed(3));
  }
}
