package frc.robot.commands.autocommands;

import java.lang.invoke.ConstantCallSite;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Limelight;
import frc.robot.util.Limelight.LightMode;


public class LimelightShooterAlign extends Command{
  private Shooter shooter;
  private Timer timer;
  private double angle, savedAngle;
  private int powerMultplier;

  
  public LimelightShooterAlign(Shooter shooter) {
    timer = new Timer();
    this.shooter = shooter;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
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
    SmartDashboard.putNumber("Limelight Y", Limelight.getTy());
    angle = Limelight.getTy(); 

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

    // PathPlannerTrajectory AlignToScore = PathPlanner.generatePath(
    //   new PathConstraints(4.0, 4.0),
    //   new PathPoint(swerve.get, angle2d.fromDegrees(0), angle2d.fromDegrees(0)), // position, heading(direction of travel), holonomic angle
    //   new PathPoint(new Translation2d(distanceFromTarget, strafeOffset), angle2d.fromDegrees(0), angle2d.fromDegrees(0)) // position, heading(direction of travel), holonomic angle
    //   );

    // swerve.followTrajectoryCommand(AlignToScore);
    
    //   swerve.driveStraight(new Translation2d(distanceFromTarget, strafeOffset).times(2.0), 0, true, true);
    // swerve.drive(new Translation2d(0, 0), (angle2d.fromRadians(angle)).getRadians(), true, true);
      if(angle != 0){
        savedAngle = angle;
      }

      powerMultplier = savedAngle > 0 ? -1 : 1;

        if(Math.abs(savedAngle) >= 12){
            shooter.articulateSlow(.7*powerMultplier);
        }
        else if(Math.abs(savedAngle) > 4){
            shooter.articulateSlow(.3*powerMultplier);
        }
        else{
            shooter.articulateSlow(.1*powerMultplier);
        }
        // swerve.driveSlow(new Translation2d(0,0), savedangle, true, true, maxSpeed);
  

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
    shooter.shooterOff();
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if((Limelight.getTy() != 0 && Math.abs(Limelight.getTy()) < 0.1) || timer.hasElapsed(3)){
      return true;
    } else {
      return false;
    }
    // // return (Math.abs(angle - swerve.getPose().getangle().getDegrees()) <= 5 || timer.hasElapsed(3));
  }
}
