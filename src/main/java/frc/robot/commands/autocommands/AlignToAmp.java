package frc.robot.commands.autocommands;

import java.lang.invoke.ConstantCallSite;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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


public class AlignToAmp extends Command{
  private Swerve swerve;
  private Timer timer;
  private Shooter shooter;
  private double distanceSensed, desiredDistance, rotation, savedRotation, angle, savedAngle, powerMultplier;

  
  public AlignToAmp(Swerve swerve, Shooter shooter) {
    timer = new Timer();
    this.swerve = swerve;
    this.shooter = shooter;
    this.desiredDistance = 0;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.''
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    Limelight.setPipeline(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Limelight X", Limelight.getTx());
    rotation = -Limelight.getTx(); 
    distanceSensed = swerve.getDistanceSensor();
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

      int forward = distanceSensed < desiredDistance ? -1 : 1;
      int translation = savedRotation < 0 ? -1 : 1;
      double pythagoreanDistance = Math.sqrt(distanceSensed*distanceSensed + savedRotation*savedRotation);

        if(Math.abs(pythagoreanDistance) >= 144){
            swerve.driveSlow(new Translation2d(forward, translation), 0, false, true, 0.7);
        }
        else if(Math.abs(pythagoreanDistance) >= 25){
            swerve.driveSlow(new Translation2d(forward, translation), 0, false, true, 0.4);
        }
        else if(Math.abs(pythagoreanDistance) > 1){
            swerve.driveSlow(new Translation2d(forward, translation), 0, false, true, 0.2);
        }
        else{
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
        }
        // swerve.driveSlow(new Translation2d(0,0), savedRotation, true, true, maxSpeed);
  

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
    swerve.drive(new Translation2d(), 0, true, true);
    shooter.articulateOff();
    Limelight.setPipeline(0);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    if(((Limelight.getTx() != 0 && Math.abs(Limelight.getTx()) < 0.1) && (Limelight.getTy() != 0 && Math.abs(Limelight.getTy()) < 0.1))|| timer.hasElapsed(3)){
      return true;
    } else {
      return false;
    }
    // // return (Math.abs(rotation - swerve.getPose().getRotation().getDegrees()) <= 5 || timer.hasElapsed(3));
  }
}
