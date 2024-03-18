// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterToPosition extends Command {
  private Shooter shooter;
  private double desiredPosition, currentDistance;
  /** Creates a new ShooterToDistance. */
  public ShooterToPosition(Shooter shooter, double desiredPosition) {
    this.shooter = shooter;
    this.desiredPosition = desiredPosition;

    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      double forward = shooter.getShooterEncoder() > desiredPosition ? .8 : -.8;
      if(Math.abs(shooter.getShooterEncoder() - desiredPosition) > 80000){
          shooter.articulateSlow(forward);
      }
      else if(Math.abs(shooter.getShooterEncoder() - desiredPosition) > 20000){
        shooter.articulateSlow(forward*.05);
      }
      else {
        shooter.articulateSlow(forward*.025);
      }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.articulateOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(shooter.getShooterEncoder() - desiredPosition)< 10000;
  }
}
