// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterToDistance extends Command {
  private Shooter shooter;
  private double desiredDistance, currentDistance;
  /** Creates a new ShooterToDistance. */
  public ShooterToDistance(Shooter shooter, double desiredDistance) {
    this.shooter = shooter;
    this.desiredDistance = desiredDistance;

    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if(shooter.getDistanceSensor() < desiredDistance){
        shooter.articulateUp();
      }
      else{
        shooter.articulateDown();
      }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooter.getDistanceSensor() - desiredDistance < 1;
  }
}
