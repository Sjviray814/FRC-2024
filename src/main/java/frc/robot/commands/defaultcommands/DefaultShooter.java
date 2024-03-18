// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.defaultcommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;


public class DefaultShooter extends Command {
  /** Creates a new Defaultshooter. */
  private Shooter shooter;
  private BooleanSupplier shooterUp, shooterDown, shooterOn, shooterFeed, shooterSlow, shooterTrap, positioningRing;

  public DefaultShooter(BooleanSupplier shooterUp, BooleanSupplier shooterDown, BooleanSupplier shooterOn, BooleanSupplier shooterFeed, BooleanSupplier shooterSlow, BooleanSupplier shooterTrap, BooleanSupplier positioningRing, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.shooterUp = shooterUp;
    this.shooterDown = shooterDown;
    this.shooterOn = shooterOn;
    this.shooterFeed = shooterFeed;
    this.shooterSlow = shooterSlow;
    this.shooterTrap = shooterTrap;
    this.positioningRing = positioningRing;

    addRequirements(shooter);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooterOn.getAsBoolean()){
      shooter.shooterOn();
    }
    else if(shooterSlow.getAsBoolean()){
      shooter.shooterSlow();
    }
    else if(shooterTrap.getAsBoolean()){
      shooter.shooterTrap();
    }
    else{
      shooter.shooterOff();
    }

    if(shooterUp.getAsBoolean()){
      shooter.articulateUp();
    }
    else if(shooterDown.getAsBoolean()){
      shooter.articulateDown();
    }
    else{
      shooter.articulateOff();
    }

    if(shooterFeed.getAsBoolean() && positioningRing.getAsBoolean()){
      if(shooter.getShooterBeamBreak()){
        shooter.feedSlow();
      }
      else{
        shooter.feedBack();
      }
    }
    else if(shooterFeed.getAsBoolean()){
        shooter.feed();
    }
    else{
        shooter.feedOff();
    }
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
