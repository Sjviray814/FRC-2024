// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.defaultcommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;


public class DefaultClimber extends Command {
  /** Creates a new DefaultClimber. */
  private Climber climber;
  private BooleanSupplier climberUp, climberDown, bothReverse, bothOn;

  public DefaultClimber(BooleanSupplier climberUp, BooleanSupplier climberDown, BooleanSupplier bothReverse, BooleanSupplier bothOn, Climber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climber;
    this.climberUp = climberUp;
    this.climberDown = climberDown;
    this.bothReverse = bothReverse;
    this.bothOn = bothOn;

    addRequirements(climber);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(bothOn.getAsBoolean()){
      climber.climberOn();
    }
    else if(climberUp.getAsBoolean()){
      climber.leftClimberOn();
    }
    else if (climberDown.getAsBoolean()){
      climber.rightClimberOn();
    }
    else if(bothReverse.getAsBoolean()){
      climber.climberReverse();
    }
    else{
      climber.climberStop();
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
