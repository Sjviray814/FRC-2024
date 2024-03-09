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
  private BooleanSupplier climberUp, climberDown;

  public DefaultClimber(BooleanSupplier climberUp, BooleanSupplier climberDown,  Climber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climber;
    this.climberUp = climberUp;
    this.climberDown = climberDown;

    addRequirements(climber);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(climberUp.getAsBoolean()){
      climber.climberOn();
    }
    else if (climberDown.getAsBoolean()){
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
