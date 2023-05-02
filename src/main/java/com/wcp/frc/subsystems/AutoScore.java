// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.subsystems;

import com.wcp.frc.Constants;
import com.wcp.frc.Controls;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoScore extends SequentialCommandGroup {
  /** Creates a new AutoScore. */
  public AutoScore() {
    Intake intake = new Intake();
    Scores scores = new Scores();

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> scores.scoring()),
      new WaitCommand(1.5),
      new InstantCommand(() -> intake.setPercentOutput(.5)),
      new WaitCommand(.1),
     // new InstantCommand(() -> scores.setHeight(Constants.ElevatorConstants.HOLD)),
      new InstantCommand(() -> intake.setPercentOutput(0)),
      new WaitCommand(.5),
      new InstantCommand(() -> scores.zero())
    );
  }

}
