package org.team2168.commands.auto;

import org.team2168.commands.drivetrain.ArcadeDrive;
import org.team2168.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DriveForward extends SequentialCommandGroup {

    
    
  /** Creates a new DoNothing. */
  public DriveForward(Drivetrain drivetrain) {
    

    addCommands(
        new ArcadeDrive(drivetrain, () -> 0.5, () -> 0.0).withTimeout(5.0),
        new ArcadeDrive(drivetrain, () -> 0.0, () -> 0.0)
    );
  }
}

