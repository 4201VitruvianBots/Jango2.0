/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.orchestra;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Orchestra;

/**
 * An example command that uses an example subsystem.
 */
public class SetSong extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Orchestra m_orchestra;
  private String m_song;

  /*
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SetSong(Orchestra orchestra, String song) {
    m_orchestra = orchestra;
    m_song = song;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(orchestra);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //will see if the input to the funtion is either bumble, saber, or death. and play the appropriate sound file
    if(m_song=="bumble") {
      m_orchestra.loadSong("src/main/deploy/bumble_bee.mid");
    }else if(m_song=="saber") {
      //m_orchestra.loadSong("lightsaber sound.mid");
    }else if(m_song=="death") {
      //m_orchestra.loadSong("death sound.mid");
    }
    //plays the loaded song
    m_orchestra.playSong();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //stops the song
    m_orchestra.stopSong();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
