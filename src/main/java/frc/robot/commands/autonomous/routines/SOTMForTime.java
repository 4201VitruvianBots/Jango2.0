/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous.routines;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.turret.ShootOnTheMove;


/**
 * An example command that uses an example subsystem.
 */
public class SOTMForTime extends CommandBase {


    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private final ShootOnTheMove m_shootOnTheMove;
    private final double m_duration;
    private double startTime;

    public SOTMForTime(ShootOnTheMove shootOnTheMove, double duration) {
        m_shootOnTheMove = shootOnTheMove;
        m_duration = duration;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {


    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() >= startTime + m_duration;
    }
}

