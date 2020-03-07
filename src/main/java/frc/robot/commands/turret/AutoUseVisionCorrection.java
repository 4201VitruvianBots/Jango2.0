/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

/**
 * An example command that uses an example subsystem.
 */
public class AutoUseVisionCorrection extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Turret m_turret;
    private final Vision m_vision;
    boolean turning, usingVisionSetpoint;
    private double setpoint;
    private double startTime;
    /**
     * Creates a new ExampleCommand.
     */
    public AutoUseVisionCorrection(Turret turretSubsystem, Vision visionSubsystem) {
        m_turret = turretSubsystem;
        m_vision = visionSubsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(turretSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_turret.getControlMode() == 1) {
            if (m_vision.getValidTarget()) {
                usingVisionSetpoint = true;
                if (!turning) {
                    m_vision.ledsOn();
                    setpoint = m_turret.getTurretAngle() + m_vision.getTargetX();

                    if (setpoint > m_turret.getMaxAngle()) {
                        setpoint -= 360;
                        if (setpoint < m_turret.getMinAngle())
                            setpoint = m_turret.getMinAngle();
                        turning = true;
                    } else if (setpoint < m_turret.getMinAngle()) {
                        setpoint += 360;
                        if (setpoint > m_turret.getMaxAngle())
                            setpoint = m_turret.getMaxAngle();
                        turning = true;
                    }
                } else {
                    if (m_turret.onTarget())
                        turning = false;
                }
            } else if (!m_vision.getValidTarget()) {
                usingVisionSetpoint = false;
                setpoint = m_turret.getTurretAngle();
            }

            m_turret.setRobotCentricSetpoint(setpoint);
//                m_turret.setFieldCentricSetpoint(setpoint);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (Math.abs(m_vision.getTargetX()) <= 3);
    }
}