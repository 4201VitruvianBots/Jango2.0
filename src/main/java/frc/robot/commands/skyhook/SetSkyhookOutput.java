/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.skyhook;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Skyhook;

import java.util.function.DoubleSupplier;

/**
 * An example command that uses an example subsystem.
 */
public class SetSkyhookOutput extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Climber m_climber;
    private final Skyhook m_skyhook;
    private final DoubleSupplier m_output;
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    double value;

    public SetSkyhookOutput(Climber climber, Skyhook skyhook, DoubleSupplier output) {
        m_climber = climber;
        m_skyhook = skyhook;
        m_output = output;
        // Use addRequirements() here to declare skyhook dependencies.
        addRequirements(skyhook);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SmartDashboardTab.putNumber("Skyhook", "Output", m_output.getAsDouble());
        double output = m_output.getAsDouble();
        if(m_climber.getClimbState() && Math.abs(output) > 0.25)
            m_skyhook.setSkyhook(output);
        else {
            m_skyhook.setSkyhook(0);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_skyhook.setSkyhook(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
