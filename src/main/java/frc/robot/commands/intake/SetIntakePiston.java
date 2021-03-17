/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

/**
 * Sets the piston that raises/lowers the intake.
 */
public class SetIntakePiston extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Intake intake;
    boolean extend;

    /**
     * 
     * @param intake The robot's intake
     * @param extend True means lowered and able to intake power cells, false means raised
     */
    public SetIntakePiston(Intake intake, boolean extend) {
        this.intake = intake;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(intake);
        this.extend = extend;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if(intake.getIntakePistonExtendStatus() != extend)
            intake.setintakePiston(extend);
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
        return true;
    }
}
