//      turn = Math.max(Math.min(turn, 0.4), -0.4);
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

import java.util.function.DoubleSupplier;

public class InvertDrive extends CommandBase {

    private final double PID_SETPOINT = 10; // 10 pixels of room for error?

    private final DriveTrain driveTrain;
    private final DoubleSupplier m_throttleInput;
    private final DoubleSupplier m_turnInput;

    public InvertDrive(DriveTrain driveTrain, DoubleSupplier throttleInput, DoubleSupplier turnInput) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.driveTrain = driveTrain;
        m_throttleInput = throttleInput;
        m_turnInput = turnInput;
        addRequirements(this.driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        driveTrain.setMotorArcadeDrive(- m_throttleInput.getAsDouble(), m_turnInput.getAsDouble());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
