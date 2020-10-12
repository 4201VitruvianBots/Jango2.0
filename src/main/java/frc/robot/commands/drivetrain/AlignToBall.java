/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

import java.util.function.DoubleSupplier;

public class AlignToBall extends CommandBase {

    private final double P_TERM = 0.05;
    private final double I_TERM = 0;
    private final double D_TERM = 0;

    private final DriveTrain driveTrain;
    private final Vision vision;
    private final DoubleSupplier throttle;
    private PIDController pid = new PIDController(P_TERM, I_TERM, D_TERM);

    public AlignToBall(DriveTrain driveTrain, Vision vision, DoubleSupplier throttle) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.driveTrain = driveTrain;
        this.vision = vision;
        this.throttle = throttle;
        addRequirements(this.driveTrain);
        addRequirements(this.vision);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (vision.hasPowerCell()) {
            double setpoint = driveTrain.getAngle() + vision.getPowerCellX();

            double leftVoltage = throttle.getAsDouble() * 12.0 + pid.calculate(driveTrain.getAngle(), setpoint);
            double rightVoltage = throttle.getAsDouble() * 12.0 - pid.calculate(driveTrain.getAngle(), setpoint);

            driveTrain.setVoltageOutput(leftVoltage, rightVoltage);
        }
    }
//If the vision has a powercell, use the angle to calculate how much voltage should go to each side of the Drive Train. 
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
