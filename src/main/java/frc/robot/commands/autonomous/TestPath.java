/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveTrain;

import java.util.ArrayList;

/**
 * An example command that uses an example subsystem.
 */
public class TestPath extends CommandBase {
    private static final double m_period = 0.02;
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveTrain m_driveTrain;
    private final ArrayList<Pose2d> m_path;
    private final boolean m_isInverted;
    private Trajectory trajectory;
    private Notifier m_notifier;

    /**
     * Creates a new ExampleCommand.
     *
     * @param driveTrain The subsystem used by this command.
     */
    public TestPath(DriveTrain driveTrain, ArrayList<Pose2d> path, boolean isInverted) {
        m_driveTrain = driveTrain;
        m_path = path;
        m_isInverted = isInverted;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_driveTrain.resetOdometry(new Pose2d(), new Rotation2d());
        m_driveTrain.resetEncoderCounts();

        var trajectoryConstraints = new DifferentialDriveKinematicsConstraint(m_driveTrain.getDriveTrainKinematics(),
                3);

        var trajectoryConfig = new TrajectoryConfig(Units.feetToMeters(8), Units.feetToMeters(4));

        trajectoryConfig.setReversed(m_isInverted);

        trajectory = TrajectoryGenerator.generateTrajectory(m_path, trajectoryConfig);

        RamseteCommand followTrajectory = new RamseteCommand(
                trajectory,
                m_driveTrain :: getRobotPose,
                new RamseteController(),
                m_driveTrain.getFeedforward(),
                m_driveTrain.getDriveTrainKinematics(),
                m_driveTrain :: getSpeeds,
                m_driveTrain.getLeftPIDController(),
                m_driveTrain.getRightPIDController(),
                m_driveTrain :: setVoltageOutput,
                m_driveTrain
        );
        CommandScheduler.getInstance().schedule(followTrajectory);
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
