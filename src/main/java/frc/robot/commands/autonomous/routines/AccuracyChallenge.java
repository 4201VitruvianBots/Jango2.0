package frc.robot.commands.autonomous.routines;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.autonomous.SmartdashboardCommand;
import frc.robot.commands.autonomous.TurnInPlace;
import frc.robot.commands.drivetrain.DriveBackwardDistance;
import frc.robot.commands.drivetrain.DriveForwardDistance;
import frc.robot.commands.drivetrain.SetDriveNeutralMode;
import frc.robot.commands.drivetrain.SetDriveShifters;
import frc.robot.commands.drivetrain.SetOdometry;
import frc.robot.commands.intake.SetIntakeSpeed;
import frc.robot.commands.intake.SetIntakePiston;
import frc.robot.commands.intake.SetIntakeStates;
import frc.robot.commands.shooter.ControlledFireNew;
import frc.robot.commands.shooter.SetRPM;
import frc.robot.constants.Constants;
import frc.robot.simulation.FieldSim;
import frc.robot.simulation.SimConstants;
import frc.robot.simulation.SimulationShoot;
import frc.robot.subsystems.*;
import frc.vitruvianlib.utils.TrajectoryUtils;
import java.util.ArrayList;
import java.util.List;

public class AccuracyChallenge extends SequentialCommandGroup {
    private int baseRPM = 3000; // The RPM to set the shooter to while not calculating precisely
    public AccuracyChallenge(DriveTrain driveTrain, Shooter shooter, Indexer indexer, FieldSim fieldSim, int index) {
        double[] distancesToDrive = {
            Units.inchesToMeters(180 + 6) + SimConstants.robotLength, // Green
            Units.inchesToMeters(120 + 6) + SimConstants.robotLength, // Yellow
            Units.inchesToMeters(120 - 6) - SimConstants.robotLength, // Blue
            Units.inchesToMeters(60 - 6) - SimConstants.robotLength // Red
        };
        addCommands(
                new SetDriveShifters(driveTrain, Constants.DriveConstants.inSlowGear),
                new SetOdometry(driveTrain, fieldSim, new Pose2d()),
                new SetDriveNeutralMode(driveTrain, 0),
                new SetRPM(shooter, 3000),
                new DriveForwardDistance(driveTrain, fieldSim, distancesToDrive[index]).andThen(() -> driveTrain.setVoltageOutput(0, 0)),
                new InstantCommand(() -> shooter.setIdealRPM()),
                new ControlledFireNew(shooter, indexer).withTimeout(3),
                new SetRPM(shooter, 3000),
                new DriveBackwardDistance(driveTrain, fieldSim, distancesToDrive[index]).andThen(() -> driveTrain.setVoltageOutput(0, 0))
        );
    }
}

