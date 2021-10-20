package frc.robot.commands.autonomous.routines;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
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
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.autonomous.SmartdashboardCommand;
import frc.robot.commands.autonomous.TurnInPlace;
import frc.robot.commands.drivetrain.SetDriveNeutralMode;
import frc.robot.commands.drivetrain.SetDriveShifters;
import frc.robot.commands.drivetrain.SetOdometry;
import frc.robot.commands.intake.SetIntakeSpeed;
import frc.robot.commands.intake.SetIntakePiston;
import frc.robot.commands.intake.SetIntakeStates;
import frc.robot.constants.Constants;
import frc.robot.simulation.FieldSim;
import frc.robot.simulation.SimulationShoot;
import frc.robot.subsystems.*;
import frc.vitruvianlib.utils.TrajectoryUtils;
import java.util.ArrayList;
import java.util.List;

public class AutoNavBarrel extends SequentialCommandGroup {
    public AutoNavBarrel(DriveTrain driveTrain, FieldSim fieldSim) {
        Pose2d[] waypoints = {
                new Pose2d(Units.inchesToMeters(40), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(150), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(176), Units.inchesToMeters(45), new Rotation2d(Units.degreesToRadians(-120))),
                new Pose2d(Units.inchesToMeters(124), Units.inchesToMeters(45), new Rotation2d(Units.degreesToRadians(120))),
                new Pose2d(Units.inchesToMeters(150), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(240), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(270), Units.inchesToMeters(120), new Rotation2d(Units.degreesToRadians(90))),
                new Pose2d(Units.inchesToMeters(210), Units.inchesToMeters(120), new Rotation2d(Units.degreesToRadians(-90))),
                new Pose2d(Units.inchesToMeters(285), Units.inchesToMeters(34), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(330), Units.inchesToMeters(60), new Rotation2d(Units.degreesToRadians(90))),
                new Pose2d(Units.inchesToMeters(285), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(180))),
                new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(180)))
        };

        Pose2d startPosition = waypoints[0];

        TrajectoryConfig configA = new TrajectoryConfig(Units.feetToMeters(9), Units.feetToMeters(6));
        configA.setReversed(false);
        //configA.setEndVelocity(configA.getMaxVelocity());
        configA.addConstraint(new DifferentialDriveKinematicsConstraint(driveTrain.getDriveTrainKinematics(), configA.getMaxVelocity()));
        configA.addConstraint(new DifferentialDriveVoltageConstraint(driveTrain.getFeedforward(), driveTrain.getDriveTrainKinematics(),10));
        configA.addConstraint(new CentripetalAccelerationConstraint(1.2)); // This is what we can change when we're actually testing

        addCommands(new SetDriveShifters(driveTrain, Constants.DriveConstants.inSlowGear),
                new SetOdometry(driveTrain, fieldSim, startPosition),
                new SetDriveNeutralMode(driveTrain, 0));

        for(int i = 0; i < waypoints.length - 1; i++) {
                if (i != 0) {
                        configA.setEndVelocity(configA.getMaxVelocity());
                        configA.setStartVelocity(configA.getMaxVelocity());
                }
                if (i == waypoints.length - 2) {
                        configA.setEndVelocity(0);
                }
                Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints[i],
                List.of(),
                waypoints[i + 1],
                configA);
            var command = TrajectoryUtils.generateVitruvianRamseteCommand(driveTrain, trajectory);
            addCommands(command);
        }
    }
}

