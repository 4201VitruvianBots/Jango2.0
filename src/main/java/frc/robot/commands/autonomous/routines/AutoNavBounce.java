package frc.robot.commands.autonomous.routines;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.SetDriveNeutralMode;
import frc.robot.commands.drivetrain.SetDriveShifters;
import frc.robot.commands.drivetrain.SetOdometry;
import frc.robot.constants.Constants;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.DriveTrain;
import frc.vitruvianlib.utils.TrajectoryUtils;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class AutoNavBounce extends SequentialCommandGroup {
    public AutoNavBounce(DriveTrain driveTrain, FieldSim fieldSim) {
        Pose2d[] waypoints = {
            new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))),
            new Pose2d(Units.inchesToMeters(90), Units.inchesToMeters(150), new Rotation2d(Units.degreesToRadians(90))),
            new Pose2d(Units.inchesToMeters(105), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(120))),
            new Pose2d(Units.inchesToMeters(150), Units.inchesToMeters(40), new Rotation2d(Units.degreesToRadians(180))),
            new Pose2d(Units.inchesToMeters(180), Units.inchesToMeters(139), new Rotation2d(Units.degreesToRadians(-90))),
            new Pose2d(Units.inchesToMeters(211), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(0))),
            new Pose2d(Units.inchesToMeters(252), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(0))),
            new Pose2d(Units.inchesToMeters(270), Units.inchesToMeters(147), new Rotation2d(Units.degreesToRadians(90))),       
            new Pose2d(Units.inchesToMeters(315), Units.inchesToMeters(100), new Rotation2d(Units.degreesToRadians(160))),
        };

        boolean[] pathIsReversed = {false, true, true, true, false, false, false, true};
        Pose2d startPosition = waypoints[0];


        TrajectoryConfig configA = new TrajectoryConfig(Units.feetToMeters(10), Units.feetToMeters(10));
        configA.setReversed(false);
        //configA.setEndVelocity(configA.getMaxVelocity());
        configA.addConstraint(new DifferentialDriveKinematicsConstraint(driveTrain.getDriveTrainKinematics(), configA.getMaxVelocity()));
        configA.addConstraint(new DifferentialDriveVoltageConstraint(driveTrain.getFeedforward(), driveTrain.getDriveTrainKinematics(),10));
        configA.addConstraint(new CentripetalAccelerationConstraint(1.5));

        addCommands(new SetDriveShifters(driveTrain, Constants.DriveConstants.inSlowGear),
                new SetOdometry(driveTrain, fieldSim, startPosition),
                new SetDriveNeutralMode(driveTrain, 0));

        double[] startVelocities = {configA.getMaxVelocity(), 0, configA.getMaxVelocity(), configA.getMaxVelocity(), 0, 
                configA.getMaxVelocity(), configA.getMaxVelocity(), 0, 0};
        double[] endVelocities = {0, configA.getMaxVelocity(), configA.getMaxVelocity(), 0, configA.getMaxVelocity(), 
                configA.getMaxVelocity(), 0, 0};

        var trajectoryStates = new ArrayList<Pose2d>();
        for(int i = 0; i < waypoints.length - 1; i++) {
            configA.setStartVelocity(startVelocities[i]);
            configA.setEndVelocity(endVelocities[i]);
            configA.setReversed(pathIsReversed[i]);
            Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints[i],
            List.of(),
            waypoints[i + 1],
            configA);

            trajectoryStates.addAll(trajectory.getStates().stream()
                    .map(state -> state.poseMeters)
                    .collect(Collectors.toList()));

            var command = TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory);
            addCommands(command);
        }
        fieldSim.getField2d().getObject("trajectory").setPoses(trajectoryStates);
    }
}

