package frc.robot.commands.autonomous.routines;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.EllipticalRegionConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.SetDriveNeutralMode;
import frc.robot.commands.drivetrain.SetDriveShifters;
import frc.robot.commands.drivetrain.SetOdometry;
import frc.robot.constants.Constants;
import frc.robot.simulation.FieldSim;
import frc.robot.simulation.SimConstants;
import frc.robot.subsystems.DriveTrain;
import frc.vitruvianlib.utils.TrajectoryUtils;

import java.util.List;

public class AutoNavSlalom extends SequentialCommandGroup {
    public AutoNavSlalom(DriveTrain driveTrain, FieldSim fieldSim) {
        int[][] waypointsRaw = {
                {40,30,0},
                // {90,60,60},
                {120,90,0},
                {245,90,0},
                {276,60,-60},
//                {286,50,-45},
                {300,30,0},
                {360,60,90},
                {320,80,180},
                {280,60,270},
                {240,15,180},
//                {200,15,180},
                {135,15,180},
                {96,60,90},
                {30,90,180}
        };
        Pose2d[] waypoints = new Pose2d[waypointsRaw.length];
        for (int j = 0; j < waypointsRaw.length; j++) {
                waypoints[j] = new Pose2d(Units.inchesToMeters(waypointsRaw[j][0]), Units.inchesToMeters(waypointsRaw[j][1]), new Rotation2d(Units.degreesToRadians(waypointsRaw[j][2])));
        }
        
        Pose2d startPosition = waypoints[0];

        TrajectoryConfig configA = new TrajectoryConfig(Units.feetToMeters(10), Units.feetToMeters(2));
        configA.setReversed(false);
        //configA.setEndVelocity(configA.getMaxVelocity());
        configA.addConstraint(new DifferentialDriveKinematicsConstraint(driveTrain.getDriveTrainKinematics(), configA.getMaxVelocity()));
        configA.addConstraint(new DifferentialDriveVoltageConstraint(driveTrain.getFeedforward(), driveTrain.getDriveTrainKinematics(),10));
        configA.addConstraint(new CentripetalAccelerationConstraint(1.5));

        int numberOfCones = SimConstants.autoNavSlalomCones.length;
            for(int i = 0; i < numberOfCones; i++) {
            var constraint = new EllipticalRegionConstraint(
                    SimConstants.autoNavSlalomCones[i],
                    Units.inchesToMeters(20),
                    Units.inchesToMeters(20),
                    new Rotation2d(),
                    new DifferentialDriveKinematicsConstraint(driveTrain.getDriveTrainKinematics(), 0)
            );
                //configA.addConstraint(constraint);
        }

        addCommands(new SetDriveShifters(driveTrain, Constants.DriveConstants.inSlowGear),
                new SetOdometry(driveTrain, fieldSim, startPosition),
                new SetDriveNeutralMode(driveTrain, 0));

        for(int i = 0; i < waypoints.length - 1; i++) {

            configA.setEndVelocity(Units.feetToMeters(3));
            if (i != 0) {
                configA.setStartVelocity(Units.feetToMeters(3));
            }

            if (i == waypoints.length - 2) {
                configA.setEndVelocity(0);
            }
            Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints[i],
            List.of(),
            waypoints[i + 1],
            configA);


            var command = TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory);
            addCommands(command);
        }
    }
}

