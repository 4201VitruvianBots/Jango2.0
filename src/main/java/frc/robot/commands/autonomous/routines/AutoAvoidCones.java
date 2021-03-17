/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous.routines;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * An example command that uses an example subsystem.
 */
public class AutoAvoidCones extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveTrain m_driveTrain;
    private final DoubleSupplier m_leftOutput, m_rightOutput;
    private double currentLeftOutput, currentRightOutput;
    private final Translation2d[] m_conePoses;
    private final BooleanSupplier m_override;

    private final double tolerance = Units.inchesToMeters(6);
    private final double robotLength = 0.838 + tolerance;
    private final double robotWidth = 0.673 + tolerance;

    // Angle from center of robot to a corner
    private final double criticalAngle = Math.atan2(robotWidth, robotLength);

    private final double coneRadius = Units.inchesToMeters(6);
    /* Robot Points:
           ^: Front of the robot
             0-------1
             |   ^   |
             3-------2
         */
    Translation2d[] robotCornerPoses = new Translation2d[4];

    public AutoAvoidCones(DriveTrain driveTrain, DoubleSupplier leftOutput, DoubleSupplier rightOutput, BooleanSupplier override, Translation2d[] conePoses) {
        m_driveTrain = driveTrain;
        m_leftOutput = leftOutput;
        m_rightOutput = rightOutput;
        m_conePoses = conePoses;
        m_override = override;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        currentLeftOutput = m_leftOutput.getAsDouble();
        currentRightOutput = m_rightOutput.getAsDouble();
        if (aboutToHitCone() && !m_override.getAsBoolean()) {
            m_driveTrain.setMotorTankDrive(0, 0);
        } else {
            m_driveTrain.setMotorTankDrive(currentLeftOutput, currentRightOutput);
        }
    }

    private boolean aboutToHitCone() {
        Pose2d robotPose = m_driveTrain.getRobotPose();
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double cos = robotPose.getRotation().getCos();
        double sin = robotPose.getRotation().getSin();

        Translation2d[] robotCornerPoses = {
            new Translation2d(robotLength / 2.0, robotLength / 2.0),
            new Translation2d(robotLength / 2.0, -robotLength / 2.0),
            new Translation2d(-robotLength / 2.0, -robotLength / 2.0),
            new Translation2d(-robotLength / 2.0, robotLength / 2.0)
        };
        for (int i = 0; i < 4; i++) {
            Translation2d updatedPosition = robotCornerPoses[i].rotateBy(robotPose.getRotation()).plus(robotPose.getTranslation());
        }
        double deltaXa = robotLength / 2.0;
        double deltaXb = - robotLength / 2.0;
        double deltaYa = robotWidth / 2.0;
        double deltaYb = - robotWidth / 2.0;

        robotCornerPoses[0] = new Translation2d(cos * deltaXa - sin * deltaYa + robotX,
                sin * deltaXa + cos * deltaYa + robotY);
        robotCornerPoses[1] = new Translation2d(cos * deltaXa - sin * deltaYb + robotX,
                sin * deltaXa + cos * deltaYb + robotY);
        robotCornerPoses[2] = new Translation2d(cos * deltaXb - sin * deltaYb + robotX,
                sin * deltaXb + cos * deltaYb + robotY);
        robotCornerPoses[3] = new Translation2d(cos * deltaXb - sin * deltaYa + robotX,
                sin * deltaXb + cos * deltaYa + robotY);
        SmartDashboardTab.putNumber("DriveTrain", "Corner pose 0", Units.metersToFeet(robotCornerPoses[0].getX()));
        for (Translation2d cone : m_conePoses) { 
            double angleToCone = Math.atan2(cone.getY() - robotPose.getY(), cone.getX() - robotPose.getX());
            Translation2d adjustedConePose = new Translation2d(
                cone.getX() - coneRadius * Math.cos(angleToCone),
                cone.getY() - coneRadius * Math.sin(angleToCone));

            double slope0to1 = (robotCornerPoses[1].getY() - robotCornerPoses[0].getY()) /(robotCornerPoses[1].getX() - robotCornerPoses[0].getX());

            double slope1to2 = (robotCornerPoses[2].getY() - robotCornerPoses[1].getY()) /(robotCornerPoses[2].getX() - robotCornerPoses[1].getX());

            if ((
                    (adjustedConePose.getY() >= slope0to1 * (adjustedConePose.getX() - robotCornerPoses[0].getX()) + robotCornerPoses[0].getY()) ==
                            (adjustedConePose.getY() <= slope0to1 * (adjustedConePose.getX() - robotCornerPoses[2].getX()) + robotCornerPoses[2].getY())
            ) && (
                    (adjustedConePose.getY() >= slope1to2 * (adjustedConePose.getX() - robotCornerPoses[0].getX()) + robotCornerPoses[0].getY()) ==
                            (adjustedConePose.getY() <= slope1to2 * (adjustedConePose.getX() - robotCornerPoses[1].getX()) + robotCornerPoses[1].getY())
            )) {
                    SmartDashboardTab.putBoolean("DriveTrain", "Will hit Cone", true);
                    return true;
                }
        }
        SmartDashboardTab.putBoolean("DriveTrain", "Will hit Cone", false);
        return false;
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
