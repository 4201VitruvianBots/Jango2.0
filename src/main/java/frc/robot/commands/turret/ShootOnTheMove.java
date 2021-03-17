/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.simulation.FieldSim;
import frc.robot.simulation.SimulationShoot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

import java.lang.reflect.Field;


/**
 * An example command that uses an example subsystem.
 */
public class ShootOnTheMove extends CommandBase {

  /*
  Project Doc:
    https://docs.google.com/document/d/1ksk5DAHCi1Ag2KrfKTEo95SJ_jbmWwSPbHlM2ZlTe-o/edit?usp=sharing

  Slides:
    https://docs.google.com/presentation/d/1hh_Wb-yfTFJWq16qWlhEl8ExGFHcJ4ZX2CQzLz0vmVg/edit?usp=sharing 
  */

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Turret m_turret;
    private final Shooter m_shooter;
    private final DriveTrain m_drivetrain;
    private final LED m_led;
    private final Vision m_vision;
    private final SimulationShoot m_simulationShoot;
    private final FieldSim m_fieldSim;
    private final double hexagonCenterCanHitHeight = Constants.outerTargetHeight - (2 * Constants.ballRadius) - (2 * Constants.ballTolerance); // Height of hexagon that center of ball must hit
    private double ledState = 0; // 0 = can't shoot at all, 1 = can only hit outer, 2 = can hit inner
    private double initialHeading; // Current robot position and where it's facing on the field relative to x-axis
    private double currentTurretAngle; // Initial horizontal absolute (relative to field) angle the turret is facing
    private double changeInHeading; // Angle in radians that robot rotates during time to shoot
    private final double timeStep = 0.1;
    private Notifier SOTMNotifier = new Notifier(new SOTMRunnable());

    private double horizontalBallSpeed; // The xy velocity the ball will leave the robot with
    private double targetTurretAngle; // Angle turret needs to rotate to in order for ball to hit target
    private double shooterBallMagnitude; // Magnitude of velocity shooter needs to shoot at in order for ball to hit target
    private double RPM; // Needed RPM to shoot
    private double angleToOuter;
    private double angleToInner; // x-y angles to inner & outer targets
    private double currentDistanceToOuterTargetXY;
    private double currentAngleToOuter;
    private double predictedDistanceToOuterTargetXY; // predicted distance in meters from robot to outer target
    private double predictedDistanceToInnerTargetXY; // predicted distance in meters from robot to inner target on xy-plane (field)ot to outer target on xy-plane (field)
    private double robotLinearVelocity, robotAngularVelocity;
    private boolean hasShot = false;

//  private final double maxHorizontalRatioOuter = (Constants.ballRadius + Constants.ballTolerance) / (2 / Math.sqrt(3)) * hexagonCenterCanHitHeight; // Maximum horizontal for ball to go through outer target
//  private double maxVerticalRatioOuter = hexagonCenterCanHitHeight / 2 / (Constants.ballRadius + Constants.ballTolerance);

    /**
     * Creates a new ExampleCommand.
     *
     * @param turret The subsystem used by this command.
     */

    public ShootOnTheMove(Turret turret, Shooter shooter, DriveTrain drivetrain, LED led, Vision vision, SimulationShoot simulationShoot, FieldSim fieldSim) {
        m_turret = turret;
        m_shooter = shooter;
        m_drivetrain = drivetrain;
        m_led = led;
        m_vision = vision;
        m_simulationShoot = simulationShoot;
        m_fieldSim = fieldSim;

        // Use addRequirements() here to declare subsystem dependencies.
        //addRequirements(turret, shooter, vision);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        SOTMNotifier.startPeriodic(timeStep);
        hasShot = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {


            if(ledState != 0) {
                if (!hasShot) {
                    m_simulationShoot.setVelocity(new Pose2d(new Translation2d(
                                    robotLinearVelocity * Math.cos(initialHeading + changeInHeading) + shooterBallMagnitude * Constants.cosOfVerticalShootAngle * Math.cos(targetTurretAngle),
                                    robotLinearVelocity * Math.sin(initialHeading + changeInHeading) + shooterBallMagnitude * Constants.cosOfVerticalShootAngle * Math.sin(targetTurretAngle)),
                                    new Rotation2d()),
                            shooterBallMagnitude * Constants.sinOfVerticalShootAngle);
                    m_simulationShoot.schedule();
                    hasShot = true;
                }

                m_turret.setFieldCentricSetpoint(Math.toDegrees(targetTurretAngle)); // Setting turret to turn to angle
                m_turret.setControlMode(1); // Enabling turret to turn to setpoint
                m_shooter.setRPM(RPM); // Spin the shooter to shoot the ball
                m_led.setState(ledState == 2 ? 8 : 10); // Green for inner, blue for outer
            } else {
                m_led.setState(2); // Solid red, unable to shoot
            }
            
    }

    private void updateSmartDashboard() {
        SmartDashboardTab.putNumber("Shoot on the Move", "Predicted heading", initialHeading + changeInHeading);
        SmartDashboardTab.putNumber("Shoot on the Move", "Current distance to outer", currentDistanceToOuterTargetXY);
        SmartDashboardTab.putNumber("Shoot on the Move", "Current angle to outer", currentAngleToOuter);
        SmartDashboardTab.putNumber("Shoot on the Move", "robot linear velocity", robotLinearVelocity);
        SmartDashboardTab.putNumber("Shoot on the Move", "robot angular velocity", robotAngularVelocity);

        if(ledState == 2) {
            SmartDashboardTab.putBoolean("Shoot on the Move", "Can shoot", true);
            SmartDashboardTab.putString("Shoot on the Move", "Target", "Inner");
            SmartDashboardTab.putNumber("Shoot on the Move", "xy Distance to inner target", predictedDistanceToInnerTargetXY);
            SmartDashboardTab.putNumber("Shoot on the Move", "xy Angle to inner target", angleToInner);
        } else if(ledState == 1) {
            SmartDashboardTab.putBoolean("Shoot on the Move", "Can shoot", true);
            SmartDashboardTab.putString("Shoot on the Move", "Target", "Outer");
            SmartDashboardTab.putNumber("Shoot on the Move", "xy Distance to outer target", predictedDistanceToOuterTargetXY);
            SmartDashboardTab.putNumber("Shoot on the Move", "xy Angle to outer target", angleToOuter);
        } else {
            SmartDashboardTab.putBoolean("Shoot on the Move", "Can shoot", false);
            SmartDashboardTab.putString("Shoot on the Move", "Target", "None");
        }

        SmartDashboardTab.putNumber("Shoot on the Move", "Shooting at speed", shooterBallMagnitude);
        SmartDashboardTab.putNumber("Shoot on the Move", "Revving shooter to RPM", RPM);
        SmartDashboardTab.putNumber("Shoot on the Move", "Rotating turret to angle", targetTurretAngle);
    }

    private boolean canGoThroughInnerTarget() {
        double tangentOfFinalAngle = Math.abs(2 * Constants.verticalTargetDistance / (predictedDistanceToInnerTargetXY == 0 ? 0.01 : predictedDistanceToInnerTargetXY)
         - shooterBallMagnitude * Constants.sinOfVerticalShootAngle / horizontalBallSpeed);
        return tangentOfFinalAngle <= hexagonCenterCanHitHeight / Constants.targetOffset / 2 && // Vertical angle alone is fine
                tangentOfFinalAngle <= - Math.sqrt(3) * Constants.targetOffset / Math.abs(Math.tan(angleToInner)) + hexagonCenterCanHitHeight; // Within sloped hexagon lines
    }

    private boolean canGoThroughOuterTarget() {
        double verticalTargetIntersection = Math.abs(2 * Constants.verticalTargetDistance / predictedDistanceToOuterTargetXY
         - shooterBallMagnitude * Constants.sinOfVerticalShootAngle / horizontalBallSpeed) * Constants.ballRadius;
        return verticalTargetIntersection <= hexagonCenterCanHitHeight / 2 &&
                verticalTargetIntersection <= - Math.sqrt(3) * Math.abs(Constants.ballRadius / Math.tan(angleToOuter)) + hexagonCenterCanHitHeight;
    }

    /*private double solveCubicEquation(double a, double b, double d) { // Solves a cubic with x-coefficient 0
        double discriminant = Math.sqrt(Math.pow(Math.pow(b, 3) / 27 / Math.pow(a, 3) + d / 2 / a, 2) - Math.pow(b, 6) / 729 / Math.pow(a, 6));
        return Math.cbrt(-Math.pow(b, 3) / 27 / Math.pow(a, 3) - d / 2 / a + discriminant) 
        + Math.cbrt(-Math.pow(b, 3) / 27 / Math.pow(a, 3) - d / 2 / a - discriminant)
        - b / 3 / a;
    }*/
    
    private double solveQuarticEquation(double a, double b, double c, double e) { // Coefficient of X term is 0)
        // Rather than using a formula, makes a guess then uses how far off that guess is to get closer to the x-intercept
        // Looking for a positive root since it's a speed
        double answer = 5, attempts = 0;
        double error, derivative;
        do {
            error = a * Math.pow(answer, 4) + b * Math.pow(answer, 3) + c * Math.pow(answer, 2) + e;
            derivative = 4 * a * Math.pow(answer, 3) + 3 * b * Math.pow(answer, 2) + c * answer;
            answer -= error / derivative;
            attempts++; // Wouldn't want to do this forever if there's no roots
        } while (Math.abs(error / derivative) > 0.1 && attempts < 10);
        return answer;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        SOTMNotifier.stop(); // Stop the periodic calculations
        m_turret.stopTurret(); // Stop turret from rotating
        m_shooter.setRPM(0); // Stopping shooter
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; // Fix this, not sure how long command should run for
    }

    public class SOTMRunnable implements Runnable { // This does all the calculations and can be run with a certain periodicity

        @Override
        public void run() {
            // contains initial straight and angular velocity of robot
            ChassisSpeeds speeds = m_drivetrain.getDriveTrainKinematics().toChassisSpeeds(m_drivetrain.getSpeeds()); // Getting straight and angular velocity of drivetrain

            // Separating into linear and angular components
            // Should be re-calculated based on the maximum amount of time the turret and shooter can take to get to any given position and RPM
            robotLinearVelocity = speeds.vxMetersPerSecond;
            // Linear velocity is meters per second straight ahead, angular velocity is rotation per second calculated from differences between wheels
            robotAngularVelocity = speeds.omegaRadiansPerSecond;

            initialHeading = Math.toRadians(m_drivetrain.getHeading());
            currentTurretAngle = Math.toRadians(m_turret.getTurretAngle()) + initialHeading;
            if (RobotBase.isReal()) {
                currentDistanceToOuterTargetXY = Units.feetToMeters(m_vision.getTargetDistance());
                currentAngleToOuter = Math.toRadians(m_vision.getHorizontalAngleToTarget()) + currentTurretAngle;
            } else {
                currentDistanceToOuterTargetXY = m_fieldSim.getIdealTargetDistance();
                currentAngleToOuter = Math.toRadians(m_fieldSim.getIdealTurretAngle());
            }


            changeInHeading = robotAngularVelocity * timeStep; // Calculating how much robot's heading will change during time to shoot
            double radius = robotLinearVelocity / (robotAngularVelocity == 0 ? 0.01 : robotAngularVelocity); // Calculating distance from robot's position and center of robot's rotation
            double a = 2 * Math.abs(radius * Math.sin(changeInHeading / 2));

            predictedDistanceToOuterTargetXY = Math.sqrt(
                Math.pow(currentDistanceToOuterTargetXY, 2)
                + a * a
                - 2 * currentDistanceToOuterTargetXY * a * Math.cos(initialHeading - currentAngleToOuter + changeInHeading / 2)
            );
            angleToOuter = currentAngleToOuter + (Math.acos(Math.min(
                (Math.pow(currentDistanceToOuterTargetXY, 2) + Math.pow(predictedDistanceToOuterTargetXY, 2) - a * a) / 2 / currentDistanceToOuterTargetXY / predictedDistanceToOuterTargetXY, 
                1))) // Make sure we don't take inverse cosine of 1.000000002
                * (currentAngleToOuter > initialHeading ? 1 : -1); // Have to either add or subtract depending on which way we're going relative to target

            predictedDistanceToInnerTargetXY = Math.sqrt(
                Math.pow(predictedDistanceToOuterTargetXY, 2) + Math.pow(Constants.targetOffset, 2)
                + 2 * predictedDistanceToOuterTargetXY * Constants.targetOffset * Math.sin(angleToOuter)
            );

            angleToInner = angleToOuter - Math.asin(-Constants.targetOffset / predictedDistanceToInnerTargetXY * Math.cos(angleToOuter));

            // Calculations for inner
            horizontalBallSpeed = solveQuarticEquation(
                Math.pow(Constants.verticalTargetDistance / predictedDistanceToInnerTargetXY, 2) - Constants.tanSquaredVerticalShooterAngle,
                2 * Constants.tanSquaredVerticalShooterAngle * robotLinearVelocity * Math.cos(initialHeading + changeInHeading - angleToInner),
                Constants.verticalTargetDistance * Constants.g - robotLinearVelocity * robotLinearVelocity * Constants.tanSquaredVerticalShooterAngle,
                Math.pow(predictedDistanceToInnerTargetXY * Constants.g, 2) / 4
            );
            shooterBallMagnitude = Math.sqrt(
                Math.pow(horizontalBallSpeed, 2) + Math.pow(robotLinearVelocity, 2)
                 - 2 * horizontalBallSpeed * robotLinearVelocity * Math.cos(initialHeading + changeInHeading - angleToInner)
            ) / Constants.cosOfVerticalShootAngle;
            if(canGoThroughInnerTarget()) {
                ledState = 2;
                targetTurretAngle = angleToInner + Math.asin(-robotLinearVelocity * Math.sin(initialHeading + changeInHeading - angleToInner)
                / shooterBallMagnitude / Constants.cosOfVerticalShootAngle);
            } else {
                // Calculations for outer
                horizontalBallSpeed = solveQuarticEquation(
                    Math.pow(Constants.verticalTargetDistance / predictedDistanceToOuterTargetXY, 2) - Constants.tanSquaredVerticalShooterAngle,
                    2 * Constants.tanSquaredVerticalShooterAngle * robotLinearVelocity * Math.cos(initialHeading + changeInHeading - angleToOuter),
                    Constants.verticalTargetDistance * Constants.g - robotLinearVelocity * robotLinearVelocity * Constants.tanSquaredVerticalShooterAngle,
                    Math.pow(predictedDistanceToOuterTargetXY * Constants.g, 2) / 4
                );
                shooterBallMagnitude = Math.sqrt(
                    Math.pow(horizontalBallSpeed, 2) + Math.pow(robotLinearVelocity, 2)
                    - 2 * horizontalBallSpeed * robotLinearVelocity * Math.cos(initialHeading + changeInHeading - angleToOuter)
                ) / Constants.cosOfVerticalShootAngle;
                
                if(canGoThroughOuterTarget()) {
                    ledState = 1;
                    targetTurretAngle = angleToOuter + Math.asin(-robotLinearVelocity * Math.sin(initialHeading + changeInHeading - angleToOuter)
                    / shooterBallMagnitude / Constants.cosOfVerticalShootAngle);
                } else {
                    ledState = 0;
                }
            }
            RPM = shooterBallMagnitude * 60 / (Constants.flywheelDiameter * Math.PI); // Kind of works, probably a better way
            if (RPM > Constants.maxShooterRPM) {
                ledState = 0;
            }
            updateSmartDashboard();
        }
    } 
}

//TODO: make it all work
