/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.vitruvianlib.utils.XBoxTrigger;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Climber m_climber = new Climber();
  private final ControlPanel m_controlPanel = new ControlPanel();
  private final Controls m_controls = new Controls();
  private final DriveTrain m_driveTrain = new DriveTrain();
  private final Intake m_intake = new Intake();
  private final Shooter m_shooter = new Shooter();
  private final Skyhook m_skyhook = new Skyhook();
  private final Vision m_vision = new Vision();
  private final Indexer m_indexer = new Indexer();

  private Command m_autoCommand;

  static Joystick leftJoystick = new Joystick(Constants.leftJoystick);
  static Joystick rightJoystick = new Joystick(Constants.rightJoystick);
  static Joystick xBoxController = new Joystick(Constants.xBoxController);
  public Button[] leftButtons = new Button[7];
  public Button[] rightButtons = new Button[7];
  public Button[] xBoxButtons = new Button[10];
  public Button[] xBoxPOVButtons = new Button[8];
  public Button xBoxLeftTrigger, xBoxRightTrigger;

  SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_autoChooser.setDefaultOption("Test Path", new TestPathFollowing(m_driveTrain));
    SmartDashboard.putData(m_autoChooser);

    initializeSubsystems();
    // Configure the button bindings
    configureButtonBindings();
    m_driveTrain.setDefaultCommand(new setTankDrive(m_driveTrain));
  }

  public void initializeSubsystems() {
    m_driveTrain.setDefaultCommand(new SetArcadeDrive(m_driveTrain));
    CommandScheduler.getInstance().schedule(new ZeroEncoders(m_driveTrain));
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  public static double getLeftJoystickY() {
    return leftJoystick.getY();
  }
  public static double getRightJoystickY() {
    return rightJoystick.getY();
  }
  private void configureButtonBindings() {
    for (int i = 0; i < leftButtons.length; i++)
      leftButtons[i] = new JoystickButton(leftJoystick, (i + 1));
    for (int i = 0; i < rightButtons.length; i++)
      rightButtons[i] = new JoystickButton(rightJoystick, (i + 1));
    for (int i = 0; i < xBoxButtons.length; i++)
      xBoxButtons[i] = new JoystickButton(xBoxController, (i + 1));
    for (int i = 0; i < xBoxPOVButtons.length; i++)
      xBoxPOVButtons[i] = new POVButton(xBoxController, (i * 45));
    //xBoxPOVButtons[0] = new POVButton(xBoxController, 0);
    //xBoxPOVButtons[1] = new POVButton(xBoxController, 90);

    xBoxLeftTrigger = new XBoxTrigger(xBoxController, 2);
    xBoxRightTrigger = new XBoxTrigger(xBoxController, 3);
  }

  public static double getLeftJoystickX() {
    return -leftJoystick.getX();
  }

  public static double getLeftJoystickY() {
    return leftJoystick.getY();
  }

  public static double getLeftJoystickZ() {
    return -leftJoystick.getZ();
  }

  public static double getLeftRotation(){
    return leftJoystick.getZ();
  }

  public static double getRightJoystickX() {
    return -rightJoystick.getX();
  }

  public static double getRightJoystickY() {
    return rightJoystick.getY();
  }

  public static double getRightJoystickZ() {
    return -rightJoystick.getY();
  }

  public static double getRawLeftJoystickX() {
    return leftJoystick.getX();
  }

  public static double getRawLeftJoystickY() {
    return -leftJoystick.getY();
  }

  public static double getRawRightJoystickX() {
    return rightJoystick.getX();
  }

  public static double getRawRightJoystickY() {
    return -rightJoystick.getY();
  }

  public static double getXBoxLeftX(){
    return xBoxController.getRawAxis(0);
  }

  public static double getXBoxLeftY(){
    return -xBoxController.getRawAxis(1);
  }

  public static double getXBoxRightX(){
    return xBoxController.getRawAxis(4);
  }

  public static double getXBoxRightY(){
    return -xBoxController.getRawAxis(5);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    if(m_autoChooser.getSelected() != null)
      m_autoCommand = m_autoChooser.getSelected();

    return m_autoCommand.andThen(() -> m_driveTrain.setVoltageOutput(0, 0));
  }
}
