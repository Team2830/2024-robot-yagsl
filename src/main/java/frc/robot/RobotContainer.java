// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.IntakeFromShooter;
import frc.robot.commands.IntakeOff;
import frc.robot.commands.IntakeOn;
import frc.robot.commands.IntakeReverse;
import frc.robot.commands.PrepareToShoot;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootAuto;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterAdjuster;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo")); 
  private final Shooter m_Shooter = new Shooter();   
  private final Intake m_Intake = new Intake();
  private final ShooterAdjuster m_ShooterAdjuster = new ShooterAdjuster();                                                                                                                                     

  // Replace with CommandPS4Controller or CommandJoystick if needed
  CommandXboxController driverXbox = new CommandXboxController(0);
  CommandXboxController operatorXbox = new CommandXboxController(1);
  CommandJoystick pretendJoystic = new CommandJoystick(2);


  private final Trigger leftTrigger = new Trigger(() ->
    Math.abs(driverXbox.getLeftTriggerAxis()) > 0.2);

  private final Trigger rightTrigger = new Trigger(() ->
    Math.abs(driverXbox.getRightTriggerAxis()) > 0.2);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    pretendJoystic.button(1).onTrue(getAutonomousCommand());
    //registers Named Commands
    NamedCommands.registerCommand("IntakeOff", new IntakeOff(m_Intake));
    NamedCommands.registerCommand("IntakeOn", new IntakeOn(m_Intake));
    NamedCommands.registerCommand("PrepareToShoot", new PrepareToShoot(m_Shooter));
    NamedCommands.registerCommand("Shoot", new Shoot(m_Shooter));
    NamedCommands.registerCommand("Shoot Auto", new ShootAuto(m_Shooter));

    // Configure the trigger bindings
    configureBindings();

    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                                OperatorConstants.LEFT_Y_DEADBAND),
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                                OperatorConstants.LEFT_X_DEADBAND),
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                                                                                                OperatorConstants.RIGHT_X_DEADBAND),
                                                                   driverXbox.y(),
                                                                   driverXbox.a(),
                                                                   driverXbox.x(),
                                                                   driverXbox.b());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRightX(),
        () -> -driverXbox.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRightX());

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> -MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRawAxis(2));

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);
      // m_Shooter.setDefaultCommand(new IntakeFromShooter(m_Shooter));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    driverXbox.back().onTrue((new InstantCommand(drivebase::zeroGyro)));
    // driverXbox.back().onTrue(new InstantCommand(drivebase::addFakeVisionReading));
 
    //Intake Commands
    operatorXbox.start().whileTrue(new IntakeFromShooter(m_Shooter));
    operatorXbox.rightTrigger().whileTrue(new IntakeOn(m_Intake));
    operatorXbox.leftTrigger().whileTrue(new IntakeReverse(m_Intake));

    //Shooter Commands
    operatorXbox.y().onTrue(new PrepareToShoot(m_Shooter));
    driverXbox.rightTrigger().whileTrue(new Shoot(m_Shooter));
    operatorXbox.x().onTrue(new InstantCommand(() -> {m_ShooterAdjuster.setPosition(Constants.ShooterAngles.UNDER_STAGE);}, m_ShooterAdjuster));
    operatorXbox.povUp().onTrue(new InstantCommand(() -> {m_ShooterAdjuster.setPosition(Constants.ShooterAngles.SUBWOOFER);}, m_ShooterAdjuster));
    operatorXbox.povLeft().onTrue(new InstantCommand(() -> {m_ShooterAdjuster.setPosition(Constants.ShooterAngles.AMP);}, m_ShooterAdjuster));
    operatorXbox.povRight().onTrue(new InstantCommand(() -> {m_ShooterAdjuster.setPosition(Constants.ShooterAngles.PEDESTAL);},m_ShooterAdjuster));
    operatorXbox.b().onTrue(new InstantCommand(() -> {m_ShooterAdjuster.setPosition(Constants.ShooterAngles.AMP_ZONE);},m_ShooterAdjuster));

    //Drive Commands
    // driverXbox.povUp().onTrue((new InstantCommand(()->drivebase.setLastAngleScalar(0))));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    return drivebase.sysIdDriveMotorCommand();
    // An example command will be run in autonomousas
  //  return new PathPlannerAuto("Two Piece Auto");
  // return new PathPlannerAuto("Straight Auto");
  // return new ShootAuto(m_Shooter);
  }
  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
