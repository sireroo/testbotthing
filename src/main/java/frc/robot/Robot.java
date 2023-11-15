// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private static final String kTheAuto = "The Auto";
  private static final String kSEncoderAuto = "Seamus Encoder Auto";
  private static final String kSPIDAuto = "Seamus PID Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final CANSparkMax frontright_motor = new CANSparkMax(1,MotorType.kBrushless);
  private final CANSparkMax backright_motor = new CANSparkMax(2,MotorType.kBrushless);
  private final CANSparkMax frontleft_motor = new CANSparkMax(3,MotorType.kBrushless);
  private final CANSparkMax backleft_motor = new CANSparkMax(4,MotorType.kBrushless);

  private final RelativeEncoder backleft_encoder = (backleft_motor.getEncoder());

  double forwardPower;

  private final MotorControllerGroup left_motors = new MotorControllerGroup(backleft_motor, frontleft_motor);
  private final MotorControllerGroup right_motors = new MotorControllerGroup(backright_motor, frontright_motor);
  private final DifferentialDrive robot = new DifferentialDrive(left_motors, right_motors);
  private XboxController controller = new XboxController(0);
  private Timer timer = new Timer();

  private final PIDController distanceController = new PIDController(3.5, 0.005, 0.001);


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    m_chooser.addOption("The Auto", kTheAuto);
    m_chooser.addOption("Seamus Encoder Auto", kSEncoderAuto);
    m_chooser.addOption("Seamus PID Auto", kSPIDAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    right_motors.setInverted(true);
  }


  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    timer.reset();
    timer.start();
    resetEncoder();
  }  

  private void resetEncoder() {
    backleft_encoder.setPosition(0);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    SmartDashboard.putNumber("Back Left Encoder", backleft_encoder.getPosition());
    SmartDashboard.putNumber("Forward Power", forwardPower);

    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kTheAuto:
        // Put the auto code here
        if(timer.get() < 0.5) {
          robot.tankDrive(0.5, 0.5);
        }else if (timer.get() >= 0.5 && timer.get() < 1.5) {
          robot.tankDrive(-0.5, -0.5);
        }else{
          robot.tankDrive(0.0, 0);
        }

        break;
      case kSEncoderAuto:
        
      if(backleft_encoder.getPosition()<10) {
        robot.tankDrive(0.5, 0.5);
      }
      else {
        robot.tankDrive(0, 0);
      }
      break;

      case kSPIDAuto:

      distanceController.setSetpoint(5);

      forwardPower = distanceController.calculate(backleft_encoder.getPosition()/2);

      if (forwardPower > 0.5){
        forwardPower = 0.5;
      } else if (forwardPower < -0.5){
        forwardPower = -0.5;
      }

      if (distanceController.getSetpoint() != 0) {
        robot.tankDrive(forwardPower, forwardPower);
    }
      
      break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    robot.tankDrive(controller.getLeftY()/2, controller.getRightY()/2);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
