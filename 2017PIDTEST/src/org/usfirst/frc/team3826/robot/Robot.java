package org.usfirst.frc.team3826.robot;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TalonSRX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.VisionThread;
//                                          2017 Bot Code
/**
 *
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	RobotDrive ourRobot;
	Joystick xBox;
	Relay spike;
	Victor climber;
	TalonSRX sweeper;
	Victor gear;//
	TalonSRX roller;
	CANTalon shooter;

	Encoder leftSide;
	Encoder rightSide;
	
	static final int IMG_WIDTH = 640;
	static final int IMG_HEIGHT = 480;
	VisionThread visionThread;
	double centerX = 0.0;
	double centerRX = 0;
	double centerQX = 0;
	double rCenter = 0;
	double qCenter = 0;
	final Object imgLock = new Object();
	double targetRot;
	
	double setPoint;
	double angleDisplace;
	double rotSpeed;
	double angleNow;
	AHRS ahrs;
	
	int autoCounter;
	double autoSetpoint;

	AnalogInput flagOne;
	AnalogInput flagTwo;
	
	int fOne;
	int fTwo;
	
	boolean open;
	boolean inTransition;
	boolean closed;
	
	private int mode = 1;
	private SendableChooser chooser;
	boolean atSpeed;
	
	double leftPos;
	double rightPos;
	double encDis;
	boolean backing;
	boolean safeMode;
	boolean useCam;
	boolean runAuto;
	boolean init;
	int autoTime;
	Timer autoTimer;
	int autoInt;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    
    public void robotInit() {
    	ourRobot = new RobotDrive(0,1);//Drive Train PWM ports are 0 and 1
    	xBox = new Joystick(0);//Xbox controller has an ID of 0 on the driver station
    	spike = new Relay(1);
    	climber = new Victor(3);
    	sweeper = new TalonSRX(2);//ROLLER AND SWEEPER ARE TALONSRXs ON COMP BOT
    	gear = new Victor(5);
    	roller = new TalonSRX(4);
    	shooter = new CANTalon(0);    	
    	flagOne = new AnalogInput(0);
    	flagTwo = new AnalogInput(1);
    	leftSide = new Encoder(2,3);
    	rightSide = new Encoder(4,5);
    	leftSide.setReverseDirection(true);
    	leftSide.reset();
    	rightSide.setReverseDirection(false);
    	rightSide.reset();
    	autoTimer = new Timer();
    	UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
        camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
        camera.setExposureManual(45);
        camera.setFPS(10);
        
       
        
        visionThread = new VisionThread(camera, new VisionPipelinew(), pipeline -> {
            if (pipeline.filterContoursOutput().size() >= 2) {
                Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
                Rect q = Imgproc.boundingRect(pipeline.filterContoursOutput().get(1));
                synchronized (imgLock) {
                    centerRX = r.x + (r.width / 2);
                    centerQX = q.x + (q.width / 2);
                }
            }
            else if(pipeline.filterContoursOutput().size() == 1){
            	Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
                synchronized (imgLock) {
                    centerRX = r.x + (r.width / 2);
                    centerQX = r.x + (r.width / 2);
                }
            }
        });
    	ahrs = new AHRS(SPI.Port.kMXP);
    	
    	shooter.changeControlMode(TalonControlMode.Speed);
    	shooter.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
    	shooter.reverseOutput(true);
    	shooter.setPID(.85,0.000075,0);
    	

        visionThread.start();
    	
    	chooser = new SendableChooser();
    	chooser.addDefault("No Auto", 1);
    	chooser.addObject("Right Gear DR", 2);
    	chooser.addObject("Left Gear DR", 3);
    	chooser.addObject("Middle Gear", 4);
    	chooser.addObject("Right Gear Safe",5);
    	chooser.addObject("Left Gear Safe",6);
    	chooser.addObject("Right Gear Camera",7);
		chooser.addObject("Left Gear Camera",8);
		chooser.addObject("Right Shooting",9);
		chooser.addObject("Left Shooting",10);
    	SmartDashboard.putData("Auto Modes", chooser);
    	
    }//
    
    /**
     * This function is run once each time the robot enters autonomous mode
     */
    public void autonomousInit() {
    	autoCounter = 0;
    	ahrs.reset();
    	leftSide.reset();
    	rightSide.reset();
    	spike.set(Relay.Value.kForward);
    	mode = (int) chooser.getSelected();
    	autoTime = 0;
    	autoTimer.reset();
    	autoTimer.start();
    	autoInt = 0;
    }

    /**
     * This function is called periodically during autonomous
     */
	public void autonomousPeriodic() {
	
		
		if(Math.abs(centerRX - centerQX) > 50 || (centerRX == centerQX && (centerRX <= 50 || centerRX >= 590))){
			//Targets have to at least 50 pixels apart to be recognized as 2 different targets
			//or they are identified as the same target (R = X) when it is within 50 pixels of the border
			if(centerRX > centerQX){//larger target's x value is always rCenter
				rCenter = centerRX;
				qCenter = centerQX;
			}
			else{
				rCenter = centerQX;
				qCenter = centerRX;
			}
    	}
		
		centerX = (rCenter+qCenter)/2;//centerX is the avg. of the two
		if (centerX <240){//From left border to Center-80 pixels
			targetRot = .1925 + 0.0015 * (320 - centerX);//Positive value for turning (turn right)
		}
		else if (centerX > 400){//From right border to Center+80 pixels
			targetRot = -.1925 + 0.0015 * (320 - centerX);//Negative value for turning (turn left)	
		}
		else{//If within 80 pixels tolerance
			Timer.delay(0.15);//Wait to make sure we are stopped (to avoid false positives)
			if(centerX > 240 && centerX < 400 && autoCounter == 4 && Math.abs(leftSide.getRate()) < 50){//And so that we are in step 5
				targetRot = 0;
			}
			
		}
		
		
	angleNow = ahrs.getAngle();//Defining variables
	angleDisplace = Math.abs(angleNow - autoSetpoint);
        
        if(angleNow > autoSetpoint && angleDisplace > 15){//We are over 15 deg off (to the right)
        	rotSpeed = 0.018925 * (angleDisplace+6);
        }
        else if(angleNow > autoSetpoint && angleDisplace > 5){//Under 15 deg off (to the right)
        	rotSpeed = 0.385 + (angleDisplace/75);
        }
        else if(angleNow < autoSetpoint && angleDisplace > 15){//Over 15 deg off (to the left)
        	rotSpeed = -0.018925 * (angleDisplace+6);
        }
        else if(angleNow < autoSetpoint && angleDisplace > 5){//Under 15 deg off (to the left)
        	rotSpeed = -0.385 - (angleDisplace/75);
        }
        else if(angleDisplace <= 5){//Within 3 deg
        	rotSpeed = 0;
        }
        else{//Backup - not sure if needed
        	rotSpeed = 0;
        }
        
        
        switch(mode) {//Case switch, input number based on value from smart dashboard
        	
        
        	default://No moving
        		
        		System.out.println("Default: Nothing");
        		runAuto = false;
        		ourRobot.arcadeDrive(0,0);
        		mode = 1;
        		break;
        	
        	case 1://No moving
        		
        		System.out.println("Case 1: Nothing");
        		runAuto = false;
        		ourRobot.arcadeDrive(0,0);
        		break;
        		
        	case 2://Right Gear DR
        		
        		System.out.println("Case 2: Right DR");
            	autoSetpoint = -56.25;//Degrees to turn (negative = left)
            	safeMode = false;
            	useCam = false;
            	runAuto = true;
        		break;
        		
        		
        	case 3://Left Gear DR
        		
        		System.out.println("Case 3: Left DR");
            	autoSetpoint = 56;
            	safeMode = false;
            	useCam = false;
            	runAuto = true;     		
            	break;
        		
        	case 4://Middle Gear
        		
        		System.out.println("Case 4: Middle Gear");
        		runAuto = false;
            	if(autoCounter == 0){//Step 1 - Forward
            		if(leftSide.get() < 50){
            			ourRobot.arcadeDrive(-.6,.25*angleNow);
            		}
            		else if(leftSide.get() < 750 && leftSide.getRate() > 10){//leftSide.get() < 1400){
        				ourRobot.arcadeDrive(-.6,.25*angleNow);//Move forward until encoder value is passed
        			}
            		else if(leftSide.getRate() < 10 && leftSide.get() > 50 && leftSide.get() < 750){
        				ourRobot.arcadeDrive(-.7,.25*angleNow);
        			}
            		else if(leftSide.get() < 2125 && leftSide.getRate() > 50){
        				ourRobot.arcadeDrive(-0.45,.25*angleNow);
        			}
        			else if(leftSide.getRate() < 50 && leftSide.get() < 1875 && leftSide.get() > 750){
        				ourRobot.arcadeDrive(-0.675,.25*angleNow);
        			}
        			else if(leftSide.getRate() < 50 && leftSide.get() > 1875 && leftSide.get() < 2025){
        				ourRobot.arcadeDrive(-.6,.65);
        				Timer.delay(0.15);
        				ourRobot.arcadeDrive(-.6,-.65);
        				Timer.delay(0.15);
        			}
        			else{
        				ourRobot.arcadeDrive(0,0);//stop
        				autoCounter = 1;//next step
        			}
        		}
        		else if(autoCounter == 1){//Step 2 - Stop
        			ourRobot.arcadeDrive(0,0);
        			if(leftSide.getRate() < 10){
        				autoCounter = 2;//next step
        			}
        		}
        		else if(autoCounter == 2){
        			if(flagOne.getValue() > 3000){//if door closed
        				gear.set(-.6);//open door
    				}
        			if(flagTwo.getValue() > 2500){//If door open
        				gear.set(0);
        				leftSide.reset();
        				rightSide.reset();
        				autoCounter = 3;//next step
        			}
        		}
        		else if (autoCounter == 3){
        			if(leftSide.get() > -350){
        				ourRobot.arcadeDrive(.6,0);//Move backwards
        			}
        			else{
        				autoCounter = 4;
        			}
        		}
        		else if(autoCounter == 4){//Stop and close doors
        			ourRobot.arcadeDrive(0,0);
        			if(flagTwo.getValue() > 2500){
        				gear.set(.5);
        			}
        			else if(flagOne.getValue() > 3000){
        				gear.set(0);
        				autoCounter = 5;
        			}
        		}
            	
        		break;
        		

        	case 5://Right Gear Safe
        		
        		System.out.println("Case 5: Right Safe");
            	autoSetpoint = -56.25;//Degrees to turn (negative = left)
            	safeMode = true;
            	useCam = false;
            	runAuto = true;
            	break;
        		
        	case 6://Left Gear Safe
		
        		System.out.println("Case 6: Left Safe");
            	autoSetpoint = 56;//Degrees to turn (negative = left)
            	safeMode = true;
            	useCam = false;
            	runAuto = true;
            	break;

        	case 7://Right Gear Camera
		
        		System.out.println("Case 7: Right Camera");
            	autoSetpoint = -56.25;//Degrees to turn (negative = left)
            	safeMode = false;
            	useCam = true;
            	runAuto = true;
            	break;

        	case 8://Left Gear Camera
		
        		System.out.println("Case 8: Left Camera");
            	autoSetpoint = 56;//Degrees to turn (negative = left)
            	safeMode = false;
            	useCam = true;
            	runAuto = true;
            	break;
            	
        	case 9://Right Shooting
        		
        		System.out.println("Case 9: Right Shooting");
        		safeMode = false;
        		useCam = false;
        		runAuto = false;
        		
        		if(shooter.getSpeed() < -18200){
	    			roller.set(-0.5);
	    			sweeper.set(-1);
	    			Timer.delay(.1);//How long roller spins
	    			roller.set(0);
	    			sweeper.set(0);
	    			Timer.delay(.2);//Time between spins
	    		}
        		else{
	    			roller.set(0);
	    			sweeper.set(0);
	    		}
        		
        		if(autoCounter == 0 && mode == 9){
        			if(autoTimer.get() < 9.25){
        	    		shooter.changeControlMode(TalonControlMode.Speed);
        	    		shooter.set(-18275);//Setpoint for PID 18050
        				}
        	    	else{
        	    		shooter.changeControlMode(TalonControlMode.Voltage);//Change mode to voltage in order to stop the shooter
        	    		shooter.set(0);
        	    		autoCounter = 16;
        	    	}
        		}
        		else if(autoCounter == 16){
        			if(angleNow < 57 && (autoInt==0 || autoInt ==1)){
        				ourRobot.arcadeDrive(-.55,-.625);
        				autoInt = 1;
        			}
        			else if(leftSide.get() < 2400 && (autoInt == 1 || autoInt == 2)){
        				ourRobot.arcadeDrive(-.65,0);
        				autoInt = 2;
        			}
        			else if(autoTime<15 && (autoInt == 2 || autoInt == 3)){
        				ourRobot.arcadeDrive(0.4,0);
        				autoTime++;
        				autoInt = 3;
        			}
        			else if(angleNow > 20 && (autoInt == 3 || autoInt == 4)){
        				ourRobot.arcadeDrive(0,.55);
        				autoInt = 4;
        			}
        			else{
        				autoCounter = 20;
        			}
        		}
        		else if(autoCounter == 20){
                	ourRobot.arcadeDrive(0,0);
                }
        	
        	case 10://Left Shooting
        		
        		if(mode == 10){
        			System.out.println("Case 10: Left Shooting");
        		}
        		safeMode = false;
        		useCam = false;
        		runAuto = false;
        		
        		if(shooter.getSpeed() < -18200){
	    			roller.set(-0.5);
	    			sweeper.set(-0.5);
	    			Timer.delay(.1);//How long roller spins
	    			roller.set(0);
	    			sweeper.set(0);
	    			Timer.delay(.2);//Time between spins
	    		}
        		else{
	    			roller.set(0);
	    			sweeper.set(0);
	    		}
        		
        		if(autoCounter == 0 && mode == 10){
        			if(autoTimer.get() < 9.25){
        	    		shooter.changeControlMode(TalonControlMode.Speed);
        	    		shooter.set(-18275);//Setpoint for PID 18050
        				}
        	    	else{
        	    		shooter.changeControlMode(TalonControlMode.Voltage);//Change mode to voltage in order to stop the shooter
        	    		shooter.set(0);
        	    		autoCounter = 15;
        	    	}
        		}
        		else if(autoCounter == 15){
        			if(angleNow > -47.5 && (autoInt==0 || autoInt ==1)){
        				ourRobot.arcadeDrive(-.55,.625);
        				autoInt = 1;
        			}
        			else if(leftSide.get() < 1600 && (autoInt == 1 || autoInt == 2)){
        				ourRobot.arcadeDrive(-.65,0);
        				autoInt = 2;
        			}
        			else if(autoTime<7.5 && (autoInt == 2 || autoInt == 3)){
        				ourRobot.arcadeDrive(0.4,0);
        				autoTime++;
        				autoInt = 3;
        			}
        			else if(angleNow < -35 && (autoInt == 3 || autoInt == 4)){
        				ourRobot.arcadeDrive(0,-.5);
        				autoInt = 4;
        			}
        			else{
        				autoCounter = 20;
        			}
        		}
        		else if(autoCounter == 20){
                	ourRobot.arcadeDrive(0,0);
                }
        }//End of Case Switch
        
        if(mode != 9 && mode != 10){
        	shooter.changeControlMode(TalonControlMode.Voltage);//Change mode to voltage in order to stop the shooter
    		shooter.set(0);
    		roller.set(0);
    		sweeper.set(0);
        }
        
        if(mode != 1 && mode != 4 && mode != 9 && mode != 10){
		if(autoCounter == 0 && runAuto){//Step 1 - Move Forward
        	if(rightSide.get() < 1225){
        		ourRobot.arcadeDrive(-.45,.25*angleNow);
        	}
        	else{
        		ourRobot.arcadeDrive(0,0);//stop
        		autoCounter = 1;//next step
        	}
        }
        else if(autoCounter == 1){//Step 2 - Stop
        	ourRobot.arcadeDrive(0,0);//stop
      		if(leftSide.getRate() < 10){
        			autoCounter = 2;//next step
      		}
        }
        else if(autoCounter == 2){//Step 3 - Turn (Angle based using NavX)
        	if(rotSpeed >= 0.8){//Limits steep to 80%
        		ourRobot.arcadeDrive(0,.8);
        	}
        	else{
        		ourRobot.arcadeDrive(0,rotSpeed);//turn at speed based on angle (See up above for how rotSpeed is calculated)
        	}
        	if(rotSpeed == 0 && Math.abs(rightSide.getRate()) < 50){//leftSide.getRate()) < 50){//If turning speed is supposed to be 0 and encoders say we are slow
        		Timer.delay(0.1);//Wait to make sure we are actually stopped, not just passing our angle setpoint
        		if(rotSpeed == 0 && Math.abs(rightSide.getRate()) < 50){//leftSide.getRate()) < 50){//Same check
        			ourRobot.arcadeDrive(0,0);//stop
        			Timer.delay(.15);
        			leftSide.reset();//Reset encoders
        			rightSide.reset();
        			autoCounter = 3;//next step
        		}
        	} 			
        }
        else if(autoCounter == 3){//Step 4 - Forward (to get in vision range)
        	if (rightSide.get() < 750){//leftSide.get()<750){
        		ourRobot.arcadeDrive(-.5,0.25*(angleNow-autoSetpoint));//Move forward until encoders past setpoint
        	}
        	else{
        		ourRobot.arcadeDrive(0,0);//stop
        		if(leftSide.getRate()<10){
					if(useCam){
        				autoCounter=4;//next step
					}
					else{
						autoCounter = 5;
					}
				}
        	}
        }
        else if(autoCounter == 4){//Step 5 - Vision
        	if(centerX > 240 && centerX < 400){//If target is already centered
        		leftSide.reset();//reset encoders
        		rightSide.reset();
        		autoCounter=5;//next step
        	}
        	else{
        		if(targetRot > .25){//If center is on the left, turn based on distance from center
           			if(targetRot < .345){
           				ourRobot.arcadeDrive(0,.35);
           			}
           			else if(targetRot > .5){
           				ourRobot.arcadeDrive(0,.48);
           			}
           			else{
           				ourRobot.arcadeDrive(0,targetRot);
           			}
           		}
           		else if(targetRot < -.25){//If center is on the right, turn based on distance from center
           			if(targetRot > -.345){
           				ourRobot.arcadeDrive(0,-.35);
           			}
           			else if(targetRot < -.475){
           				ourRobot.arcadeDrive(0,-.48);
           			}
           			else{
           				ourRobot.arcadeDrive(0,targetRot);//See up above for how targetRot is calculated
           			}
           		}
           		else{//If target is now centered,
           			Timer.delay(.25);//wait
           			if(rightSide.getRate()<10){//leftSide.getRate()<10){//make sure bot is stopped
           				ourRobot.arcadeDrive(0,0);//stop
           				Timer.delay(.25);
           				leftSide.reset();//reset encoders
           				rightSide.reset();
           				autoCounter = 5;//next step
           			}
           		}
        	}
        }
        else if(autoCounter == 5){//Step 6 - Forward, Stop + Open Doors
        	if(rightSide.get() < 700){
        		ourRobot.arcadeDrive(-.475,0);//move until past encoders
        		}
        	else{
        		ourRobot.arcadeDrive(0,0);//stop
        		if(!safeMode){
        			if(flagOne.getValue() > 3000 && leftSide.getRate() < 10){//if door closed
            			gear.set(-.7);//open door
            			Timer.delay(0.2);
            			gear.set(-.3);
        			}
					if(flagTwo.getValue() > 2500){
						gear.set(0);
						autoCounter = 6;
					}	
				}
				else if(safeMode && leftSide.getRate() < 10){
					autoCounter = 20;
				}	
			}
        }
        else if(autoCounter == 6){//Step 7 - Delay
        	leftSide.reset();//reset encoders
        	rightSide.reset();
        	autoCounter = 7;//next step
        }
        else if(autoCounter == 7){//Step 8 - Start Reversing
        	ourRobot.arcadeDrive(.5,0.25*(angleNow-autoSetpoint));
        	autoCounter = 8;//next step
       	}
        else if(autoCounter == 8){//Step 9 - Keep Reversing
        	if(rightSide.get() > - 300){
        		ourRobot.arcadeDrive(.5,0.25*(angleNow-autoSetpoint));
        	}
        	else{
        		ourRobot.arcadeDrive(0,0);//stop
        		autoCounter = 9;//next Step
        	}
        }
        else if(autoCounter == 9){//Step 10 - Close Doors
        	if(flagTwo.getValue() > 2500){//If doors open
        		gear.set(.6);//close doors
        	}
        	if(flagOne.getValue() > 3000){
				gear.set(0);
			}
        }
        else if(autoCounter == 20){
        	ourRobot.arcadeDrive(0,0);
        }
        }




        		
        
        
        
        
        
        SmartDashboard.putNumber("Center X",centerX);
    	SmartDashboard.putNumber("Center R",rCenter);
    	SmartDashboard.putNumber("Center Q",qCenter);
    	SmartDashboard.putNumber("Rotation to Target",targetRot);
    	SmartDashboard.putNumber("Angle",angleNow);
    	SmartDashboard.putNumber("Setpoint",setPoint);
    	SmartDashboard.putNumber("Displacement",angleDisplace);
    	SmartDashboard.putNumber("Angle Speed",rotSpeed);
    	SmartDashboard.putNumber("Auto Counter",autoCounter);
    	SmartDashboard.putNumber("FlagOne",fOne);
    	SmartDashboard.putNumber("FlagTwo",fTwo);
    	SmartDashboard.putNumber("Left Side", leftSide.get());
    	SmartDashboard.putNumber("Right Side", rightSide.get());
    	SmartDashboard.putNumber("Left Speed", leftSide.getRate());
    	SmartDashboard.putNumber("Mode",mode);
        
    }

    
    /**
     * This function is called once each time the robot enters teleoperated mode
     */
    @SuppressWarnings("deprecation")
	public void teleopInit(){
    	ahrs.reset();
    	leftSide.reset();
    	rightSide.reset();
    	spike.set(Relay.Value.kForward);//Sets the spike to on in the positive direction
    	closed = false;
    	open = false;
    	inTransition = true;
    	init = true;
    	visionThread.stop();
    }

    /**
     * This function is called periodically during operator control
     */
    /* (non-Javadoc)
     * @see edu.wpi.first.wpilibj.IterativeRobot#teleopPeriodic()
     */
    public void teleopPeriodic() {
    	
    	
    	if(xBox.getRawButton(5) && Math.abs(xBox.getRawAxis(0)) > .75){//If holding LB, go into slow mode (Also attempts to keep you from turning unless joystick is 75% or more to the side), only controls with left stick
    		if(Math.abs(xBox.getRawAxis(1)) > .1){
    			ourRobot.arcadeDrive(.6*xBox.getRawAxis(1), -.6*xBox.getRawAxis(0));
    		}
    		else{
    			ourRobot.arcadeDrive(0, -.6*xBox.getRawAxis(0));
    		}
    		
    	}
    	else if(xBox.getRawButton(5) && Math.abs(xBox.getRawAxis(0)) < .75){
    		if(Math.abs(xBox.getRawAxis(1)) > .1){
    			ourRobot.arcadeDrive(.6*xBox.getRawAxis(1), 0);
    		}
    		else{
    			ourRobot.arcadeDrive(0, 0);
    		}
    	}
    	else if(backing){//Back up a set distance
    		if(encDis < rightSide.get() + 40){
    			ourRobot.arcadeDrive(0.45,0);
    		}
    		else if(encDis > rightSide.get() + 40){
    			ourRobot.arcadeDrive(0,0);
    			backing = false;
    		}
    	}
    	else if(!xBox.getRawButton(5) && Math.abs(xBox.getRawAxis(1)) > .1 || Math.abs(xBox.getRawAxis(0)) > 0.15){//If left stick outside deadzone
    		ourRobot.arcadeDrive(xBox.getRawAxis(1), -.9*xBox.getRawAxis(0));//Control the bot with the left stick
    	}
    	else if(!xBox.getRawButton(5) && Math.abs(xBox.getRawAxis(5)) > .1 || Math.abs(xBox.getRawAxis(4)) > 0.15){//If right stick outside deadzone
    		ourRobot.arcadeDrive(-xBox.getRawAxis(5), -.85*xBox.getRawAxis(4));//Control the bot with the right stick
    	}
    	
    	else{//If no buttons are pressed and the left and right sticks are not being moved
    		ourRobot.arcadeDrive(0,0);//Stop the bot
    	}
    	
    	
    	if(xBox.getRawButton(6) && !xBox.getRawButton(7)){//RB activates sweeper
    		sweeper.set(-.75);
    	}
    	else if(xBox.getRawButton(6) && xBox.getRawButton(7)){
    		sweeper.set(1);
    		roller.set(1);
    	}
    	else if(shooter.getEncVelocity() > -10000){
    		sweeper.set(0);
    	}

    	if(xBox.getRawButton(8)){//Press START to back up set distance
    		encDis = rightSide.get();
    		backing = true;
    	}
    	
    	
    	if(xBox.getRawButton(7)){//SELECT activates climber
    		climber.set(1);
    	}
    	else{
    		climber.set(0);
    	}
    	
    	
    	//Gear Control - Open on B button, Close on X
    	if(inTransition && flagOne.getValue() > 3000 && !open){//CLOSED
    		fOne = 1;
    		fTwo = 0;
    		init = false;
    		gear.set(0);
			closed = true;
			open = false;
			inTransition = false;
    	}
    	else if(inTransition && flagTwo.getValue() > 2500 && !closed){//OPEN
    		fOne = 0;
    		fTwo = 1;
    		init = false;
    		gear.set(0);
    		open = true;
    		closed = false;
    		inTransition = false;
    	}
    	else if(!inTransition){//MOVING
    		if(xBox.getRawButton(2) && closed){
			gear.set(-1);
			Timer.delay(0.175);
			gear.set(-.3);
			closed = false;
			inTransition = true;
		}
    		else if(xBox.getRawButton(3) && open ){
    			gear.set(1);
    			Timer.delay(0.175);
    			gear.set(.3);
    			open = false; 
    			inTransition = true;	
    		}
    		
    	}
    	else if(inTransition && !closed && !open && init){//Failsafe - if neither sensors are triggered at the start of Teleop
    	
    			gear.set(0.5);//Try to close the doors
    		
    	}
    	else{
    		fOne = 0;
    		fTwo = 0;
    	}
    	//If feeder station is < 24" off ground, cut ears
    	//If > 26", back up
    	
    	
    	if(xBox.getRawAxis(2)>.1){//Left Trigger
    		shooter.changeControlMode(TalonControlMode.Speed);
    		shooter.set(-18250);//Setpoint for PID 18050
    	}
    	else{
    		shooter.changeControlMode(TalonControlMode.Voltage);//Change mode to voltage in order to stop the shooter
    		shooter.set(0);
    	}
    	
    	
    		if(shooter.getEncVelocity() < -18175){//Speed we shoot at 17975
    			atSpeed = true;
    		}
    		else{
    			atSpeed = false;
    		}
    	
    if(xBox.getRawAxis(3)>.1){//Right Trigger
    	if(atSpeed){
			roller.set(-0.5);
			sweeper.set(-0.5);
			Timer.delay(.1);//How long roller spins
			roller.set(0);
			sweeper.set(0);
			Timer.delay(.2);//Time between spins
			}
    	else if(!xBox.getRawButton(7)){
    		roller.set(0);
    	}
    }
    else if(!xBox.getRawButton(7) || !xBox.getRawButton(6)){
    	roller.set(0);
    }
    
    	    	
    	
    	//PRINTING
    	SmartDashboard.putNumber("Center X",centerX);
    	SmartDashboard.putNumber("Center R",rCenter);
    	SmartDashboard.putNumber("Center Q",qCenter);
    	SmartDashboard.putNumber("Rotation to Target",targetRot);
    	SmartDashboard.putNumber("Angle",angleNow);
    	SmartDashboard.putNumber("Setpoint",setPoint);
    	SmartDashboard.putNumber("Displacement",angleDisplace);
    	SmartDashboard.putNumber("Angle Speed",rotSpeed);
    	SmartDashboard.putNumber("Auto Counter",autoCounter);
    	SmartDashboard.putNumber("Shooter Speed",shooter.getEncVelocity());
    	SmartDashboard.putNumber("FlagOne",fOne);
    	SmartDashboard.putNumber("FlagTwo",fTwo);
    	SmartDashboard.putNumber("FlagOneAnalog",flagOne.getValue());
    	SmartDashboard.putNumber("FlagTwoAnalog",flagTwo.getValue());
    	SmartDashboard.putNumber("Left Side", leftSide.get());
    	SmartDashboard.putNumber("Right Side", rightSide.get());
    	
    	
    	Timer.delay(.04);// wait 40ms to avoid hogging CPU cycles

    }//
    
    /**
     * This function is called periodically during test mode
     */
    
    public void testPeriodic() {
    
    if(xBox.getRawButton(7)){//'Select-Back-Options' Button
		gear.set(-.25);
	}
    else if(xBox.getRawButton(8)){
    	gear.set(.25);
    }
	else{
		gear.set(0);
	}
    if(flagOne.getValue()>3000){
    	fOne = 1;
    	spike.set(Relay.Value.kForward);
    }
    else{
    	fOne = 0;
    	spike.set(Relay.Value.kOff);
    }
    if(flagTwo.getValue()>2500){
    	fTwo = 1;
    }
    else{
    	fTwo = 0;
    }
    System.out.println("FlagOne"+fOne);
	System.out.println("FlagTwo"+fTwo);
	System.out.println("Flag1A"+flagOne.getValue());
	System.out.println("Flag2A"+flagTwo.getValue());
    
    
    }
       
       
}
