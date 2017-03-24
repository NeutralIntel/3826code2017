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
	double autoSetPoint;
	double angleDisplace;
	double rotSpeed;
	double angleNow;
	AHRS ahrs;
	
	int autoCounter;
	int autoSetpoint;

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
	boolean farEnough;
	boolean reset;
	Timer gearTime;
	Timer autoTurn;
	int speedStep;
	

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
			targetRot = .19 + 0.0015 * (320 - centerX);//Positive value for turning (turn right)
		}
		else if (centerX > 400){//From right border to Center+80 pixels
			targetRot = -.19 + 0.0015 * (320 - centerX);//Negative value for turning (turn left)	
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
        	rotSpeed = 0.01885 * (angleDisplace+5);
        }
        else if(angleNow > autoSetpoint && angleDisplace > 5){//Under 15 deg off (to the right)
        	rotSpeed = 0.385 + (angleDisplace/75);
        }
        else if(angleNow < autoSetpoint && angleDisplace > 15){//Over 15 deg off (to the left)
        	rotSpeed = -0.01885 * (angleDisplace+5);
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
        		
        		ourRobot.arcadeDrive(0,0);
        		System.out.println("default");
        		break;
        	
        	case 1://No moving
        		
        		ourRobot.arcadeDrive(0,0);
        		System.out.println("case 1");
        		break;
        		
        	case 2://Right Gear
        		
        		System.out.println("case 2");
            	autoSetpoint = -56;//Degrees to turn (negative = left)
        		
            	if(autoCounter == 0){//Step 1 - Forward
        			if(rightSide.get() < 1500){//leftSide.get() < 1400){
        				ourRobot.arcadeDrive(-.45,.25*angleNow);//Move forward until encoder value is passed
        			}
        			else{
        				ourRobot.arcadeDrive(0,0);//stop
        				autoCounter = 1;//next step
        			}
        		}
        		else if(autoCounter == 1){//Step 2 - Stop
        			ourRobot.arcadeDrive(0,0);//stop
        			Timer.delay(0.475);//wait (to stop coasting)
        			autoCounter = 2;//next step
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
        		//Right Gear
        		else if(autoCounter == 3){//Step 4 - Forward (to get in vision range)
        			if (rightSide.get() < 750){//leftSide.get()<750){
        				ourRobot.arcadeDrive(-.5,0);//Move forward until encoders past setpoint
        			}
        			else{
        				ourRobot.arcadeDrive(0,0);//stop
        				Timer.delay(0.75);
        				autoCounter=4;//next step
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
            		else if(targetRot < -.25){//If center is on the rightt, turn based on distance from center
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
        		//Right Gear
        		else if(autoCounter == 5){//Step 6 - Forward, Stop + Open Doors
        			if(rightSide.get() < 700){//leftSide.get() < 400){
        				ourRobot.arcadeDrive(-.475,0);//move until past encoders
        				if(leftSide.get() > 250 && leftSide.getRate() < 0){//If we are prematurely stopped (hitting the peg)
        					ourRobot.arcadeDrive(0,0);//stop
        					if(flagOne.getValue() > 3000){//if door closed
                				gear.set(-1);//open door
                				Timer.delay(0.175);
                				gear.set(-.4);
            				}
            				Timer.delay(0.1);
            				autoCounter = 6;//next step
        				}
        			}
        			else{//if encoders pass 425
        				ourRobot.arcadeDrive(0,0);//stop
        				Timer.delay(0.1);
        				if(flagOne.getValue() > 3000){//if door closed
            				gear.set(-.7);//open door
            				Timer.delay(0.2);
            				
            				
            				gear.set(-.3);
        				}
        				Timer.delay(0.05);
        				autoCounter = 6;//next step
        			}
        		}
        		else if(autoCounter == 6){//Step 7 - Delay
        			leftSide.reset();//reset encoders
            		rightSide.reset();
        			Timer.delay(.05);//wait
        			autoCounter = 7;//next step
        		}
        		else if(autoCounter == 7){//Step 8 - Start Reversing, Stop Door
        			if(flagTwo.getValue() > 2500){//if doors open
                		gear.set(0);//stop gear motor
                		ourRobot.arcadeDrive(.5,0);//keep reversing
                		autoCounter = 8;//next step
                	}
        		}
        		//Right Gear
        		else if(autoCounter == 8){//Step 9 - Keep Reversing
        			if(rightSide.get() > - 300){//leftSide.get() > - 300){//keep moving while encoders > -1000
        				ourRobot.arcadeDrive(.5,0);
        			}
        			else{
        				ourRobot.arcadeDrive(0,0);//stop
        				Timer.delay(.1);//wait
        				autoCounter = 9;//next Step
        			}
        		}
        		else if(autoCounter == 9){//Step 10 - Close Doors
        			if(flagTwo.getValue() > 2500){//If doors open
        				gear.set(1);//close doors
        				Timer.delay(0.175);
        				gear.set(.4);
        				autoCounter = 10;//next Step
        			}
        		}
        		else if(autoCounter == 10){//Step 11 - Stop Doors
        			if(flagOne.getValue() > 3000){//if doors closed
        				gear.set(0);//stop gear motor
        			}
        		}
        		else if(autoCounter == 20){
        			ourRobot.arcadeDrive(0,0);
        		}
        		break;
        		//Right Gear
        		
        		
        	case 3://Left Gear
        		
            	autoSetpoint = 56;
            	if(autoCounter == 0){//Step 1 - Forward
        			if(rightSide.get() < 1500){//leftSide.get() < 1400){
        				ourRobot.arcadeDrive(-.45,.25*angleNow);//Move forward until encoder value is passed
        			}
        			else{
        				ourRobot.arcadeDrive(0,0);//stop
        				autoCounter = 1;//next step
        			}
        		}
        		else if(autoCounter == 1){//Step 2 - Stop
        			ourRobot.arcadeDrive(0,0);//stop
        			Timer.delay(0.475);//wait (to stop coasting)
        			autoCounter = 2;//next step
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
        		//Left Gear
        		else if(autoCounter == 3){//Step 4 - Forward (to get in vision range)
        			if (rightSide.get() < 650){//leftSide.get()<750){
        				ourRobot.arcadeDrive(-.5,0);//Move forward until encoders past setpoint
        			}
        			else{
        				ourRobot.arcadeDrive(0,0);//stop
        				Timer.delay(0.75);
        				autoCounter=4;//next step
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
            		else if(targetRot < -.25){//If center is on the rightt, turn based on distance from center
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
        		//Left Gear
        		else if(autoCounter == 5){//Step 6 - Forward, Stop + Open Doors
        			if(rightSide.get() < 650){//leftSide.get() < 400){
        				ourRobot.arcadeDrive(-.475,0);//move until past encoders
        				if(leftSide.get() > 250 && leftSide.getRate() < 0){//If we are prematurely stopped (hitting the peg)
        					ourRobot.arcadeDrive(0,0);//stop
        					if(flagOne.getValue() > 3000){//if door closed
                				gear.set(-1);//open door
                				Timer.delay(0.175);
                				gear.set(-.4);
            				}
            				Timer.delay(0.1);
            				autoCounter = 6;//next step
        				}
        			}
        			else{//if encoders pass 425
        				ourRobot.arcadeDrive(0,0);//stop
        				Timer.delay(0.1);
        				if(flagOne.getValue() > 3000){//if door closed
        					gear.set(-.7);//open door
            				Timer.delay(0.1);
            				gear.set(-.3);
        				}
        				Timer.delay(0.05);
        				autoCounter = 6;//next step
        			}
        		}
        		else if(autoCounter == 6){//Step 7 - Delay
        			leftSide.reset();//reset encoders
            		rightSide.reset();
        			Timer.delay(.05);//wait
        			autoCounter = 7;//next step
        		}
        		else if(autoCounter == 7){//Step 8 - Start Reversing, Stop Door
        			if(flagTwo.getValue() > 2500){//if doors open
                		gear.set(0);//stop gear motor
                		ourRobot.arcadeDrive(.5,0);//keep reversing
                		autoCounter = 8;//next step
                	}
        		}
        		//Right Gear
        		else if(autoCounter == 8){//Step 9 - Keep Reversing
        			if(rightSide.get() > - 300){//leftSide.get() > - 300){//keep moving while encoders > -1000
        				ourRobot.arcadeDrive(.5,0);
        			}
        			else{
        				ourRobot.arcadeDrive(0,0);//stop
        				Timer.delay(.1);//wait
        				autoCounter = 9;//next Step
        			}
        		}
        		else if(autoCounter == 9){//Step 10 - Close Doors
        			if(flagTwo.getValue() > 2500){//If doors open
        				gear.set(1);//close doors
        				Timer.delay(0.175);
        				gear.set(.4);
        				autoCounter = 10;//next Step
        			}
        		}
        		else if(autoCounter == 10){//Step 11 - Stop Doors
        			if(flagOne.getValue() > 3000){//if doors closed
        				gear.set(0);//stop gear motor
        			}
        		}
        		else if(autoCounter == 20){
        			ourRobot.arcadeDrive(0,0);
        		}
        		break;
        		
        	case 4://Middle Gear
        		

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
            		else if(leftSide.get() < 2100 && leftSide.getRate() > 50){
        				ourRobot.arcadeDrive(-0.45,.25*angleNow);
        			}
        			else if(leftSide.getRate() < 50 && leftSide.get() < 1900 && leftSide.get() > 750){
        				ourRobot.arcadeDrive(-0.675,.25*angleNow);
        			}
        			else if(leftSide.getRate() < 50 && leftSide.get() > 1900 && leftSide.get() < 2000){
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
        				gear.set(-1);//open door
        				Timer.delay(0.175);
        				gear.set(-.4);
    				}
        			else if(flagTwo.getValue() > 2500){
        				gear.set(0);
        				leftSide.reset();
        				rightSide.reset();
        				autoCounter = 3;//next step
        			}
        		}
        		else if (autoCounter == 3){
        			if(leftSide.get() > -350){
        				ourRobot.arcadeDrive(.6,.25*angleNow);
        			}
        			else{
        				autoCounter = 4;
        			}
        		}
        		else if(autoCounter == 4){
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
        		

        	case 5://Right Gear
        		
        		System.out.println("case 5");
            	autoSetpoint = -56;//Degrees to turn (negative = left)
        		
            	if(autoCounter == 0){//Step 1 - Forward
        			if(rightSide.get() < 1500){//leftSide.get() < 1400){
        				ourRobot.arcadeDrive(-.45,.25*angleNow);//Move forward until encoder value is passed
        			}
        			else{
        				ourRobot.arcadeDrive(0,0);//stop
        				autoCounter = 1;//next step
        			}
        		}
        		else if(autoCounter == 1){//Step 2 - Stop
        			ourRobot.arcadeDrive(0,0);//stop
        			Timer.delay(0.475);//wait (to stop coasting)
        			autoCounter = 2;//next step
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
        		//Right Gear
        		else if(autoCounter == 3){//Step 4 - Forward (to get in vision range)
        			if (rightSide.get() < 750){//leftSide.get()<750){
        				ourRobot.arcadeDrive(-.5,0.05*(angleNow-autoSetpoint));//Move forward until encoders past setpoint
        			}
        			else{
        				ourRobot.arcadeDrive(0,0);//stop
        				Timer.delay(0.75);
        				autoCounter=5;//next step
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
            		else if(targetRot < -.25){//If center is on the rightt, turn based on distance from center
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
        		//Right Gear
        		else if(autoCounter == 5){//Step 6 - Forward, Stop + Open Doors
        			if(rightSide.get() < 950){//leftSide.get() < 400){
        				ourRobot.arcadeDrive(-.6,0);//move until past encoders
        				if(rightSide.get() > 250 && rightSide.getRate() < 0){//If we are prematurely stopped (hitting the peg)
        					ourRobot.arcadeDrive(0,0);//stop
        					//if(flagOne.getValue() > 3000){//if door closed
                				//gear.set(-1);//open door
                				//Timer.delay(0.175);
                				//gear.set(-.4);
            				//}
            				Timer.delay(0.1);
            				autoCounter = 6;//next step
        				}
        			}
        			else{//if encoders pass 425
        				ourRobot.arcadeDrive(0,0);//stop
        				Timer.delay(0.1);
        				//if(flagOne.getValue() > 3000){//if door closed
            				//gear.set(-1);//open door
            				//Timer.delay(0.1);
            				//gear.set(-.5);
            				//ourRobot.arcadeDrive(.45,0);
            				//Timer.delay(0.1);
            				//gear.set(-.3);
        				//}
        				Timer.delay(0.05);
        				autoCounter = 6;//next step
        			}
        		}
        		else if(autoCounter == 6){//Step 7 - Delay
        			leftSide.reset();//reset encoders
            		rightSide.reset();
        			Timer.delay(.05);//wait
        			autoCounter = 20;//next step
        		}
        		else if(autoCounter == 7){//Step 8 - Start Reversing, Stop Door
        			if(flagTwo.getValue() > 2500){//if doors open
                		gear.set(0);//stop gear motor
                		ourRobot.arcadeDrive(.5,00);//keep reversing
                		autoCounter = 8;//next step
                	}
        		}
        		//Right Gear
        		else if(autoCounter == 8){//Step 9 - Keep Reversing
        			if(rightSide.get() > - 300){//leftSide.get() > - 300){//keep moving while encoders > -1000
        				ourRobot.arcadeDrive(.5,00);
        			}
        			else{
        				ourRobot.arcadeDrive(0,0);//stop
        				Timer.delay(.1);//wait
        				autoCounter = 9;//next Step
        			}
        		}
        		else if(autoCounter == 9){//Step 10 - Close Doors
        			if(flagTwo.getValue() > 2500){//If doors open
        				gear.set(1);//close doors
        				Timer.delay(0.175);
        				gear.set(.4);
        				autoCounter = 10;//next Step
        			}
        		}
        		else if(autoCounter == 10){//Step 11 - Stop Doors
        			if(flagOne.getValue() > 3000){//if doors closed
        				gear.set(0);//stop gear motor
        			}
        		}
        		else if(autoCounter == 20){
        			ourRobot.arcadeDrive(0,0);
        		}
        		break;
        		//Right Gear
        		
        	case 6:
        		System.out.println("case 6");
            	autoSetpoint = 56;//Degrees to turn (negative = left)
        		
            	if(autoCounter == 0){//Step 1 - Forward
        			if(rightSide.get() < 1500){//leftSide.get() < 1400){
        				ourRobot.arcadeDrive(-.45,.25*angleNow);//Move forward until encoder value is passed
        			}
        			else{
        				ourRobot.arcadeDrive(0,0);//stop
        				autoCounter = 1;//next step
        			}
        		}
        		else if(autoCounter == 1){//Step 2 - Stop
        			ourRobot.arcadeDrive(0,0);//stop
        			Timer.delay(0.475);//wait (to stop coasting)
        			autoCounter = 2;//next step
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
        		//Right Gear
        		else if(autoCounter == 3){//Step 4 - Forward (to get in vision range)
        			if (rightSide.get() < 750){//leftSide.get()<750){
        				ourRobot.arcadeDrive(-.5,00);//Move forward until encoders past setpoint
        			}
        			else{
        				ourRobot.arcadeDrive(0,0);//stop
        				Timer.delay(0.75);
        				autoCounter=5;//next step
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
            		else if(targetRot < -.25){//If center is on the rightt, turn based on distance from center
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
        		//Right Gear
        		else if(autoCounter == 5){//Step 6 - Forward, Stop + Open Doors
        			if(rightSide.get() < 950){//leftSide.get() < 400){
        				ourRobot.arcadeDrive(-.6,00);//move until past encoders
        				if(rightSide.get() > 250 && rightSide.getRate() < 0){//If we are prematurely stopped (hitting the peg)
        					ourRobot.arcadeDrive(0,0);//stop
        					//if(flagOne.getValue() > 3000){//if door closed
                				//gear.set(-1);//open door
                				//Timer.delay(0.175);
                				//gear.set(-.4);
            				//}
            				Timer.delay(0.1);
            				autoCounter = 6;//next step
        				}
        			}
        			else{//if encoders pass 425
        				ourRobot.arcadeDrive(0,0);//stop
        				Timer.delay(0.1);
        				//if(flagOne.getValue() > 3000){//if door closed
            				//gear.set(-1);//open door
            				//Timer.delay(0.1);
            				//gear.set(-.5);
            				//ourRobot.arcadeDrive(.45,0);
            				//Timer.delay(0.1);
            				//gear.set(-.3);
        				//}
        				Timer.delay(0.05);
        				autoCounter = 6;//next step
        			}
        		}
        		else if(autoCounter == 6){//Step 7 - Delay
        			leftSide.reset();//reset encoders
            		rightSide.reset();
        			Timer.delay(.05);//wait
        			autoCounter = 20;//next step
        		}
        		else if(autoCounter == 7){//Step 8 - Start Reversing, Stop Door
        			if(flagTwo.getValue() > 2500){//if doors open
                		gear.set(0);//stop gear motor
                		ourRobot.arcadeDrive(.5,0);//keep reversing
                		autoCounter = 8;//next step
                	}
        		}
        		//Right Gear
        		else if(autoCounter == 8){//Step 9 - Keep Reversing
        			if(rightSide.get() > - 300){//leftSide.get() > - 300){//keep moving while encoders > -1000
        				ourRobot.arcadeDrive(.5,0);
        			}
        			else{
        				ourRobot.arcadeDrive(0,0);//stop
        				Timer.delay(.1);//wait
        				autoCounter = 9;//next Step
        			}
        		}
        		else if(autoCounter == 9){//Step 10 - Close Doors
        			if(flagTwo.getValue() > 2500){//If doors open
        				gear.set(1);//close doors
        				Timer.delay(0.175);
        				gear.set(.4);
        				autoCounter = 10;//next Step
        			}
        		}
        		else if(autoCounter == 10){//Step 11 - Stop Doors
        			if(flagOne.getValue() > 3000){//if doors closed
        				gear.set(0);//stop gear motor
        			}
        		}
        		else if(autoCounter == 20){
        			ourRobot.arcadeDrive(0,0);
        		}
        		break;
        		
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
    	farEnough = false;
    	visionThread.stop();
    }

    /**
     * This function is called periodically during operator control
     */
    /* (non-Javadoc)
     * @see edu.wpi.first.wpilibj.IterativeRobot#teleopPeriodic()
     */
    public void teleopPeriodic() {
    	
    	spike.set(Relay.Value.kForward);
    	
    	if(xBox.getRawButton(5) && Math.abs(xBox.getRawAxis(0)) > .75){
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
    	else if(backing && reset){
    		if(encDis < rightSide.get() + 40){
    			ourRobot.arcadeDrive(0.45,0);
    		}
    		else if(encDis > rightSide.get() + 40){
    			ourRobot.arcadeDrive(0,0);
    			backing = false;
    			reset = false;
    		}
    	}
    	else if(!xBox.getRawButton(5) && Math.abs(xBox.getRawAxis(1)) > .1 || Math.abs(xBox.getRawAxis(0)) > 0.15){//If stick outside deadzone
    		ourRobot.arcadeDrive(xBox.getRawAxis(1), -.85*xBox.getRawAxis(0));//Control the bot with the left stick
    	}
    	else if(!xBox.getRawButton(5) && Math.abs(xBox.getRawAxis(5)) > .1 || Math.abs(xBox.getRawAxis(4)) > 0.15){//If stick outside deadzone
    		ourRobot.arcadeDrive(-xBox.getRawAxis(5), -.85*xBox.getRawAxis(4));//Control the bot with the right stick
    	}
    	
    	else{//If no buttons are pressed and the left and right sticks are not being moved
    		ourRobot.arcadeDrive(0,0);//Stop the bot
    	}
    	
    	
    	if(xBox.getRawButton(6)){
    		sweeper.set(-.75);
    	}
    	else if(shooter.getEncVelocity() > -15000){
    		sweeper.set(0);
    	}

    	if(xBox.getRawButton(8)){
    		encDis = rightSide.get();
    		backing = true;
    		reset = true;
    	}
    	
    	
    	if(xBox.getRawButton(7)){
    		climber.set(1);
    	}
    	else{
    		climber.set(0);
    	}
    	
    	
    	//Gear Control - Open on B button, Close on X
    	if(inTransition && flagOne.getValue() > 3000 && !open){//CLOSED
    		fOne = 1;
    		fTwo = 0;
			gear.set(0);
			closed = true;
			open = false;
			farEnough = false;
			inTransition = false;
    	}
    	else if(inTransition && flagTwo.getValue() > 2500 && !closed){//OPEN
    		fOne = 0;
    		fTwo = 1;
			gear.set(0);
			farEnough=false;
			leftPos=leftSide.get();
			rightPos=rightSide.get();
			open = true;
			closed = false;
			inTransition = false;
    	}
    	else if(!inTransition){//MOVING
    		/*if(leftSide.get() - leftPos <= -300 || rightSide.get() - rightPos <= -300){
    			farEnough = true;
    		}
    		else if(!farEnough && xBox.getRawButton(3) && leftSide.get() - leftPos >= 600){
    			farEnough = true;
    		}*/
    		if(xBox.getRawButton(2) && closed){
				gear.set(-1);
				Timer.delay(0.175);
				gear.set(-.3);
				closed = false;
				inTransition = true;
			}
    		else if(xBox.getRawButton(3) && open ){//&& farEnough){
    			gear.set(1);
    			Timer.delay(0.175);
    			gear.set(.3);
    			open = false; 
    			inTransition = true;	
    		}
    		
    	}
    	else{
    		fOne = 0;
    		fTwo = 0;
    	}
    	
    	
    	if(xBox.getRawAxis(2)>.1){//Left Trigger
    		shooter.changeControlMode(TalonControlMode.Speed);
    		shooter.set(-18850);//Setpoint for PID
    		//-18850
    	}
    	else{
    		shooter.changeControlMode(TalonControlMode.Voltage);
    		shooter.set(0);
    	}
    	
    	
    		if(shooter.getEncVelocity() < -18500){//Speed we shoot at
    			atSpeed = true;
    		}
    		else{
    			atSpeed = false;
    		}
    	
    if(xBox.getRawAxis(3)>.1){//Right Trigger
    	if(atSpeed){
			roller.set(-0.5);
			sweeper.set(-0.5);
			Timer.delay(.15);//How long roller spins
			roller.set(0);
			sweeper.set(0);
			Timer.delay(.15);//Time between spins
			}
    	else{
    		roller.set(0);
    	}
    }
    
    	/*
    	//Gear Control - Moves to open while holding B
    	if(flagOne.getValue()>3000 && !open){//Right Sensor is detected (When Closed)
    		fOne = 1;//Sets variables for Troubleshooting purposes
    		fTwo = 0;
    		
    		if(!xBox.getRawButton(2)){//When B button is not pressed
    			gear.set(0);//Do nothing
    			closed = true;//This prevents the other sensor from interuppting
    		}
    		else{//Button is pressed
    			gear.set(-.4);//Move the flag towards sensor two ("flagTwo")
    			inTransition = true;
    			closed = false;
    		}
    	}
    	else if(flagTwo.getValue()>2500 && !closed){//Left sensor is detected (When Open)
    		fTwo = 1;
    		fOne = 0;
    		
    		if(!xBox.getRawButton(2)){//Button not pressed
    			gear.set(1);//Move the flag towards closed
    			inTransition = true;
    			open = false;
    		}
    		else{//Button is pressed
    			gear.set(0);//Do nothing
    			open = true;//This prevents the other sensor from interuppting
    		}
    	}
    	else if(inTransition){//If neither sensors are detected (Transitioning between opened and closed)
    		fOne = 0;
    		fTwo = 0;
    		if(!xBox.getRawButton(2)){//Button not pressed
    			gear.set(1);//Move towards closed state
    			Timer.delay(.175);//wait
    			gear.set(0.25);//slow down
    			inTransition = false;//Setting this to false causes the motor to continue moving until
    			//either sensor detects the flag
    		}
    		else{//Button is pressed
    			gear.set(-1);//Same as above, but moves towards open
    			Timer.delay(.15);
    			gear.set(-0.35);
    			inTransition = false;
    		}
    	}
    	else if(xBox.getRawButton(7)){//Gear testing: Comment out when done
    		gear.set(-.3);
    	}
    	*/
    	
    	
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
