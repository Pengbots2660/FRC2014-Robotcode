#include "WPILib.h" //Includes all functions for the FIRST Library

class RobotDemo : public SimpleRobot 
{ 
    CANJaguar fl; //initiate front left wheel
    CANJaguar bl; //initiate back left wheel
    CANJaguar fr; //initiate front right wheel
    CANJaguar br; //initiate back right wheel
    CANJaguar leftFork; //left fork motor
    CANJaguar rightFork; //right fork motor	
    Victor comp;
    Compressor compressor; 
    DoubleSolenoid Piston;
    RobotDrive myRobot; 
    Watchdog dog; 
    Joystick stick;
    AnalogChannel Sonic;
  
public: 
    RobotDemo(void): 
          
        fl(1, CANJaguar::kPercentVbus), 
        bl(2, CANJaguar::kPercentVbus), 
        fr(4, CANJaguar::kPercentVbus), 
        br(3, CANJaguar::kPercentVbus), 
        leftFork(5, CANJaguar::kPercentVbus), 
        rightFork(6, CANJaguar::kPercentVbus), 
        comp(10),
        compressor(1,1),
        Piston(1,2),
        myRobot(fl,bl,fr,br),
        stick(1),
        Sonic (1,1)
    { 
        myRobot.SetExpiration(0.5); 
        myRobot.SetInvertedMotor(RobotDrive::kFrontLeftMotor,true); 
        myRobot.SetInvertedMotor(RobotDrive::kRearLeftMotor,true);       
        dog.SetExpiration(0.5); 
        dog.SetEnabled(false);
    } 
  
      
    void Autonomous(void) 
    { 
    	dog.SetEnabled(false);
    	while (IsAutonomous()){
    		compressor.Start(); 
    		AnalogModule* aMod = AnalogModule::GetInstance(1); 
            float dist = aMod->GetAverageVoltage(1);
    		if(dist > 0.5){
    			fl.Set(-0.375,0);
    			bl.Set(-0.375,0);
    			fr.Set(0.375,0);
    			br.Set(0.375,0);
    		}else{
    			fl.Set(-0.0,0);
    			bl.Set(-0.0,0);
    			fr.Set(0.0,0);
    			br.Set(0.0,0);
    		}
    	}
    } 
  
      
    void OperatorControl(void) 
    { 
//        AnalogModule* aMod = AnalogModule::GetInstance(1); 
        DigitalModule* dMod = DigitalModule::GetInstance(1);
        compressor.Start(); 
        dMod->AllocateDIO(2, false); //Firing Mechanism
        
        fl.SetSafetyEnabled(false);
        bl.SetSafetyEnabled(false);
        fr.SetSafetyEnabled(false);
        br.SetSafetyEnabled(false);
        myRobot.SetSafetyEnabled(false);
        dog.SetEnabled(false);
          
        while (IsOperatorControl()) 
        { 
              
//----------------------Decleration-------------------------------------------------------- 

            bool trigger = stick.GetTrigger();  //firing piston 
            bool up = stick.GetRawButton(5);    // 
            bool dw = stick.GetRawButton(3);    //
            bool upF = stick.GetRawButton(6);   //
            bool dwF = stick.GetRawButton(4);   // 
            float xaxis = stick.GetX();         //robotdrive-side 
            float yaxis = stick.GetY();         //robotdrive-F/B 
            float twist = stick.GetZ();         //robotdrive-rotate 
              
              
//----------------------Pnumatic code-------------------------------------------------------             
            comp.Set(1,0);
            
            if(trigger==true){ 
                Piston.Set(DoubleSolenoid::kForward); 
            }else if(trigger==false){ 
                Piston.Set(DoubleSolenoid::kReverse); 
            }
              
//----------------------Motor Controller----------------------------------------------------             
              
            if(up==true){
				rightFork.Set(.5,0);		//move the right fork
				leftFork.Set(.5,0);		//move the left fork
			}else if(dw==true){
				rightFork.Set(-.5,0);		//move the right fork
				leftFork.Set(-.5,0);		//move the left fork
			}else if(upF==true){
				rightFork.Set(1,0);		//move the right fork
				leftFork.Set(1,0);		//move the left fork
			}else if(dwF==true){
				rightFork.Set(-1,0);	//move the right fork
				leftFork.Set(-1,0);		//move the left fork
			}else{
				rightFork.Set(0,0);		//move the right fork
				leftFork.Set(0,0);		//move the left fork
			}

//----------------------Drive Train--------------------------------------------------
              
            myRobot.MecanumDrive_Cartesian(xaxis,yaxis,twist,0); // drive with arcade style (use right stick) 
            Wait(0.005);                //   x     y   rotate 
       
        } 
    } 
      
      
    void Test() { 
  
    } 
}; 
  
START_ROBOT_CLASS(RobotDemo); 
