#include "WPILib.h"
#include "MyJoystick.h"
#include "AHRS.h"

#define ANALOG0 0
#define ANALOG1 1
#define ANALOG2 2

#define USB0 0
#define USB1 1
#define USB2 2

#define PWM0 0
#define PWM1 1
#define PWM2 2
#define PWM3 3
#define PWM4 4
#define PWM5 5

#define PCM0 0
#define PCH0 0
#define PCH1 1
#define PCH2 2
#define PCH3 3
#define PCH4 4
#define PCH5 5

#define DIO0 0
#define DIO1 1
#define DIO2 2
#define DIO3 3
#define DIO4 4
#define DIO5 5
#define DIO6 6
#define DIO7 7
#define DIO8 8
#define DIO9 9

#define PULSEIN 0.15

class Robot : public IterativeRobot {
public:
	Joystick left;
	Joystick right;
	Joystick controller;

	Spark leftDriveA;
	Spark leftDriveB;
	Spark rightDriveA;
	Spark rightDriveB;
	Spark ropeClimber;
	Spark ballShooter;

	DoubleSolenoid leftGearBox;
	DoubleSolenoid rightGearBox;
	DoubleSolenoid gearPusher;

	Compressor compressor;

	MyJoystick* rightJoystick;
	MyJoystick* leftJoystick;
	MyJoystick* handheld;

	DigitalInput* ropeLimit;
	AnalogInput leftUltrasonic;
	AnalogInput rightUltrasonic;
	Ultrasonic* springUltrasonic;
	DigitalInput* pusherSensor;

	DigitalInput* switchA;
	DigitalInput* switchB;

	Encoder leftEncoder;
	Encoder rightEncoder;

	AHRS* ahrs;

	std::shared_ptr<NetworkTable> table;
	std::vector<double> xCoords;
	double xCoordA;
	double xCoordB;

	bool autoAiming;
	bool driveSwapped;
	bool enabled;
	bool pressureStatus;
	float current;

	bool ultrasonicAligning;
	bool ultrasonicApproaching;
	bool ultrasonicRetreating;
	bool approaching;
	bool turning;
	bool boiler;
	double medianDistance;
	double minDistance;
	double endDistance;
	double distanceToTurn;
	double valueToInches;

	int currentAuto;
	std::string autostate;

	bool singleOperatorMode;
	bool controllerMode;

	enum Gears {
		up,
		down
	} leftGear, rightGear;

	enum AutoState {
		init,
		forward,
		turn,
		step0,
		step1,
		step2,
		step3,
		step4,
		step5,
		step6,
		step7,
		done
	} autoStep;

	Robot() :
		left(USB0),
		right(USB1),
		controller(USB2),
		leftDriveA(PWM0),
		leftDriveB(PWM1),
		rightDriveA(PWM2),
		rightDriveB(PWM3),
		ropeClimber(PWM4),
		ballShooter(PWM5),
		leftGearBox(PCM0, PCH2, PCH3),
		rightGearBox(PCM0, PCH4, PCH5),
		gearPusher(PCM0, PCH0, PCH1),
		compressor(PCM0),
		leftUltrasonic(ANALOG0),
		rightUltrasonic(ANALOG1),
		leftEncoder(DIO5, DIO4),
		rightEncoder(DIO7, DIO6)
	{
		singleOperatorMode = false;
		controllerMode = false;
		rightJoystick = new MyJoystick();
		leftJoystick = new MyJoystick();
		handheld = new MyJoystick();

		ahrs = new AHRS(SerialPort::kMXP);

		table = NetworkTable::GetTable("GRIP/myContoursReport");
		xCoords = table->GetNumberArray("xCoords", llvm::ArrayRef<double>());
		if (xCoords.size() > 1)
		{
			xCoordA = xCoords[0];
			xCoordB = xCoords[1];
		}
		autoAiming = false;
		current = 0;
		enabled = false;
		pressureStatus = 0;

		medianDistance = 750;
		minDistance = 350;
		endDistance = 500;
		valueToInches = 0.125;
		distanceToTurn = 420;
		ultrasonicAligning = false;
		ultrasonicApproaching = false;
		ultrasonicRetreating = false;
		approaching = false;
		turning = false;
		boiler = false;

		springUltrasonic = new Ultrasonic(DIO9, DIO8);
		springUltrasonic->SetAutomaticMode(true);

		leftDriveA.SetExpiration(0.1);
		leftDriveB.SetExpiration(0.1);
		rightDriveA.SetExpiration(0.1);
		rightDriveB.SetExpiration(0.1);

		driveSwapped = false;
		leftGear = down;
		rightGear = down;

		ropeLimit = new DigitalInput(DIO0);
		pusherSensor = new DigitalInput(DIO1);
		switchA = new DigitalInput(DIO2);
		switchB = new DigitalInput(DIO3);

		autoStep = init;

		leftEncoder.Reset();
		rightEncoder.Reset();
		leftEncoder.SetDistancePerPulse(PULSEIN);
		rightEncoder.SetDistancePerPulse(-PULSEIN);

		currentAuto = 0;
		autostate = "";
	}

	~Robot()
	{
		delete pusherSensor;
		delete ropeLimit;
		delete rightJoystick;
		delete leftJoystick;
		delete handheld;
	}

	void RobotInit()
	{
		CameraServer::GetInstance()->StartAutomaticCapture("cam0", 0);
		CameraServer::GetInstance()->StartAutomaticCapture("cam1", 1);
		enabled = compressor.Enabled();
		pressureStatus = compressor.GetPressureSwitchValue();
		current = compressor.GetCompressorCurrent();
		compressor.SetClosedLoopControl(true);
	}

	void AutonomousInit()
	{
		int switchAVal = switchA->Get();
		int switchBVal = switchB->Get();

//		if (switchCVal == 0)
//		{
			boiler = true;
			distanceToTurn = 400;
//		}
//		else
//		{
//			boiler = false;
//			distanceToTurn = 420;
//		}

		if (switchAVal == 0 && switchBVal == 0)
		{
			currentAuto = 0;
		}
		else if (switchAVal == 0 && switchBVal == 1)
		{
			currentAuto = 1;
		}
		else if (switchAVal == 1 && switchBVal == 0)
		{
			currentAuto = 2;
		}
		else
		{
			currentAuto = 3;
		}

//		currentAuto = 1;

		if (currentAuto == 1)
		{
			autoStep = step0;
			autostate = "step0";
		}
		else
		{
			autoStep = init;
			autostate = "init";
		}
	}

	void AutonomousPeriodic()
	{
		if (currentAuto == 0)
		{
			LeftAuto();
		}
		else if (currentAuto == 1)
		{
			CenterAuto();
		}
		else if (currentAuto == 2)
		{
			RightAuto();
		}

		SmartDashboard::PutNumber("Current Auto", currentAuto);
		SmartDashboard::PutNumber("Yaw", ahrs->GetYaw());

		SmartDashboard::PutString("AutoState", autostate);
		SmartDashboard::PutNumber("Distance Until Turn", distanceToTurn);

	}

	void LeftAuto()
	{
		if (autoStep == init)
		{
			ShiftDown();
			ResetEverything();
			ahrs->ZeroYaw();
			ahrs->Reset();
			turning = true;
			autoStep = forward;
		}
		else if (autoStep == forward)
		{
			autostate = "forward";
			approaching = true;
			MoveForward(0.5, distanceToTurn, turn);

		}
		else if (autoStep == turn)
		{
			autostate = "turn";
			TurnGyro(45, 0.5, step0);
		}
		else
		{
			autostate = "Moving on to center code.";
			CenterAuto();
		}
	}

	void CenterAuto()
	{
		if (autoStep == step0)
		{
			autostate = "step0";
			ResetEverything();
			autoStep = step1;
		}
		else if (autoStep == step1)
		{
			autostate = "step1";
			ultrasonicAligning = true;
			AlignWithGear(leftUltrasonic.GetValue() - rightUltrasonic.GetValue(), step2);
		}
		else if (autoStep == step2)
		{
			autostate = "step2";
			ultrasonicApproaching = true;
			ApproachGear(medianDistance, step3);
		}
		else if (autoStep == step3)
		{
			autostate = "step3";
			GearAim(step4);
		}
		else if (autoStep == step4)
		{
			autostate = "step4";
			ultrasonicApproaching = true;
			ApproachGear(minDistance, step5);
		}
		else if (autoStep == step5)
		{
			autostate = "step5";
			CheckSpring(step6);
		}
		else if (autoStep == step6)
		{
			autostate = "step6";
			ultrasonicRetreating = true;
			Retreat(endDistance, step7);
		}
		else if (autoStep == step7)
		{
			autostate = "step7";
			if (pusherSensor->Get() == 0)
			{
				AutoPushGear(false); //RETRACT GEAR PUSHER
				autoStep = done;
			}
		}
		else if (autoStep == done)
		{
			autostate = "done";
			ResetEverything();
		}
	}

	void RightAuto()
	{
		if (autoStep == init)
			{
				ShiftDown();
				ResetEverything();
				ahrs->ZeroYaw();
				ahrs->Reset();
				turning = true;
				autoStep = forward;
			}
			else if (autoStep == forward)
			{
				autostate = "forward";
				approaching = true;
				MoveForward(0.5, distanceToTurn, turn);

			}
			else if (autoStep == turn)
			{
				autostate = "turn";
				TurnGyro(-52.5, 0.5, step0);
			}
			else
			{
				autostate = "Moving on to center code.";
				CenterAuto();
			}
	}

	void BasicAuto()
	{
		if (autoStep == step0)
		{
			ResetEverything();
			autoStep = step1;
		}
		else if (autoStep == step1)
		{
			ultrasonicAligning = true;
			AlignWithGear(leftUltrasonic.GetValue() - rightUltrasonic.GetValue(), step2);
		}
		else if (autoStep == step2)
		{
			ultrasonicApproaching = true;
			ApproachGear(medianDistance, step3);
		}
		else if (autoStep == step3)
		{
			GearAim(step4);
		}
		else if (autoStep == step4)
		{
			ultrasonicApproaching = true;
			ApproachGear(minDistance, done);
		}
		else if (autoStep == done)
		{
			ResetEverything();
		}
	}

	void ResetEverything()
	{
		leftEncoder.Reset();
		rightEncoder.Reset();
		ahrs->ZeroYaw();
	}

	void MoveForward(double speed, double distance, AutoState nextStep)
	{
		if (approaching)
		{
			if (leftEncoder.Get() < distance)
			{
				SetMotors(speed, speed);
			}
			else
			{
				approaching = false;
				SetMotors(0, 0);
				autoStep = nextStep;
			}
		}
	}

	void TurnStandalone (bool direction, double speed)
	{
		if (direction) //TURN RIGHT
		{
			SetMotors(-speed, speed);
		}
		else
		{
			SetMotors(speed, -speed);
		}
	}

	void TurnGyro(double angle, double speed, AutoState nextStep)
	{
		if (turning)
		{
			if (angle < 0) //TURN COUNTERCLOCKWISE
			{
				if (ahrs->GetYaw() >= angle)
				{
					SetMotors(-speed, speed);
				}
				else
				{
					SetMotors(0, 0);
					turning = false;
				}
			}
			else if (angle > 0) //TURN CLOCKWISE
			{
				if (ahrs->GetYaw() <= angle)
				{
					SetMotors(speed, -speed);
				}
				else
				{
					SetMotors(0, 0);
					turning = false;
				}
			}
		}
		else
		{
			autoStep = nextStep;
		}
	}

	void AlignWithGear (double difference, AutoState nextStep)
	{
		if (ultrasonicAligning)
		{
			int turning = 0;
			ShiftDown();
			if (difference > 50)
			{
				turning = 1;
				TurnStandalone(false, 0.33); //TURN RIGHT
			}
			else if (difference < -50)
			{
				turning = 2;
				TurnStandalone(true, 0.33); //TURN LEFT
			}
			else
			{
				turning = 0;
				SetMotors(0, 0);
				ultrasonicAligning = false;
				autoStep = nextStep;
			}

			SmartDashboard::PutNumber("L=1 R=2", turning);
		}
	}

	void ApproachGear (double distance, AutoState nextStep)
	{
		double averageUltrasonic = (leftUltrasonic.GetValue() + rightUltrasonic.GetValue()) / 2;
		if (ultrasonicApproaching)
		{
			ShiftDown();
			if ( averageUltrasonic > distance)
			{
				SetMotors(0.4, 0.4);
			}
			else
			{
				SetMotors(0, 0);
				ultrasonicApproaching = false;
				autoStep = nextStep;
			}
		}
	}

	void Retreat (double distance, AutoState nextStep)
	{
		double averageUltrasonic = (leftUltrasonic.GetValue() + rightUltrasonic.GetValue()) / 2;
		if (ultrasonicRetreating)
		{
			ShiftDown();
			if ( averageUltrasonic < distance)
			{
				SetMotors(-0.33, -0.33);
			}
			else
			{
				SetMotors(0, 0);
				ultrasonicRetreating = false;
				autoStep = nextStep;
			}
		}
	}

	void CheckSpring(AutoState nextStep)
	{
		double distance = springUltrasonic->GetRangeInches();
		if (distance >= 2.5 && distance <= 17)
		{
			AutoPushGear(true);
			autoStep = nextStep;
		}
	}

	void AutoPushGear(bool push)
	{
		if (push)
		{
			gearPusher.Set(DoubleSolenoid::kForward);
		}
		else
		{
			gearPusher.Set(DoubleSolenoid::kReverse);
		}
	}

	void GearAim(AutoState nextStep)
	{
		std::vector<double> xCoords = table->GetNumberArray("centerX", llvm::ArrayRef<double>());
		SmartDashboard::PutNumberArray("X Coords", xCoords);
	//	std::vector<double> areas = table->GetNumberArray("area", llvm::ArrayRef<double>());
		if (xCoords.size() > 1){
			ShiftDown();
			SmartDashboard::PutBoolean("Is Autoaiming?", true);
			double xCoordA = xCoords[0];
			double xCoordB = xCoords[1];
			SmartDashboard::PutNumber("xCoordA", xCoordA);
			SmartDashboard::PutNumber("xCoordB", xCoordB);
			double averageXCoord = (xCoordB + xCoordA)/2;
			SmartDashboard::PutNumber("Average X Coord", averageXCoord);
			bool isLeft = false;
			bool isRight = false;
			bool isCenter = false;
			if (averageXCoord < 260){
				autoAiming = true;
				isLeft = true;
				isRight = false;
				isCenter = false;
				leftDriveA.Set(-0.33);
				leftDriveB.Set(-0.33);
				rightDriveA.Set(0.33);
				rightDriveB.Set(0.33);
			}
			else if (averageXCoord > 380){
				autoAiming = true;
				isLeft = false;
				isRight = true;
				isCenter = false;
				rightDriveA.Set(-0.33);
				rightDriveB.Set(-0.33);
				leftDriveA.Set(0.33);
				leftDriveB.Set(0.33);
			}
			else
			{
				isLeft = false;
				isRight = false;
				isCenter = true;
				rightDriveA.Set(0.0);
				rightDriveB.Set(0.0);
				leftDriveA.Set(0.0);
				leftDriveB.Set(0.0);
				autoAiming = false;
				autoStep = nextStep;
			}
			SmartDashboard::PutBoolean("Is Left?", isLeft);
			SmartDashboard::PutBoolean("Is Right?", isRight);
			SmartDashboard::PutBoolean("Is Center?", isCenter);
		}
		else
		{
			autoAiming = false;
		}
	}

	void TeleopInit()
	{
		leftJoystick->init(&left);
		rightJoystick->init(&right);
		handheld->init(&controller);

		driveSwapped = false;
	}

	void TeleopPeriodic()
	{
		leftJoystick->readJoystick();
		rightJoystick->readJoystick();
		handheld->readJoystick();

		SmartDashboard::PutNumber("ZZZ_HANDHELD POV", handheld->getPOV());

		ToggleSingleOperator(rightJoystick->readButton(7), rightJoystick->readButton(8));
		ToggleController(handheld->readButton(9), handheld->readButton(10));

		ManualResetEverything(handheld->readButton(3), leftJoystick->readButton(5));
		ManualGearAim(handheld->readButton(1), leftJoystick->readButton(3));
		SwapDrive(rightJoystick->readButton(5), rightJoystick->readButton(3), handheld->getPOV());
		SmartDashboard::PutBoolean("Drive Swapped?", driveSwapped);
		DualTankDrive();
		ManualShiftGears(rightJoystick->readButton(6),rightJoystick->readButton(4), handheld->getPOV());

		TeleopCheckSpring();
		PushGear(handheld->readButton(6), rightJoystick->readButton(1));
		RetractGear(handheld->readButton(8), leftJoystick->readButton(1));
		ClimbRope(handheld->readButton(4), handheld->readButton(2), leftJoystick->readButton(6), leftJoystick->readButton(4));
		BallShooter(handheld->readButton(5), handheld->readButton(7), rightJoystick->readButton(12), rightJoystick->readButton(11));

		ManualAlignment(leftJoystick->readButton(2), leftUltrasonic.GetValue() - rightUltrasonic.GetValue());
		ManualApproach(leftJoystick->readButton(3), (leftUltrasonic.GetValue() + rightUltrasonic.GetValue()) / 2);

		SmartDashboard::PutNumber("Left Ultrasonic", leftUltrasonic.GetValue());
		SmartDashboard::PutNumber("Right Ultrasonic", rightUltrasonic.GetValue());
		SmartDashboard::PutNumber("Difference", leftUltrasonic.GetValue() - rightUltrasonic.GetValue());
		SmartDashboard::PutNumber("Left Encoder", leftEncoder.Get());
		SmartDashboard::PutNumber("Right Encoder", rightEncoder.Get());
		SmartDashboard::PutNumber("Avg Encoder", (leftEncoder.Get() + rightEncoder.Get()) / 2);
		SmartDashboard::PutNumber("Spring Ultrasonic", springUltrasonic->GetRangeInches());
		SmartDashboard::PutNumber("SwitchA", switchA->Get());
		SmartDashboard::PutNumber("SwitchB", switchB->Get());
		SmartDashboard::PutNumber("Distance Until Turn", distanceToTurn);

	}

	void SetMotors(double left, double right){
		leftDriveA.Set(left);
		leftDriveB.Set(left);
		rightDriveA.Set(right);
		rightDriveB.Set(right);
	}

	void ManualGearAim(bool aimButton, bool single){
		std::vector<double> xCoords = table->GetNumberArray("centerX", llvm::ArrayRef<double>());
		SmartDashboard::PutNumberArray("X Coords", xCoords);
	//	std::vector<double> areas = table->GetNumberArray("area", llvm::ArrayRef<double>());

		bool button;

		if (singleOperatorMode)
		{
			button = single;
		}
		else
		{
			button = aimButton;
		}

		if (xCoords.size() > 1 && button == true){
			ShiftDown();
			SmartDashboard::PutBoolean("Is Autoaiming?", true);
			double xCoordA = xCoords[0];
			double xCoordB = xCoords[1];
			SmartDashboard::PutNumber("xCoordA", xCoordA);
			SmartDashboard::PutNumber("xCoordB", xCoordB);
			double averageXCoord = (xCoordB + xCoordA)/2;
			SmartDashboard::PutNumber("Average X Coord", averageXCoord);
			bool isLeft = false;
			bool isRight = false;
			bool isCenter = false;
			if (averageXCoord < 260){
				autoAiming = true;
				isLeft = true;
				isRight = false;
				isCenter = false;
				leftDriveA.Set(-0.33);
				leftDriveB.Set(-0.33);
				rightDriveA.Set(0.33);
				rightDriveB.Set(0.33);
			}
			else if (averageXCoord > 380){
				autoAiming = true;
				isLeft = false;
				isRight = true;
				isCenter = false;
				rightDriveA.Set(-0.33);
				rightDriveB.Set(-0.33);
				leftDriveA.Set(0.33);
				leftDriveB.Set(0.33);
			}
			else
			{
				isLeft = false;
				isRight = false;
				isCenter = true;
				rightDriveA.Set(0.0);
				rightDriveB.Set(0.0);
				leftDriveA.Set(0.0);
				leftDriveB.Set(0.0);
				autoAiming = false;
			}
			SmartDashboard::PutBoolean("Is Left?", isLeft);
			SmartDashboard::PutBoolean("Is Right?", isRight);
			SmartDashboard::PutBoolean("Is Center?", isCenter);
		}
		else
		{
			autoAiming = false;
		}
	}

	void SwapDrive(bool forward, bool reverse, double controllerPOV) {
		if (controllerMode)
		{
			if (controllerPOV == 90)
			{
				if (driveSwapped)
				{
					driveSwapped = false;
				}
			}

			if (controllerPOV == 270)
			{
				if (!driveSwapped)
				{
					driveSwapped = true;
				}
			}
		}
		else
		{
			if (forward)
			{
				if (driveSwapped)
				{
					driveSwapped = false;
				}
			}
			if (reverse)
			{
				if (!driveSwapped)
				{
					driveSwapped = true;
				}
			}
		}
	}

	void DualTankDrive()
	{
		double leftInput;
		double rightInput;

		if (controllerMode)
		{
			leftInput = -handheld->checkRightStickY();
			rightInput = handheld->checkLeftStickY();
		}
		else
		{
			leftInput = left.GetRawAxis(1);
			rightInput = right.GetRawAxis(1);
		}
		if (!driveSwapped)
		{
			if(fabs(leftInput) > 0.3) {
				leftDriveA.Set(leftInput * -0.65);
				leftDriveB.Set(leftInput * -0.65);
			}
			else if(!autoAiming)
			{
				leftDriveA.Set(0);
				leftDriveB.Set(0);
			}

			if(fabs(rightInput) > 0.3) {
				rightDriveA.Set(rightInput * -0.65);
				rightDriveB.Set(rightInput * -0.65);
			}
			else if(!autoAiming)
			{
				rightDriveA.Set(0);
				rightDriveB.Set(0);
			}
		}
		else if (driveSwapped)
		{
			if(fabs(leftInput) > 0.3) {
				rightDriveA.Set(leftInput * 0.65);
				rightDriveB.Set(leftInput * 0.65);
			}
			else if(!autoAiming)
			{
				SmartDashboard::PutString("Right", "Inactive");
				rightDriveA.Set(0);
				rightDriveB.Set(0);
			}

			if(fabs(rightInput) > 0.3) {
				leftDriveA.Set(rightInput * 0.65);
				leftDriveB.Set(rightInput * 0.65);
			}
			else if(!autoAiming)
			{
				leftDriveA.Set(0);
				leftDriveB.Set(0);
			}
		}
	}

	void ShiftDown()
	{
		if(leftGear == up && rightGear == up)
		{
			leftGearBox.Set(DoubleSolenoid::kReverse);
			rightGearBox.Set(DoubleSolenoid::kReverse);
			leftGear = down;
			rightGear = down;
		}
	}

	void ShiftUp()
	{
		if(leftGear == down && rightGear == down)
		{
			leftGearBox.Set(DoubleSolenoid::kForward);
			rightGearBox.Set(DoubleSolenoid::kForward);
			leftGear = up;
			rightGear = up;
		}
	}
	void ManualShiftGears(bool upButton, bool downButton, double handheldPOV)
	{
		if (controllerMode)
		{
			if (handheldPOV == 0)
			{
				ShiftUp();
			}

			if (handheldPOV == 180)
			{
				ShiftDown();
			}
		}
		else
		{
			if(upButton)
			{
				ShiftUp();
			}

			if(downButton)
			{
				ShiftDown();
			}
		}
	}


	void PushGear(bool pushButton, bool single)
	{
//		bool button;
//
//		if (singleOperatorMode)
//		{
//			button = single;
//		}
//		else
//		{
//			button = pushButton;
//		}

		SmartDashboard::PutNumber("Pusher Status", pusherSensor->Get());
		if (pushButton || single)
		{
			gearPusher.Set(DoubleSolenoid::kForward);
		}

		if (pusherSensor->Get() == 0)
		{
			gearPusher.Set(DoubleSolenoid::kReverse);
		}
	}

	void RetractGear(bool retractButton, bool single)
	{
//		bool button;
//
//		if (singleOperatorMode)
//		{
//			button = single;
//		}
//		else
//		{
//			button = retractButton;
//		}

		SmartDashboard::PutNumber("PusherStatus", pusherSensor->Get());
		if (retractButton || single)
		{
			gearPusher.Set(DoubleSolenoid::kReverse);
		}
	}

	void ClimbRope (bool climbButton, bool descendButton, bool singleClimb, bool singleDescend)
	{
		bool climb;
		bool descend;

		if (singleOperatorMode)
		{
			climb = singleClimb;
			descend = singleDescend;
		}
		else
		{
			climb = climbButton;
			descend = descendButton;
		}

		if (climb && !descend && ropeLimit->Get())
		{
			ropeClimber.Set(1.0);
		}
		else if (!climb && descend)
		{
			ropeClimber.Set(-1.0);
		}
		else
		{
			ropeClimber.Set(0.0);
		}
	}

	void BallShooter (bool fireButton, bool collectButton, bool singleFire, bool singleCollect)
	{
		bool fire;
		bool collect;
		if (singleOperatorMode)
		{
			fire = singleFire;
			collect = singleCollect;
		}
		else
		{
			fire = fireButton;
			collect = collectButton;
		}

		if (fire && !collect)
		{
			ballShooter.Set(1.0);
		}
		else if (!fire && collect)
		{
			ballShooter.Set(-1.0);
		}
		else
		{
			ballShooter.Set(0.0);
		}
	}

	void ManualResetEverything (bool resetButton, bool single)
	{

		bool reset;

		if(singleOperatorMode)
		{
			reset = single;
		}
		else
		{
			reset = resetButton;
		}

		if (reset)
		{
			leftEncoder.Reset();
			rightEncoder.Reset();
		}
	}

	void ManualAlignment(bool alignButton, double difference)
	{
		if (alignButton)
		{
			int turning = 0;
			ShiftDown();
			if (difference > 50)
			{
				turning = 1;
				TurnStandalone(false, 0.33); //TURN RIGHT
			}
			else if (difference < -50)
			{
				turning = 2;
				TurnStandalone(true, 0.33); //TURN LEFT
			}
			else
			{
				turning = 0;
				SetMotors(0, 0);
				ultrasonicAligning = false;
			}

			SmartDashboard::PutNumber("L=1 R=2", turning);
		}
	}

	void ManualApproach(bool approachButton, double distance){
		if (approachButton)
		{
			ShiftDown();
			if (distance > minDistance)
			{
				SetMotors(0.33, 0.33);
			}
			else
			{
				SetMotors(0, 0);
			}
		}
	}

	void TeleopCheckSpring()
	{
		double distance = springUltrasonic->GetRangeInches();
		bool springInside = false;
		if (distance >= 2.5 && distance <= 10)
		{
			springInside = true;
		}
		else
		{
			springInside = false;
		}
		SmartDashboard::PutBoolean("Spring Inside", springInside);
	}

	void ToggleSingleOperator(bool enable, bool disable)
	{
		if (enable)
		{
			singleOperatorMode = true;
		}

		if (disable)
		{
			singleOperatorMode = false;
		}
	}

	void ToggleController(bool enable, bool disable)
	{
		if (enable)
		{
			controllerMode = true;
		}

		if (disable)
		{
			controllerMode = false;
		}
	}
};

START_ROBOT_CLASS(Robot);
