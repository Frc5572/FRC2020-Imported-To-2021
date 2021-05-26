#include "Movement/DriveTrainManager.hpp"

/* CAN ID layout for drive train from a top view

          Front of Robot
         
         |--------------|
         |              |
      1  | M1        M2 | 2
      3  | M3        M4 | 3
      5  | M5        M6 | 6
         |              |
         |              |
         |--------------|

          Back Of Robot
*/
void DriveTrain::InitPID() 
{
    
    m_pidControllerLeft->SetP(kP);
    m_pidControllerLeft->SetI(kI);
    m_pidControllerLeft->SetD(kD);
    // m_pidControllerLeft->SetIZone(kIz);
    // m_pidControllerLeft->SetFF(kFF);
    m_pidControllerLeft->SetOutputRange(kMinOutput, kMaxOutput);

    m_pidControllerRight->SetP(kP);
    m_pidControllerRight->SetI(kI);
    m_pidControllerRight->SetD(kD);
    // m_pidController2->SetIZone(kIz);
    // m_pidController2->SetFF(kFF);
    m_pidControllerRight->SetOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    frc::SmartDashboard::PutNumber("P Gain", kP);
    frc::SmartDashboard::PutNumber("I Gain", kI);
    frc::SmartDashboard::PutNumber("D Gain", kD);
    // frc::SmartDashboard::PutNumber("I Zone", kIz);
    // frc::SmartDashboard::PutNumber("Feed Forward", kFF);
    frc::SmartDashboard::PutNumber("Max Output", kMaxOutput);
    frc::SmartDashboard::PutNumber("Min Output", kMinOutput);
    frc::SmartDashboard::PutNumber("SetPoint", SetP);
    frc::SmartDashboard::PutNumber("Auto Selection", AutoSelection);
}

void DriveTrain::AutoPID()
{
    double p = frc::SmartDashboard::GetNumber("P Gain", 0);
    double i = frc::SmartDashboard::GetNumber("I Gain", 0);
    double d = frc::SmartDashboard::GetNumber("D Gain", 0);
    // double iz = frc::SmartDashboard::GetNumber("I Zone", 0);
    // double ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);
    double max = frc::SmartDashboard::GetNumber("Max Output", 0);
    double min = frc::SmartDashboard::GetNumber("Min Output", 0);
    double SetPoint = frc::SmartDashboard::GetNumber("SetPoint", 0);
    double Auto = frc::SmartDashboard::GetNumber("Auto Selection", 0);
        
        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((p != kP)) { m_pidControllerLeft->SetP(p); m_pidControllerRight->SetP(p); kP = p; }
        if((i != kI)) { m_pidControllerLeft->SetI(i); m_pidControllerRight->SetI(i); kI = i; }
        if((d != kD)) { m_pidControllerLeft->SetD(d); m_pidControllerRight->SetD(d); kD = d; }
        // if((iz != kIz)) { m_pidControllerLeft->SetIZone(iz); m_pidController2->SetIZone(iz); kIz = iz; }
        // if((ff != kFF)) { m_pidControllerLeft->SetFF(ff); m_pidController2->SetFF(ff); kFF = ff; }
        if((max != kMaxOutput) || (min != kMinOutput)) { 
        m_pidControllerLeft->SetOutputRange(min, max); m_pidControllerRight->SetOutputRange(min, max); 
        kMinOutput = min; kMaxOutput = max; 
        }

    if (Auto == 0)
    {
        SetPoint = 2700; //3375
        SP = SetPoint;
        // m_pidControllerLeft->SetReference(SetPoint, rev::ControlType::kVelocity);
        leftRPM = leftMotorEncoder->GetVelocity();
        rightRPM = rightMotorEncoder->GetVelocity();
        rpm = ((leftRPM + rightRPM) / 2);   
    }

    else
    {
        SetPoint = 0;
    }
    
         

    frc::SmartDashboard::PutNumber("SetPoint", SetPoint);
    frc::SmartDashboard::PutNumber("ProcessVariable", leftMotorEncoder->GetVelocity());

    frc::SmartDashboard::PutNumber("RPM", rpm );
    frc::SmartDashboard::PutNumber("Left RPM", leftRPM);
    frc::SmartDashboard::PutNumber("Right RPM", rightRPM);
}

void DriveTrain::RunPID()
{
    double p = frc::SmartDashboard::GetNumber("P Gain", 0);
    double i = frc::SmartDashboard::GetNumber("I Gain", 0);
    double d = frc::SmartDashboard::GetNumber("D Gain", 0);
    double iz = frc::SmartDashboard::GetNumber("I Zone", 0);
    double ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);
    double max = frc::SmartDashboard::GetNumber("Max Output", 0);
    double min = frc::SmartDashboard::GetNumber("Min Output", 0);
    double SetPoint = frc::SmartDashboard::GetNumber("SetPoint", 0);
        
        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((p != kP)) { m_pidControllerLeft->SetP(p); m_pidControllerRight->SetP(p); kP = p; }
        if((i != kI)) { m_pidControllerLeft->SetI(i); m_pidControllerRight->SetI(i); kI = i; }
        if((d != kD)) { m_pidControllerLeft->SetD(d); m_pidControllerRight->SetD(d); kD = d; }
        // if((iz != kIz)) { m_pidController->SetIZone(iz); m_pidControllerRight->SetIZone(iz); kIz = iz; }
        // if((ff != kFF)) { m_pidController->SetFF(ff); m_pidControllerRight->SetFF(ff); kFF = ff; }
        if((max != kMaxOutput) || (min != kMinOutput)) { 
        m_pidControllerLeft->SetOutputRange(min, max); m_pidControllerRight->SetOutputRange(min, max); 
        kMinOutput = min; kMaxOutput = max; 
        }

    if(this->Driver->POV() == 0 ){ //bumber
        SetPoint = 2600;
        // Hood->Set(frc::DoubleSolenoid::Value::kReverse);    
    }
    else if(this->Driver->POV() == 90){ //init
        SetPoint = 3375; //
        // Hood->Set(frc::DoubleSolenoid::Value::kForward); 
    }
    else if(this->Driver->POV() == 270){ //trench
        SetPoint = 3800;
        // Hood->Set(frc::DoubleSolenoid::Value::kForward); 
    }
    else if (Driver->POV() == 180) //colorwheel
    {
        SetPoint = 4500;
        // Hood->Set(frc::DoubleSolenoid::Value::kForward);
    }
    else{
        SetPoint = 0;
        // Hood->Set(frc::DoubleSolenoid::Value::kReverse); 
    }

    //  m_pidControllerLeft->SetReference(SetPoint, rev::ControlType::kVelocity);
    //  m_pidControllerRight->SetReference(SetPoint, rev::ControlType::kVelocity);

    frc::SmartDashboard::PutNumber("SetPoint", SetPoint);
    frc::SmartDashboard::PutNumber("ProcessVariable", leftMotorEncoder->GetVelocity());

    
    leftRPM = leftMotorEncoder->GetVelocity();
    rightRPM = rightMotorEncoder->GetVelocity();
    rpm = ((leftRPM + rightRPM) / 2);

    frc::SmartDashboard::PutNumber("RPM", rpm );
    frc::SmartDashboard::PutNumber("Left RPM", leftRPM);
    frc::SmartDashboard::PutNumber("Right RPM", rightRPM);

    }

DriveTrain::DriveTrain(
    rev::CANSparkMax &TopLeftMotor,
    rev::CANSparkMax &TopRightMotor,

    rev::CANSparkMax &MiddleLeft,
    rev::CANSparkMax &MiddleRight,

    rev::CANSparkMax &BottomLeftMotor, 
    rev::CANSparkMax &BottomRightMotor,

    FRC5572Controller &Driver,
    VisionManager &VisionManager,
    AHRS &ahrs
    ){
        this->LeftMotors = new frc::SpeedControllerGroup( TopLeftMotor, MiddleLeft, BottomLeftMotor);
        this->RightMotors = new frc::SpeedControllerGroup( TopRightMotor, MiddleRight, BottomRightMotor);
        this->TempRightMotors = new frc::SpeedControllerGroup( MiddleRight, BottomRightMotor);
        this->TempLeftMotors = new frc::SpeedControllerGroup( MiddleLeft, BottomLeftMotor);

        
        this->Driver = &Driver;
        this->ahrs = &ahrs;

        this->TopLeftMotor = &TopLeftMotor;
        this->TopRightMotor = &TopRightMotor;

        this->MiddleLeft = &MiddleLeft;
        this->MiddleRight = &MiddleRight;

        this->BottomLeftMotor = &BottomLeftMotor;
        this->BottomRightMotor = &BottomRightMotor;

        // this->TopLeftMotorEncoder = new rev::CANEncoder{TopLeftMotor};
        // this->TopRightMotorEncoder = new rev::CANEncoder{TopRightMotor};

        this->leftMotorEncoder = new rev::CANEncoder{MiddleLeft};
        this->rightMotorEncoder = new rev::CANEncoder{MiddleRight};
        
        // this->BottomLeftMotorEncoder = new rev::CANEncoder{BottomLeftMotor};
        // this->BottomRightMotorEncoder = new rev::CANEncoder{BottomRightMotor};

        this->LimeLight = &VisionManager;

        DriveTrain::LowerAmps();
}

DriveTrain::~DriveTrain()
{
    delete LeftMotors;
    delete RightMotors;
    delete Driver;
    delete ahrs;
}

//#define QUAD(x) (log2(x + 1) - 1)

// void DriveTrain::Drive()
// {
//     if(this->Driver->L().second > .2 || this->Driver->L().second < -.2){
        
//         LeftMotors->Set(-1 * QUAD(Driver->L().second) * .8  );
//     }
//     else{
//         LeftMotors->Set(0 + L);
//     }

//     if(this->Driver->R().second > .2 ||  this->Driver->R().second < -.2){
//         RightMotors->Set(QUAD(Driver->R().second)  * .8);
//     }
//     else{
//         RightMotors->Set(0 + R);
//     }
// }

void DriveTrain::Drive()
{
    // DrivseTrain::Aim();
    if(this->Driver->L().second > .2 || this->Driver->L().second < -.2){
        LeftMotors->Set(-1 * Driver->L().second * .7  );
    }
    else{
        LeftMotors->Set(0 + L);
    }

    if(this->Driver->R().second > .2 ||  this->Driver->R().second < -.2){
        RightMotors->Set(Driver->R().second  * .7);
    }
    else{
        RightMotors->Set(0 + R);
    }
}

void DriveTrain::LowerAmps(){
    TopLeftMotor->SetSmartCurrentLimit(60);
    TopRightMotor->SetSmartCurrentLimit(60);

    MiddleLeft->SetSmartCurrentLimit(60);
    MiddleRight->SetSmartCurrentLimit(60);

    BottomLeftMotor->SetSmartCurrentLimit(60);
    BottomRightMotor->SetSmartCurrentLimit(60);
}

void DriveTrain::Aim(){
    if (Driver->X() ==  true)
    {
        disX = LimeLight->disX; 
        nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("camMode", 0);
        nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode", 3);
    }
    else if (Driver->Y() == false){
        nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("camMode", 1);
        nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode", 1);
        disX = 0;
        L = 0; R = 0;
    }
        
    if (fabs(disX) > 1 && Driver->X() == true)
        {
            if (disX > 10)
            {
                R = -.15;  
                L = -.15;
            }
            if (disX < 10)
            {
                R = -disX/43; 
                L = -disX/43;
            } 
            if (disX < -10)
            {
                R = .15;
                L = .15;
            }
            if (disX > -10)
            {
                R = disX/43; 
                L = disX/43;
            }
        }
    else{
        L = 0;
        R = 0;
    }
}

// void DriveTrain::RunPID()
// {

// }
