#include "../include/Controller.h"



Controller::Controller() :
        m_initialized(false),
        thetaDotLPF(FILTER_CUT_OFF_FREQUENCY ,FILTER_SAMPLE_PERIOD) {

    writeCounter = 0;
    double k1 = 1.0;
    double k2 = 0.3;
    double k11 = 0.01;
    double k22 = 0.1;
    double kswing = 0.005;                  // added by Aykut

    this->setDefaultGains(k11, k22);
    this->setGains(k1, k2);
    this->setEnergyGains(k1, k2, kswing); // added by Aykut


    char szFile[MAX_PATH];
    char szFolder[MAX_PATH] = {};
    if (getcwd(szFolder, sizeof(szFolder)) != NULL)
        fprintf(stdout, "Current working dir: %s\n", szFolder);
    else
        perror("getcwd() error");
    sprintf(szFile, "%s/Client-output.dat", szFolder);
    fp = fopen(szFile, "w");
    if (!fp)
    {
        printf("error opening output file %s.  Exiting.", szFile);
        exit(1);
    }
    fprintf(fp, "%s\t%s\t%s\n", "frame", "theta", "thetaDot");
}

Controller::~Controller() {
    fclose(fp);
    delete m_serialComm;
}

int Controller::initialize() {
    if (m_initialized) {
        printf("Controller already initialized. Exiting");
    }

    try {
        m_serialComm = new SerialCommunicator();
    }
    catch (...) {
        // catch any serial errors!
        printf("COM Port not open!");
        return 1;
    }

    // create a motor object
    std::cout << "Controller Init, Thread :: ID = " << std::this_thread::get_id() << std::endl;
    int motorSetPosition = 0;
    //std::cout << "In Main Thread : Before Thread Start motorSetPosition = " << motorSetPosition << std::endl;

    // ToDo: get a handle on that thread
    m_initialized = true;

    // Start the Thread!
    std::thread threadObj(&Controller::controlArmThread, this);
    if (threadObj.joinable()) {
        //threadObj.join();
        //std::cout << "Joined Thread " << std::endl;
        std::cout << "Detaching Control Arm Thread " << std::endl;
        threadObj.detach();
    }
    return ErrorCode_OK;
}


void Controller::getDefaultGains(double *positionGain, double *velocityGain) {
    std::lock_guard<std::mutex> guard(m_mutexGains);
    *positionGain = k1Default;
    *velocityGain = k2Default;
}

void Controller::setDefaultGains(const double &positionGain, const double &velocityGain) {
    std::lock_guard<std::mutex> guard(m_mutexGains);
    k1Default = positionGain;
    k2Default = velocityGain;
}

void Controller::getGains(double *positionGain, double *velocityGain) {
    std::lock_guard<std::mutex> guard(m_mutexGains);
    *positionGain = k1;
    *velocityGain = k2;
}

void Controller::setGains(const double &positionGain, const double &velocityGain) {
    std::lock_guard<std::mutex> guard(m_mutexGains);
    k1 = positionGain;
    k2 = velocityGain;
}

// The two methods below are added by Aykut
void Controller::getEnergyGains(double *positionGain, double *velocityGain, double *swingGain) {
    std::lock_guard<std::mutex> guard(m_mutexGains);
    *positionGain = k1;
    *velocityGain = k2;
    *swingGain = kswing;
}

void Controller::setEnergyGains(const double &positionGain, const double &velocityGain, const double &swingGain) {
    std::lock_guard<std::mutex> guard(m_mutexGains);
    k1 = positionGain;
    k2 = velocityGain;
    kswing = swingGain;
}

double Controller::computeVelocity() {
    // double velocity = (m_motorPosMeasRadNew - m_motorPosMeasRadOld) / (float) LOOP_PERIOD_MS * 1000.0f;

    return thetaDotLPF.calculate(m_motorPosMeasRad);
}

double Controller::computeDesiredTorque() {
    return -k1*(m_motorPosMeasRad-M_PI) + -k2*m_motorVelMeasRadS;
}

// This method is added by Aykut
double Controller::computeEnergyBasedTorque() {
  theta = m_motorPosMeasRad - M_PI;
  thetaDot = m_motorVelMeasRadS;
  totalEnergy = 1/2*pow(thetaDot, 2) + cos(theta);
  referenceEnergy = cos(0.0);
  errorEnergy = totalEnergy - referenceEnergy;

  double effort = 0.0;

//    if ( (abs(theta) + velWeight*abs(thetaDot) < 0.5) || (abs(theta + 2*M_PI) + velWeight*abs(thetaDot) < 0.5) ) {
//        if (abs(theta) + velWeight*abs(thetaDot) < 0.5)
//            effort = -k1 * theta - k2 * thetaDot;
//        if (abs(theta + 2*M_PI) + velWeight*abs(thetaDot) < 0.5)
//            effort = -k1 * (theta + 2*M_PI) - k2 * thetaDot;
//    }
    if (abs(remainder(theta, 2*M_PI)) + velWeight*abs(thetaDot) < 0.3)
        effort = -k1 * remainder(theta, 2*M_PI) - k2 * thetaDot;
    else
        effort = -kswing * thetaDot * errorEnergy;


  return effort;
}

void Controller::controlArmThread() {

    while (true) {
        // Ensure it was initialized!
        if (!m_initialized) {
            printf("please init() controller!");
            return;
        }

        // Position and Velocity of Pendulum from Mbed
        m_motorPosMeasRad = this->m_serialComm->readEncoderRad();
        printf("m_motorPosMeasRad: %5.3f\n", m_motorPosMeasRad);

        // Calculate Loop Frequency
//        t2 = Clock::now();
//        time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
//        t1 = Clock::now();

        // Calculate velocity and save to array
        writeCounter++;
        m_motorVelMeasRadS = this->computeVelocity();
        data[0] = m_motorPosMeasRad;
        data[1] = m_motorVelMeasRadS;
        printf("m_motorVelMeasRadS: %5.3f\n", m_motorVelMeasRadS);

        // Computation
        //m_desiredTorque = this->computeDesiredTorque();
        m_desiredTorque = this->computeEnergyBasedTorque();
        printf("computeDesiredTorque: %5.3f\n", m_desiredTorque);

        // Send to mbed
        this->m_serialComm->sendMotorTorque((float) m_desiredTorque);
        m_controlThreadCounter++;
        std::this_thread::sleep_for(std::chrono::milliseconds(LOOP_PERIOD_MS));

        // Write data to file
        writeFrame(fp, data);
    }
}

double sgn(double d) {
    double eps = 1e-16;
    return d < -eps ? -1 : d > eps;
}

void Controller::writeFrame(FILE* fp, double *data)
{
    //fprintf(fp, "%d\t%.6f\t", writeCounter, time_span);
    fprintf(fp, "%d\t", writeCounter);
    fprintf(fp, "%.6f\t%.6f", data[0], data[1]);
    fprintf(fp, "\n");
}
