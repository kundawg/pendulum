//
// Created by wankunsirichotiyakul on 5/10/17.
//

#ifndef PENDULUM_CONTROLLER_H
#define PENDULUM_CONTROLLER_H

#define PI 3.14159265358979
#define M_PI 3.14159265358979323846f
#define FLOATSIZE 4
#define LOOP_PERIOD_MS 2
// #define FILTER_CUT_OFF_FREQUENCY 62.83
#define FILTER_CUT_OFF_FREQUENCY 7.0f
#define FILTER_SAMPLE_PERIOD 0.002 // = 1/500  Experimentally this turns out to be 0.0021216
#define MAX_PATH 1024

#include <math.h>
#include <thread>
#include <mutex>
#include <iostream>
#include <unistd.h>
#include "serial/serial.h"
#include "SerialCommunicator.h"
#include "../include/LowPassFilter.h"
#include "../include/NatNetTypes.h"

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

typedef union {
    float floatingPoint;
    uint8_t binary[FLOATSIZE];
} binaryFloat;

typedef std::chrono::high_resolution_clock Clock;
typedef std::chrono::duration<double, std::milli> millisec_t;

class Controller {
public:
    Controller();
    ~Controller();

    int initialize();
    void getGains(double *positionGain, double *velocityGain);
    void setGains(const double &positionGain, const double &velocityGain);
    void getDefaultGains(double *positionGain, double *velocityGain);
    void setDefaultGains(const double &positionGain, const double &velocityGain);
    void getEnergyGains(double *positionGain, double *velocityGain, double *swingGain);  // added by Aykut
    void setEnergyGains(const double &positionGain, const double &velocityGain, const double &swingGain);  // added by Aykut

protected:

    double computeVelocity();
    double computeDesiredTorque();
    double computeEnergyBasedTorque();   // added by Aykut

    void controlArmThread();
    void logger();

private:
    bool m_initialized;
    FILE* fp;
    int writeCounter;

    Clock::time_point t1, t2;
    millisec_t time_span;

    double data[2];
    double L = 35.8;    // Pendulum arm length
    double g = 9.81;         // Gravitational acceleration
    double gamma = g/L;      //

    double k1;
    double k2;

    // Quantities for Energy-Based Control [added by Aykut]
    double totalEnergy;
    double referenceEnergy;
    double errorEnergy;
    double theta;
    double thetaDot;
    double kswing;
    double velWeight = 0.25;

    double k1Default;
    double k2Default;

    double m_desiredTorque;

    //control thread
    float m_motorPosMeasRad = 0;
    float m_motorVelMeasRadS = 0;

    LowPassFilter thetaDotLPF;

    void writeFrame(FILE* fp, double *data);

    // DON'T TOUCH THOSE OTHER THAN BY GET SET FUNCTIONS
    uint64_t m_controlThreadCounter = 0;
    SerialCommunicator *m_serialComm;
    std::mutex m_mutexGains;
    bool m_debug = false;

};


#endif //PENDULUM_CONTROLLER_H
