#ifndef PENDULUM_LOWPASSFILTER_H
#define PENDULUM_LOWPASSFILTER_H


class LowPassFilter {
public:
    LowPassFilter(){}
    LowPassFilter(double omega, double T);
    ~LowPassFilter() {}
    void setFilterParameters(const double& omega, const double& T);
    double calculate(double currentState);

private:
    double cutOffFrequency;
    double samplePeriod;
    double lastState;
    double lastStateDerivative;
    double stateDerivative;
    double wT;
};


#endif //PENDULUM_LOWPASSFILTER_H
