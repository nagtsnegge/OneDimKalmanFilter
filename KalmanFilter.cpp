#include <iostream>
#include <cmath>
#include "KalmanFilter.hpp"

/** 
 *  @file   KalmanFilter.cpp
 *  @brief  Implements a simple one-dimensional version of the Kalman filter to smooth sensor values.
 *  @author Noah Gerstlauer 
 *  @date   2023-10-XX
 ***********************************************/


KalmanFilter::KalmanFilter(const double estErr, const double procNoise, const double measNoise) :
	x(0),					// Current State
	x_old(0),				// Previous State
	p(estErr + procNoise),	// Estimation Variance
	q(procNoise),			// Process noise variance
	r(measNoise) {}			// Measurement variance

// Filter the given value
int KalmanFilter::filterValue(int z)
{
	// Calculation of the Kalman gain based on the estimation error and the measurement error
	k = p / (p + r);

	// Estimate the current state based on the previous state and the Kalman gain
	x = x_old + k * (z - x);

	// Update the error estimation
	p = ((1 - k) * p) + abs(x_old-x) * q;

	// Save the current state for the next iteration
	x_old = x;

	return round(x);
}

// Reset the Kalman filter
void KalmanFilter::init(const double estErr, const double procNoise, const double measNoise)
{
	x = 0;
	x_old = 0;
	p = estErr + procNoise;
	q = procNoise;
	r = measNoise;
}

// Update the estimation error
void KalmanFilter::updateMeasurementNoise(const double measNoise)
{
	r = measNoise;
}


//##############################################
// Getter
//

int KalmanFilter::getState() const
{
	return x;
}

double KalmanFilter::getKalmanGain() const
{
	return k;
}

