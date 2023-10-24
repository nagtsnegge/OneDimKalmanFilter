#ifndef KALMANFILTER_HPP
#define KALMANFILTER_HPP

/** 
 *  @file   KalmanFilter.hpp
 *  @brief  Implements a simple one-dimensional version of the Kalman filter to smooth sensor values.
 *  @author Noah Gerstlauer 
 *  @date   2023-10-XX
 ***********************************************/


class KalmanFilter {
	public:

		/**
		 * @brief Construct a new Kalman Filter object. This is the default constructor.
		 * 
		 */
		KalmanFilter(){};

		/**
		 * @brief Construct a new Kalman Filter object
		 * 
		 * @param estErr Standart deviation of the initial estimates value. (Estimate Error) 
		 * @param procNoise Accuracy of the model. (Process noise)
		 * @param measNoise Standart deviation of the measurement (Measurement Error)
		 */
		KalmanFilter(const double estErr, const double procNoise, const double measNoise);

		/**
		 * @brief Applies the Kalman filter to the passed measured value
		 *
		 * @param z Measured value.
		 *
		 * @return Current Kalman-Filter state.
		 *
		 * @author Noah Gerstlauer
		 * @date 2023-10-12
		 */
		int filterValue(int z);

		/**
		 * @brief Resets the parameters of the filter.
		 *
		 * Must only be used if you need it afterwards. When initializing the object, the parameters are already applied by the constructor.
		 *
		 * @param estErr Standart deviation of the initial estimates value. (Estimate Error) 
		 * @param procNoise Process noise. (Accuracy of the model)
		 * @param measNoise Standart deviation of the measurement (Measurement Error)
		 *
		 * @author Noah Gerstlauer
		 * @date 2023-10-12
		 */
		void init(const double estErr, const double procNoise, const double measNoise);
	
		/**
		 * @brief Updates the measurement error.
		 *
		 * @param measNoise Standart deviation of the measurement (Measurement Error)
		 */
		void updateMeasurementNoise(const double measNoise);

		/**
		 * @brief Getter for the current state.
		 *
		 * @return Current Kalman-Filter state.
		 */
		int getState() const;

		/**
		 * @brief Getter for the current Kalman gain.
		 *
		 * @return Current Kalman gain.
		 */
		double getKalmanGain() const;

	private:

		double x;		// State Value	
		double x_old;	// Old state value	
		double q;		// Process Noise
		double r;		// Measurement Error
		double p;		// Estimate Error
		double k;		// Kalman Gain
};

#endif
