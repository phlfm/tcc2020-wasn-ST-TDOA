/* *** Time Difference of Arrival position estimation methods ***

This header file includes the declarations and configurations for all methods
related to the TDOA position estimation methods.

Author: Pedro Henrique Lage Furtado de Mendonça
All rights reserved - october 2020

*/
/* TODO: Fix refSensor not working properly
TODO: Add convergion verification in Foy and stop iterations if NaN or "exploding"
TODO: add parameter verification in calculateTDOA_maxThreshold */
#ifdef __cplusplus

#ifndef H_TDOA
#define H_TDOA

#include <iostream>
#include <string>
#include "Eigen/Core"
#include "Eigen/LU" // necessary for .inverse()

typedef unsigned int uint;
using Eigen::MatrixBase;

namespace TDOA {

	enum ReturnCode {
		OK = 0, OK_maxIterationReached, OK_minErrorReached,
		err_refSensorTooBig = -200, err_refSensorBug, err_itErrorNonConvergence
	}; // enum TDOA_Return

	const char* ReturnCodeMsg(ReturnCode Code) {
		switch (Code) {
		// OK codes
			case OK: return "OK"; break;
			case OK_maxIterationReached: return "OK (max iteration count reached)"; break;
			case OK_minErrorReached: return "OK (minimum desired error reached)"; break;
			case err_refSensorTooBig: return "ERROR: reference sensor index is outside range"; break;
			case err_refSensorBug: return "When refSensor != 0, the results are not what I expected. I think there is a bug in some matrix index but I have not had the time to search for it."; break;
			case err_itErrorNonConvergence: return "ERROR: iteration error 'exploded'"; break;
		} // switch
		return "Unknown code";
	} // char* ReturnCodeMsg

    namespace withEigen {

			/*
			TDOA_Foy estimates source location based on TDOA using Foys iterative method
			(Taylor-Series expansion into iterative solution)
				ref: W. H. Foy, “Position-location solutions by Taylor-series estimation,”
					IEEE Trans. Aerosp. Electron. Syst., vol. AES-12, pp. 187-194, Mar.1976.

				ref2: Hyperbolic Position Location Estimation in the Multipath Propagation Environment
					Jacek Stefański - Gdansk University of Technology (jstef@eti.pg.gda.pl)

				The method tries to allocate as few memory as possible and thus asks for parameters as pointers

			TEMPLATE PARAMETERS:
				N = sensor count
				S = space dimension (2d --> S = 2)
				scalar = data type (int, float, double)...
				Leave the other parameters in blank because the compiler will infere them
			PARAMETERS:
			(notation: S = space dimension, N = sensor count)
				sensorPositionsMatrix* sensorPositions (matrix S by N)
					- CONST (is not changed by function)
					- columns are sensor positions --> N = sensor count
					- rows are position coordinate --> S = space dimension
					- must have equal or more columns than rows
				TDOAsMatrix* TDOAs
					- CONST (is not changed by function)
					- must be N by N
					- TDOAs.row(i).col(j) represents TOAi - TOAj
					-example 3by3:
					TDOAs[3 by 3] = TOA0-TOA0     TOA0-TOA1       TOA0-TOA2
									TOA1-TOA0     TOA1-TOA1       TOA1-TOA2
									TOA2-TOA0     TOA2-TOA1       TOA2-TOA2
				estimateMatrix* positionEstimate
					- the function DOES alter the value of this parameter
					- on return, this parameter will hold the final position estimation
					- should be initialized with the first position guess
					- This is a column vector S by 1
				scalar const signalSpeed
				scalar const maxError
					if iterationError gets below this value, iterations stop
				scalar &iterationError
					- if the method runs (no errors in parameter verification), then the
					   method DOES alter this value to reflect the error of the last iteration
				unsigned int &iterationCount
					- the function DOES alter the value of this parameter
					- on return, this parameter will hold the final iteration number
					- initialization value does NOT matter (method sets it to zero initialy)
				unsigned int maxIterations = 10,
					- iterations will stop once iterationCount reaches maxIterations
				bool printSteps = false
					- prints each iterations information
				uint const &refSensor = 0
					- Specifies which sensor should be used as reference (number 0 to N-1)

	sensorPositions is (S by N+) so I can't hardcode the type as Matrix<scalar, S, N> because it can be
	Matrix<scalar, S, N+3> for example.
			*/
template <uint N, uint S, typename scalar, typename S_by_Nplus, typename N_by_N, typename S_by_1>
inline ReturnCode Foy(
		MatrixBase<S_by_Nplus> const &sensorPositions,
		MatrixBase<N_by_N> const &TDOAs,
		MatrixBase<S_by_1> &positionEstimate,
		scalar const &signalSpeed, scalar const &maxError, scalar &iterationError,
		uint &iterationCount, uint const &maxIterations, std::string *printSteps = nullptr, uint const &refSensor = 0)
{
		// parameter verifications
			// Assert that number of sensors is equal or bigger than space dimension
				static_assert(N > S, "Sensor count (N) must be bigger than space dimension (S)");
			// Assert that sensorPositions dimensions match N and S
				static_assert(sensorPositions.ColsAtCompileTime == N, "sensorPositions must have (N) cols at compile time");
				static_assert(sensorPositions.RowsAtCompileTime == S, "sensorPositions must have (S) rows at compile time");
			// Assert that positionEstimate is the same size as space space dimension
				static_assert(positionEstimate.ColsAtCompileTime == 1, "positionEstimate must have ONE cols at compile time");
				static_assert(positionEstimate.RowsAtCompileTime == S, "positionEstimate must have (S) rows at compile time");
			// Assert that TDOAs is a square matrix of same size of number of sensors
				static_assert(TDOAs.RowsAtCompileTime == N && TDOAs.ColsAtCompileTime == N, "TDOAs must be a square matrix with N by N");
				if (refSensor > N-1) { return err_refSensorTooBig;}
				if (refSensor != 0) { return err_refSensorBug; } /* TODO: Fix refSensor not working properly */

		// Step 1 - initial guess and TDOAs (both are being passed into the function as parameters)
		// Start iterations:
			// Declare iteration variables (static so we only allocate them once)
			static Eigen::Matrix<scalar,N,1> dists = Eigen::Matrix<scalar,N,1>::Zero();
			static Eigen::Matrix<scalar,N-1,1> h = Eigen::Matrix<scalar,N-1,1>::Zero();
			static Eigen::Matrix<scalar,N-1, S> G = Eigen::Matrix<scalar,N-1,S>::Zero();
			static Eigen::Matrix<scalar,1,S> refG = Eigen::Matrix<scalar,1,S>::Zero();
			// G is (N-1, S) so Gt is (S, N-1) so GtG is (S,S) and GtGinv is also (S, S)
			static Eigen::Matrix<scalar,S,S> GtGinv = Eigen::Matrix<scalar,S,S>::Zero();
			static Eigen::Matrix<scalar,S,1> currentError = Eigen::Matrix<scalar,S,1>::Ones();

		// Note: When iterating eigen matrices, iterate first the COL and then the ROW
			for (iterationCount = 0; iterationCount < maxIterations; ++iterationCount) {
				uint reflessIndex = 0;
			// Step 2 - calculate distance of sensors to alleged source
				for (uint sensor = 0; sensor < N; ++sensor) {
					dists[sensor] = (sensorPositions.col(sensor)-positionEstimate).norm();
				// Calculate matrix h for step 3 using the same iteration of step 2 ;)
					if (sensor == refSensor) { continue; } // skip h calculation if refSensor
					reflessIndex = sensor; if (sensor > refSensor)  {--reflessIndex;} // go back one index to compensate the ref sensor
					h[reflessIndex] = signalSpeed*TDOAs(sensor, refSensor) - dists[sensor] + dists[refSensor];
				} // for sensor loop step 2
				#ifdef TDOA_DEBUG
					cout << "\ndists:\n" << dists << "\n";
					cout << "\nh:\n" << h<<"\n";
				#endif
			// Step 3 - Calculate current error
				// Calculate matrix G (N-1 columns with S rows -- N = #sensors, S = #dimensions)
					// refG is a ROW vector with (REF_POS - ESTIMATE)/DIST[REF]
					// Eigen doesn't care that the result is a COL and that the vector is a ROW, so no need to transpose
					refG = (sensorPositions.col(refSensor)-positionEstimate)/dists[refSensor];
				#ifdef TDOA_DEBUG
					cout << "\nrefG:\n" << refG << "\n";
				#endif
					for (uint sensor = 0; sensor < N; ++sensor) {
						if (sensor == refSensor) { continue; } // skip G calculation if refSensor
						reflessIndex = sensor; if (sensor > refSensor)  {--reflessIndex;} // go back one index to compensate the ref sensor
						G.row(reflessIndex).noalias() = refG - (sensorPositions.col(sensor)-positionEstimate).transpose()/dists[sensor];
					} // for sensor loop step 3
					GtGinv.noalias() = (G.transpose()*G).inverse();

				#ifdef TDOA_DEBUG
					cout << "\nG:\n" << G << "\n";
					cout << "\nGtGinv:\n" << GtGinv << "\n";
				#endif

				// Calculate current error
					currentError.noalias() = GtGinv*G.transpose()*h;

			// Check if convergence is failing
				iterationError = currentError.norm();
				if (!(iterationError==iterationError) || iterationError > 100) {
					return err_itErrorNonConvergence;
				} // if iterationError > 100

			// Step 4 - Calculate new position
				positionEstimate = (positionEstimate + currentError).eval();

				// print steps
					if (printSteps != nullptr) {
						*printSteps += "Foy_iteration = " + std::to_string(iterationCount) + "\t with error " + std::to_string(iterationError) + "\t\t and position estimation at [";
						for (int p = 0; p < positionEstimate.size(); ++p) { *printSteps += std::to_string(positionEstimate(p)) + " ";}
						*printSteps += "]\n\r";
					} // if printSteps
				// check convergion criterion
					if (iterationError <= maxError) {
						return OK_minErrorReached;
					} // if iterationError <= maxError
			} // for iterationCount

			// If in the future we need distance of sensor to source, update dists matrix a last time here
			return OK_maxIterationReached;
		} // TDOA_Foy


	/* calculateTDOA_maxThreshold
			- Loops the Buffer looking for the maximum value (higher than "threshold") for each channel.
			- With the maximum it calculates the TDOAs for each channel.
			- The TDOA is determined by the "index offset" divided by samplingFrequency
			- TDOA(i,j) = TOAi - TOAj
			- sampleBuffer must have interleaved samples (ch0, ch1, ch2, ..., ch(N-1), ch0, ch1, ...)
		!!! CRITICAL: !!!
			(!!!) samplesAtTOA will return the highest sample BIGGER than threshold for each channel
			(!!!)     If the value is EQUAL to "threshold" then the user should
						assume that the TDOA for that channel should be ignored since
						no sample reached "threshold"
	*/
		template <uint N, uint sampleBufferSize, typename sampleType, typename sfType, typename TDOAsType, typename N_by_N, typename ONE_by_N>
		inline void calculateTDOA_maxThreshold(MatrixBase<N_by_N> &TDOAs, sampleType const (&sampleBuffer)[sampleBufferSize],
		sfType const &samplingFrequency, MatrixBase<ONE_by_N> &samplesAtTOA, sampleType const &threshold) {
		// initialize samplesAtTOA to threshold. A sample is only valid if it is BIGGER than threshold
			static Eigen::Matrix<sampleType,1,N> TOA_line;
			unsigned char TOA_found = 0;
			samplesAtTOA.fill(threshold);
		// iterate through buffer
			uint chIndex = 0; // index for accesing each channels TOA_line, TDOA, samplesAtTOA
			for (uint i = 0; i < sampleBufferSize; ++i) {
				chIndex = i % N; // will refer to channel 0, 1, 2, ... N-1, 0, 1, 2, ..., N-1, ...
			// if sample > TOA[channel] --> get new sample value and index
				if (sampleBuffer[i] > samplesAtTOA[chIndex] && !(TOA_found & chIndex)) {
					samplesAtTOA[chIndex] = sampleBuffer[i];
					TOA_line[chIndex] = (i-chIndex)/N;
					TOA_found = TOA_found | (1<<chIndex);
				}
				if (TOA_found == (1<<N)-1) { break; }
			} // iterate through buffer
			//cout << "Max index at TOA_line = \n" << TOA_line << "\n";
			TDOAs.colwise() = TOA_line.transpose().template cast<TDOAsType>(); // makes TDOA = [TOA0 TOA0 TOA0][TOA1 TOA1 TOA1][TOA2 TOA2 TOA2]
			TDOAs.noalias() = TDOAs.rowwise() - TOA_line.template cast<TDOAsType>(); // makes TDOA = [TOA0-0 TOA0-1 TOA0-2][TOA1-0 TOA1-1 TOA1-2][TOA2-0 TOA2-1 TOA2-2]
			TDOAs = TDOAs / (TDOAsType)samplingFrequency; // TDOA holds sample indice difference, so we divide by sampleFrequency to get time
			return;
			/**/
		} //calculateTDOA_maxThreshold
    } // namespace withEigen
} // namespace TDOA

/* Translation units need to see template IMPLEMENTATIONS at compile-time so either
implement in a header file or have the header file include the implementation file */
#endif // H_TDOA
#endif //#ifdef __cplusplus
