#pragma once

#include <stdio.h>
#include <stdlib.h>
#ifdef __cplusplus

extern "C" {
#endif
#include <phidget22.h> 
#ifdef __cplusplus
}
#endif

#include <mutex>


#ifndef _WIN32
#include <unistd.h>
#else
#include <Windows.h>
#endif

static void CCONV ssleep(int);

class LoadCell
{

		PhidgetVoltageRatioInputHandle ch;

		PhidgetReturnCode res;

		const char *errs;

		Phidget_DeviceID deviceID;

		double current_measurement;

		double zero_reference_voltage;

		double raw_measurement;

		int dataInterval; 
		
		PhidgetVoltageRatioInput_BridgeGain bridgeGain;

	public:

		LoadCell(int dataInterval, PhidgetVoltageRatioInput_BridgeGain bridgeGain = BRIDGE_GAIN_1);

		~LoadCell();

		void getMeasurement(double& meas);

		void getRawMeasurement(double& meas);

		void setZeroReference(double reference) {this->zero_reference_voltage = reference;};

	protected:

		PhidgetReturnCode initialize(PhidgetHandle ch);

		static void CCONV onAttachHandler(PhidgetHandle phid, void *ctx);

		static void CCONV onDetachHandler(PhidgetHandle phid, void *ctx);

		static void CCONV errorHandler(PhidgetHandle phid, void *ctx, Phidget_ErrorEventCode errorCode, const char *errorString);

		static void CCONV onVoltageRatioChangeHandler(PhidgetVoltageRatioInputHandle ch, void *ctx, double ratio); 

		void setMeasurement(double meas) {this->current_measurement = meas;};

		void setRawMeasurement(double meas) {this->raw_measurement = meas;};

		double voltageRatioToForce(double voltageRatio);
};