#include "LoadCell.h"

#include <iostream>

std::mutex sensorLock;  

LoadCell::LoadCell(int dataInterval, PhidgetVoltageRatioInput_BridgeGain bridgeGain)
{

	this->zero_reference_voltage = -2.32e-03;

	this->dataInterval = dataInterval; 
		
	this->bridgeGain = bridgeGain;

	PhidgetLog_enable(PHIDGET_LOG_INFO, NULL);
	
	res = PhidgetVoltageRatioInput_create(&ch);
	if (res != EPHIDGET_OK) 
	{
		fprintf(stderr, "failed to create voltage ratio input channel\n");
		exit(1);
	}

	res = this->initialize((PhidgetHandle) this->ch);
	if (res != EPHIDGET_OK) 
	{
		Phidget_getErrorDescription(res, &errs);
		fprintf(stderr, "failed to initialize channel:%s\n", errs);
		exit(1);
	}
	


	res = PhidgetVoltageRatioInput_setOnVoltageRatioChangeHandler(ch, LoadCell::onVoltageRatioChangeHandler, this);
	if (res != EPHIDGET_OK) 
	{
		Phidget_getErrorDescription(res, &errs);
		fprintf(stderr, "failed to set voltage ratio change handler: %s\n", errs);
	}

	/*
	 * Open the channel synchronously: waiting a maximum of 5 seconds.
	 */
	res = Phidget_openWaitForAttachment((PhidgetHandle)ch, 5000);
	if (res != EPHIDGET_OK) 
	{
		if (res == EPHIDGET_TIMEOUT) 
		{
			printf("Channel did not attach after 5 seconds: please check that the device is attached\n");
		} else 
		{
			Phidget_getErrorDescription(res, &errs);
			fprintf(stderr, "failed to open channel:%s\n", errs);
		}
	}

	res = PhidgetVoltageRatioInput_setBridgeGain(ch, BRIDGE_GAIN_128);
	PhidgetVoltageRatioInput_BridgeGain gain;
	PhidgetVoltageRatioInput_getBridgeGain(ch, &gain);

	uint32_t interval = -1;
	res = PhidgetVoltageRatioInput_setDataInterval(ch, this->dataInterval);
	res = PhidgetVoltageRatioInput_getDataInterval(ch, &interval);
	

	res = Phidget_getDeviceID((PhidgetHandle)ch, &deviceID);
	if (res != EPHIDGET_OK) 
	{
		fprintf(stderr, "failed to get device ID\n");
		return;
	}

	if (deviceID == PHIDID_1046) 
	{
		printf("setting bridge enabled\n");
		res = PhidgetVoltageRatioInput_setBridgeEnabled(ch, 1);
		if (res != EPHIDGET_OK) 
		{
			fprintf(stderr, "failed to set bridge enabled\n");
			return;
		}
	}
	

}

LoadCell::~LoadCell()
{
	res = Phidget_close((PhidgetHandle)ch);
	res = PhidgetVoltageRatioInput_delete(&ch);

	delete(this->errs);

	::std::cout << "exited with code: " << this->res << ::std::endl;
}

PhidgetReturnCode 
LoadCell::initialize(PhidgetHandle ch)
{
	PhidgetReturnCode res;
	res = Phidget_setChannel(ch, 1); 

	res = Phidget_setOnAttachHandler(ch, LoadCell::onAttachHandler, this);
	if (res != EPHIDGET_OK) 
	{
		fprintf(stderr, "failed to assign on attach handler\n");
		return (res);
	}

	res = Phidget_setOnDetachHandler(ch, LoadCell::onDetachHandler, NULL);
	if (res != EPHIDGET_OK) 
	{
		fprintf(stderr, "failed to assign on detach handler\n");
		return (res);
	}

	res = Phidget_setOnErrorHandler(ch, LoadCell::errorHandler, NULL);
	if (res != EPHIDGET_OK) 
	{
		fprintf(stderr, "failed to assign on error handler\n");
		return (res);
	}

	return (EPHIDGET_OK);
}

void
LoadCell::getMeasurement(double& meas)
{
	sensorLock.lock();
	meas = this->current_measurement;
	sensorLock.unlock();
}

void
LoadCell::getRawMeasurement(double& meas)
{
	sensorLock.lock();
	meas = this->raw_measurement;
	sensorLock.unlock();
}



void CCONV LoadCell::onAttachHandler(PhidgetHandle phid, void *ctx)
{

	LoadCell* cellPtr = reinterpret_cast<LoadCell*> (ctx);

	PhidgetReturnCode res;
	int hubPort;
	int channel;
	int serial;

	res = Phidget_getDeviceSerialNumber(phid, &serial);
	if (res != EPHIDGET_OK) 
	{
		fprintf(stderr, "failed to get device serial number\n");
		return;
	}
	
	res = Phidget_getChannel(phid, &channel);
	if (res != EPHIDGET_OK) 
	{
		fprintf(stderr, "failed to get channel number\n");
		return;
	}

	res = Phidget_getHubPort(phid, &hubPort);
	if (res != EPHIDGET_OK) 
	{
		fprintf(stderr, "failed to get hub port\n");
		hubPort = -1;
	}

	if (hubPort == -1)
		printf("channel %d on device %d attached\n", channel, serial);
	else
		printf("channel %d on device %d hub port %d attached\n", channel, serial, hubPort);

}


void CCONV LoadCell::onDetachHandler(PhidgetHandle phid, void *ctx)
{
	PhidgetReturnCode res;
	int hubPort;
	int channel;
	int serial;

	res = Phidget_getDeviceSerialNumber(phid, &serial);
	if (res != EPHIDGET_OK)
	{
		fprintf(stderr, "failed to get device serial number\n");
		return;
	}

	res = Phidget_getChannel(phid, &channel);
	if (res != EPHIDGET_OK)
	{
		fprintf(stderr, "failed to get channel number\n");
		return;
	}

	res = Phidget_getHubPort(phid, &hubPort);
	if (res != EPHIDGET_OK)
		hubPort = -1;

	if (hubPort != -1)
		printf("channel %d on device %d detached\n", channel, serial);
	else
		printf("channel %d on device %d hub port %d detached\n", channel, hubPort, serial);

}


void CCONV LoadCell::errorHandler(PhidgetHandle phid, void *ctx, Phidget_ErrorEventCode errorCode, const char *errorString)
{
	fprintf(stderr, "Error: %s (%d)\n", errorString, errorCode);
}


void CCONV LoadCell::onVoltageRatioChangeHandler(PhidgetVoltageRatioInputHandle ch, void *ctx, double ratio)
{
	LoadCell* cellPtr = reinterpret_cast<LoadCell*> (ctx);

	sensorLock.lock();

	cellPtr->setMeasurement(cellPtr->voltageRatioToForce(ratio));
	cellPtr->setRawMeasurement(ratio);

	sensorLock.unlock();
}


double LoadCell::voltageRatioToForce(double voltageRatio)
{
	double slope = -9.1828;

	return slope * (voltageRatio - this->zero_reference_voltage) * 1.0e03;
}
