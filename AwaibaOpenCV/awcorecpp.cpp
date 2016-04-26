#include <cassert>
#include <exception>
#include <algorithm>
#include <stdint.h>

#include <awcorecpp.h>

//using namespace std;

namespace Core {

   //std::vector< std::tr1::shared_ptr<Manager> > Manager::Instances;

/// Used to convert white level table for and back.
struct i2f { float operator()(int val) { return static_cast<float>(val) / 65536.f; } };
struct f2i { int operator()(float val) { return static_cast<int>(val * 65536.f); } };

/// Convert error codes back to std::exception.
void HandleError(Interface::ErrorIndex index)
{
	// If index != 0, we have an error.
	if (0 == index) return;

	// Get length info of error.
	int length = Interface::GetErrorText(index, 0, 0);

	// Read error string.
	::std::vector<char> tmp(length, 0);
	length = Interface::GetErrorText(index, &tmp[0], length);
	assert(0 == length && !!"Invalid error interface implementation.");
	::std::string error(tmp.begin(), tmp.end());

	// Throw as std::runtime_error.
	throw ::std::runtime_error(error);
}


/// Read information about source.
Interface::SourceInfo Source::GetSourceInfo(int managerNumber) const
{
	Interface::SourceInfo ret;
	
	HandleError(Interface::GetSourceInfo(&ret, managerNumber) );
	return ret;
}

/// Read information about destination.
Interface::DestinationInfo Destination::GetDestinationInfo(int managerNumber) const
{
	Interface::DestinationInfo ret;
	HandleError(Interface::GetDestinationInfo(_handle, &ret,managerNumber));
	return ret;
}

/// Load from file and apply to the specified manager instance
void BlackLevel::Load(const std::string &fileName)
{
	HandleError(Interface::LoadBlackLevelDimensions(fileName.c_str(), &_dimensions.X, &_dimensions.Y));
	_buffer.resize(_dimensions.X * _dimensions.Y * 4, 0);
	//HandleError(Interface::LoadBlackLevel(fileName.c_str(), _dimensions.X, _dimensions.Y, &_buffer[0], Offset, RowsInReset));
}

/// Save to file.
void BlackLevel::Save(const std::string &fileName) const
{ 
	//HandleError(Interface::SaveBlackLevel(fileName.c_str(), _dimensions.X, _dimensions.Y, const_cast<int*>(&_buffer[0]))); 
}

/// Load from file.
void WhiteLevel::Load(const std::string &fileName)
{
	HandleError(Interface::LoadWhiteLevelDimensions(fileName.c_str(), &_dimensions.X, &_dimensions.Y));

	// Data is stored as int (fix point) per value, 
	// Load it as int ...
	::std::vector<int> tmp(_dimensions.X * _dimensions.Y * 4, 0);
	//HandleError(Interface::LoadWhiteLevel(fileName.c_str(), _dimensions.X, _dimensions.Y, &tmp[0],Offset, RowsInReset));

	// and convert to float.
	_buffer.resize(_dimensions.X * _dimensions.Y * 4, 0);

	transform(tmp.begin(), tmp.end(), _buffer.begin(), i2f());
}

/// Save to file.
void WhiteLevel::Save(const std::string &fileName) const
{ 
	// Data is stored as fix point int, convert float values first.
	::std::vector<int> tmp(_dimensions.X * _dimensions.Y, 0);

	// Convert to int array.
	transform(_buffer.begin(), _buffer.end(), tmp.begin(), f2i());

	// And store it.
	//HandleError(Interface::SaveWhiteLevel(fileName.c_str(), _dimensions.X, _dimensions.Y, const_cast<int*>(&tmp[0]))); 
}

/// Verify interface versions and init low level API.
Manager::Manager(int managerNumber)
{
	if (Interface::InterfaceVersion != Interface::GetDllVersion())
		throw std::runtime_error("Invalid core library version.");
	HandleError(Interface::ManagerInit(managerNumber));
}

/// Set new source for the pipe, any previous defined source is stopped
/// and replaced with the new one.
void Manager::SetSource(const Source &source)
{
	Interface::SourceConfig config;

	config.Type = source.GetType();
	
	// Copy source name (safe).
	::std::string sourceName = source.GetSourceName();
	::std::fill(config.SourceName, config.SourceName + Interface::MaxSourceNameLen, 0);
	assert(sourceName.size() < Interface::MaxSourceNameLen); // Must fit (incl. terminator).
	copy(sourceName.begin(), sourceName.end(), config.SourceName);
  
	config.deviceId = source.getDeviceId();

	HandleError(Interface::SetSource(&config, _managerNumber));
}

void Manager::SetSensor(int id)
{
	Interface::SetSensor(id, _managerNumber);
}

void Manager::SetBinFile(char* fileName)
{
	Interface::SetBinFile(fileName, _managerNumber);
}

void Manager::SetLineCorrection(const Sensor &sensor, bool state)
{
	/*Interface::Sensor config;

	Interface::SetLineCorrection(&config, state, _managerNumber);*/
}

void Manager::SetFPGAData(unsigned int address, unsigned int value)
{
	Interface::SetFPGAData(address, value, _managerNumber);
}

std::string Interface::SourceConfig::filePath;

/// Send configuration values to sensor (if source is already set to sensor).
void Manager::ConfigureSensor(const Sensor &sensor)
{
	Interface::Sensor config;

	config.Gain = sensor.Gain;
	config.Offset = sensor.Offset;
	config.Exposure = sensor.Exposure;
	config.DigiPot = sensor.DigiPot;

	config.Vref_cds = sensor.Vref_Cds;
	config.Vrst_pixel = sensor.Vrst_Pixel;

	config.DAC_DREG0 = sensor.DigiPot;
	config.DAC_DREG1 = sensor.DAC_DREG1;
	config.DAC_DREG2 = sensor.DAC_DREG2;
	config.DAC_DREGEN_REG = sensor.DAC_DREGEN_REG;
	config.DAC_DSTEP1 = sensor.DAC_DSTEP1;
	config.DAC_DSTEP2 = sensor.DAC_DSTEP2;

	config.LedState = sensor.LedState;
	config.LedValue = sensor.LedValue;

	HandleError(Interface::ConfigureSensor(&config, _managerNumber));
}

/// Add raw frame destination.
void Manager::AddDestination(Destination &destination)
{
	Interface::DestinationConfig config;

	config.Type = destination.GetType();

	// Copy source name (safe).
	::std::string destName = destination.GetDestinationName();
	::std::fill(config.DestinationName, config.DestinationName + Interface::MaxDestinationNameLen, 0);
	assert(destName.size() < Interface::MaxDestinationNameLen); // Must fit (incl. terminator).
	::std::copy(destName.begin(), destName.end(), config.DestinationName);

	HandleError(Interface::AddDestination(&destination._handle, &config, _managerNumber));
}

/// Finish running destination.
void Manager::FinishDestination(const Destination &destination)
{
	HandleError(Interface::FinishDestination(destination._handle, _managerNumber));
}

/// Set new pipe parameters, can be done on an active pipe too.
void Manager::SetPipeConfig(const PipeConfig &pipeConfig)
{
	Interface::PipeConfig config;

	config.SkipFrames = pipeConfig.SkipFrames.Enable ? Interface::True : Interface::False;
	config.BadPixelReplacement = pipeConfig.BadPixelReplacement.Enable ? Interface::True : Interface::False;
	config.BlackLevelCorrection = pipeConfig.BlackLevelCorrection ? Interface::True : Interface::False;
	config.WhiteLevelCorrection = pipeConfig.WhiteLevelCorrection ? Interface::True : Interface::False;
	config.PixelLinearization = pipeConfig.PixelLinearization ? Interface::True : Interface::False;
	config.DeMosaic = pipeConfig.DeMosaic.Enable ? Interface::True : Interface::False;
	config.SkippingThreshold = pipeConfig.SkipFrames.Threshold;
	config.BadPixelReplacementThreshold = pipeConfig.BadPixelReplacement.Threshold;
	config.PixelAdjustRed = pipeConfig.DeMosaic.PixelAdjustRed;
	config.PixelAdjustGreen1 = pipeConfig.DeMosaic.PixelAdjustGreen1;
	config.PixelAdjustGreen2 = pipeConfig.DeMosaic.PixelAdjustGreen2;
	config.PixelAdjustBlue = pipeConfig.DeMosaic.PixelAdjustBlue;
	config.RedMatrixCol0 = pipeConfig.DeMosaic.RedMatrixCol0;
	config.RedMatrixCol1 = pipeConfig.DeMosaic.RedMatrixCol1;
	config.RedMatrixCol2 = pipeConfig.DeMosaic.RedMatrixCol2;
	config.GreenMatrixCol0 = pipeConfig.DeMosaic.GreenMatrixCol0;
	config.GreenMatrixCol1 = pipeConfig.DeMosaic.GreenMatrixCol1;
	config.GreenMatrixCol2 = pipeConfig.DeMosaic.GreenMatrixCol2;
	config.BlueMatrixCol0 = pipeConfig.DeMosaic.BlueMatrixCol0;
	config.BlueMatrixCol1 = pipeConfig.DeMosaic.BlueMatrixCol1;
	config.BlueMatrixCol2 = pipeConfig.DeMosaic.BlueMatrixCol2;
	config.Brightness = pipeConfig.Brightness;

	HandleError(Interface::SetPipeConfig(&config, _managerNumber));
}

/// Set black level correction table.
void Manager::SetBlackLevel(BlackLevel &table)
{
	HandleError(Interface::SetBlackLevelTable(table.GetWidth(), table.GetHeight(), &table[0], _managerNumber));
}

/// Set white level correction table.
void Manager::SetWhiteLevel(WhiteLevel &table)
{
	// Same as with WhiteLevel::Save().
	::std::vector<int> tmp(table.GetDimensions().X * table.GetDimensions().Y *4, 0);
	transform(table.Begin(), table.End(), tmp.begin(), f2i());
	HandleError(Interface::SetWhiteLevelTable(table.GetWidth(), table.GetHeight(), &tmp[0], _managerNumber));
}

/// Get frame dimensions of current pipe configuration.
XyPairInt Manager::GetFrameDimensions() const
{
	int width = -1, height = -1;
	HandleError(Interface::GetFrameDimensions(&width, &height, _managerNumber));

	return XyPairInt(width, height);
}

/// Read next frame from source. Returns immediately (and true if frames are really read and valid).
/// Both parameters can be NULL, only valid structures are updated.
bool Manager::GetNextFrame(ArgbFrame *argbFrame, RawFrame *rawFrame)
{
	// Define for interface call.
	Interface::PixelArgb *argbFrameP = 0;
	Interface::PixelRaw *rawFrameP = 0;
	Interface::FrameTime frameTime = 0;
	Interface::Boolean valid = Interface::False;

	int8_t pushButton;
    int8_t frameCounter;
	Interface::PixelRawByte *dataB = 0;

	XyPairInt dimensions(GetFrameDimensions());

	// The size of the destination frames must fit exactly !
	if (0 != argbFrame)
	{
		assert(dimensions == argbFrame->GetDimensions());
		argbFrameP = &(*argbFrame)[0];
	}

	if (0 != rawFrame)
	{
		assert(dimensions == rawFrame->GetDimensions());
		rawFrameP = &(*rawFrame)[0];
	}

	HandleError(Interface::GetNextFrame(argbFrameP, rawFrameP, &frameTime, dataB,  &frameCounter, &pushButton, &valid, _managerNumber));

	// Store frame time.
	if (0 != argbFrame)
		argbFrame->_timeStamp = frameTime;
	if (0 != rawFrame)
		rawFrame->_timeStamp = frameTime;

	return valid != Interface::False;
}

/// Set pixel linearization table.
void Manager::LoadPixelLinearization(const std::string &fileName)
{
	HandleError(Interface::LoadPixelLinearization(fileName.c_str(), _managerNumber ));
}

	/// Multiton access
  /// Return the Manager Instance with the specified number, starting from 0
  /// Create the instance if it doesn't exist
Manager& Manager::GetInstance(int managerNumber)
{
	static std::vector< std::tr1::shared_ptr<Manager> > Instances;

	if(managerNumber>= ((int)Instances.size()))
	{
		for(int i = ((int)Instances.size()); i < managerNumber+1 ; i++)
		{
			//the shared_ptr call CustomDeleter() automatically once they go out of scope
			std::tr1::shared_ptr<Manager> pManager( new Manager(managerNumber),CustomDeleter<Manager>() );
			pManager->_managerNumber=managerNumber;
			Instances.push_back(pManager);
		}
	}
	return *(Instances[managerNumber]); 
}

} // namespace Core
