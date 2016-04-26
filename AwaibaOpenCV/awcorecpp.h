#ifndef	__ANECPP_CORE_H__
#define __ANECPP_CORE_H__

#include <string>
#include <vector>
#include <exception>

#include <memory> //std::tr1::shared_ptr

namespace Core {

// Load interface into separate namespace to prevent naming conflicts.
namespace Interface {
#include <interface.h>
} // namespace Interface 

/// Generic pair of x and y values.
template<typename T>
struct XyPair
{
	T X, Y;

	XyPair() : X(0), Y(0) {}
	XyPair(const T &x, const T &y) : X(x), Y(y) {}
	XyPair(const XyPair<T> &rhs) : X(rhs.X), Y(rhs.Y) {}
	XyPair<T> operator=(const XyPair<T> &rhs) { X = rhs.X; Y = rhs.Y; return *this; }
};

/// Compare 2 XyPair's.
template<typename T>
bool operator==(const XyPair<T> &lhs, const XyPair<T> &rhs)
{
	return (lhs.X == rhs.X) && (lhs.Y == rhs.Y);
}

/// Used in this interface (atm).
typedef XyPair<int> XyPairInt;

/// Convert error codes back to std::exception.
void HandleError(Interface::ErrorIndex index);

/// Base class to any sensor frame source (hardware, file).
class Source
{

	Interface::SourceType _type;
	std::string _sourceName;

	// Prevent default construction and copy.
	Source();
	explicit Source(const Source &);
	
	Source &operator=(const Source &);

	/// Read information about source.
	Interface::SourceInfo GetSourceInfo(int managerNumber ) const;

	///differentiate the different camera modules when several are connected
	int _deviceId;

public:

	/// CTor used by derived classes.
	Source(Interface::SourceType type, const std::string &sourceName, int deviceId = 0) : _type(type), _sourceName(sourceName), _deviceId(deviceId) {}

public:

	/// This is a base class.
	virtual ~Source() {}

	/// Start feeding frames to the selected Manager instance.
	void Start(int managerNumber = 0) { HandleError(Interface::Start(managerNumber)); }

	/// Stop feeding frames to the selected Manager instance.
	void Stop(int managerNumber = 0) { HandleError(Interface::Stop(managerNumber)); }

	/// Pause streaming (if IsPausable()).
	void Pause(int managerNumber = 0) { HandleError(Interface::Pause(managerNumber)); }

	/// Seek to frame (if IsSeekable()).
	void Seek(int frame, int managerNumber = 0) { HandleError(Interface::Seek(frame, managerNumber)); }

	/// Can source be paused ?
	bool IsPausable(int managerNumber = 0) const { return GetSourceInfo(managerNumber).Pauseable != Interface::False; }

	/// Can source be sought ?
	bool IsSeekable(int managerNumber = 0) const { return GetSourceInfo(managerNumber).Seekable != Interface::False; }

	/// Get frame rate of source.
	int GetFps(int managerNumber = 0) const { return GetSourceInfo(managerNumber).Fps; }

	/// Get total frame number.
	int GetFrameCount(int managerNumber = 0) const { return GetSourceInfo(managerNumber).FrameCount; }

	/// Return current frame number.
	int GetCurrent(int managerNumber = 0) const { return GetSourceInfo(managerNumber).Current; }

	/// Access type of source.
	Interface::SourceType GetType() const { return _type; }

	/// Access source name.
	std::string GetSourceName() const { return _sourceName; }

	///Access source deviceId
	int getDeviceId() const { return _deviceId; }
};

/// The source is the default sensor.
class SensorSource : public Source
{

public:

	/// CTor.
  ///DeviceId are useful when several camera modules are connected, it starts from 0 and goes on
	SensorSource(int deviceId = 0) : Source(Interface::SourceSensor, "",deviceId) {}
};

/// The frame source is a file.
class FileSource : public Source
{
public:

	/// CTor. <sourceName> is the source file to stream.
	explicit FileSource(const std::string &sourceName) : Source(Interface::SourceFile, sourceName) {}
};

class PatternGeneratorSource : public Source
{
public:

	/// CTor. <sourceName> must be in format : "width:height:pattern".
	/// i.e.: "248:248:White".
	explicit PatternGeneratorSource(const std::string &sourceName) : Source(Interface::SourcePatternGenerator, sourceName) {}
};

/// Base class for any destination.
class Destination
{
	// To access _handle.
	friend class Manager;

	Interface::DestinationType _type;
	std::string _destName;

	// Prevent default construction and copy.
	Destination();
	explicit Destination(const Destination &);
	Destination &operator=(const Destination &);

	/// Read information about destination.
	Interface::DestinationInfo GetDestinationInfo(int managerNumber = 0) const;

protected:

	int _handle;

	/// CTor.
	Destination(Interface::DestinationType type, const std::string &destinationName) : 
		_type(type), _destName(destinationName) 
	{}

public:

	/// This is a base class.
	virtual ~Destination(){}

	/// Access type of destination.
	Interface::DestinationType GetType() const { return _type; }

	/// Access name of destination.
	const std::string GetDestinationName() const { return _destName; }

	/// Count of frames already captured.
	int GetFrameCount() const { return GetDestinationInfo().FrameCount; }
};

/// The destination is a file.
class FileDestination : public Destination
{
public:

	/// CTor. <destinationFileName> is the file name to write frames to.
	explicit FileDestination(const std::string &destinationName) : Destination(Interface::DestinationFile, destinationName) {}
};

/// Class used to configure the sensor.
struct Sensor
{
	/// Sensor gain value.
	int Gain;
	
	/// Sensor offset value.
	int Offset;

	/// Sensor exposure value.
	int Exposure;

	int Vref_Cds; 
	
	int Vrst_Pixel;

	/// Sensor Digipot level.
	float DigiPot;

	/// Default ctor.
	Sensor() : Gain(0), Offset(0), Exposure(0), DigiPot(1.8f) {}

	/// CTor.
	Sensor(int gain, int offset, int exposure, float digiPot) :
		Gain(gain), Offset(offset), Exposure(exposure), DigiPot(digiPot)
	{}

	/// CTor.
	Sensor(int gain, int offset, int exposure, int vref_cds, int vrst_pixel, float digiPot) :
		Gain(gain), Offset(offset), Exposure(exposure), Vref_Cds(vref_cds), Vrst_Pixel (vrst_pixel), DigiPot(digiPot)
	{}

	double DAC_DREG0;
	double DAC_DREG1;
	double DAC_DREG2;

	int DAC_DREGEN_REG;
	double DAC_DSTEP1;
	double DAC_DSTEP2;

	//Nan Eye 3A
	//First register
	int ADCGain;
	int VrstPixDAC;
	int CVCCurrent;
	int ADCMode;
	int LVDSCurrent;
	int Exposure3A;

	//Second register
	int DelayProg;
	int MCLK_DIV;

	int UpdateRegister;
	int UpdateDigipot;

	int Address;
	int LedValue;
	bool LedState;

	bool DoLineCorrectionChange;
};

/// Configure every pipe element.
struct PipeConfig
{
	struct SkipFramesType { bool Enable; int Threshold; };
	struct BadPixelReplacementType { bool Enable; int Threshold; };
	struct DeMosaicType 
	{ 
		bool Enable; 

		float PixelAdjustRed, PixelAdjustGreen1, PixelAdjustGreen2, PixelAdjustBlue;

		float RedMatrixCol0, RedMatrixCol1, RedMatrixCol2;
		float GreenMatrixCol0, GreenMatrixCol1, GreenMatrixCol2;
		float BlueMatrixCol0, BlueMatrixCol1, BlueMatrixCol2;
	};

	/// Frame skipping parameters.
	SkipFramesType SkipFrames;

	/// Bad pixel replacement parameters.
	BadPixelReplacementType BadPixelReplacement;

	/// Flag to enable / disable black level correction.
	bool BlackLevelCorrection;

	/// Flag to enable / disable white level correction.
	bool WhiteLevelCorrection;

	/// Flag to enable / disable pixel linearization.
	bool PixelLinearization;

	/// DeMosaic related parameters.
	DeMosaicType DeMosaic;

	/// Final brightness scale.
	float Brightness;

	/// CTor, fill with useful default values.
	PipeConfig() : 
		BlackLevelCorrection(false), WhiteLevelCorrection(false), PixelLinearization(false), Brightness(1)
	{
		SkipFrames.Enable = false;
		SkipFrames.Threshold = 1;
		BadPixelReplacement.Enable = false;
		BadPixelReplacement.Threshold = 1;
		DeMosaic.Enable = false;
		DeMosaic.PixelAdjustRed = 1;
		DeMosaic.PixelAdjustGreen1 = 1;
		DeMosaic.PixelAdjustGreen2 = 1;
		DeMosaic.PixelAdjustBlue = 1;
		DeMosaic.RedMatrixCol0 = 1;
		DeMosaic.RedMatrixCol1 = 0;
		DeMosaic.RedMatrixCol2 = 0;
		DeMosaic.GreenMatrixCol0 = 0;
		DeMosaic.GreenMatrixCol1 = 1;
		DeMosaic.GreenMatrixCol2 = 0;
		DeMosaic.BlueMatrixCol0 = 0;
		DeMosaic.BlueMatrixCol1 = 0;
		DeMosaic.BlueMatrixCol2 = 1;
	}
};

/// Base class of BlackLevel and WhiteLevel, most functionality
/// is shared.
template<typename T>
class BwTable
{
public:

	typedef T value_type;
	typedef typename std::vector<value_type> buffer_type;
	typedef typename buffer_type::size_type size_type;
	typedef typename buffer_type::iterator iterator;
	typedef typename buffer_type::const_iterator const_iterator;

protected:

	XyPairInt _dimensions;
	buffer_type _buffer;

	int Offset;
	int RowsInReset;

public:

	/// CTor.
	BwTable() {}

	/// CTor.
	explicit BwTable(const XyPairInt &dimensions) : _dimensions(dimensions), _buffer(_dimensions.X * _dimensions.Y * 4) {}

	/// CTor.
	BwTable(int width, int height) : _dimensions(width, height), _buffer(width * height * 4, 0) {}

	/// This is a base class, virtual DTor.
	virtual ~BwTable() {}

	/// Access table width.
	int GetWidth() const { return _dimensions.X; }

	/// Access table height.
	int GetHeight() const { return _dimensions.Y; }

	/// Access table dimensions.
	XyPairInt GetDimensions() const { return _dimensions; }

	/// Access single element.
	value_type &operator[](size_type index) { return _buffer[index]; }

	/// Access single element.
	const value_type &operator[](size_type index) const { return _buffer[index]; }

	/// Similar to begin() of STL containers.
	iterator Begin() { return _buffer.begin(); }

	/// Similar to begin() of STL containers.
	const_iterator Begin() const { return _buffer.begin(); }

	/// Similar to end() of STL containers.
	iterator End() { return _buffer.end(); }

	/// Similar to end() of STL containers.
	const_iterator End() const { return _buffer.end(); }
};

/// Manage black level tables.
class BlackLevel : public BwTable<int>
{
public:

	/// CTor.
	BlackLevel() {}

	/// CTor.
	explicit BlackLevel(const XyPairInt &dimensions) : BwTable(dimensions) {}

	/// CTor.
	BlackLevel(int width, int height) : BwTable(width, height) {}

	/// Load from file.
	void Load(const std::string &fileName);

	/// Save to file.
	void Save(const std::string &fileName) const;
};

/// Manage white level tables.
class WhiteLevel : public BwTable<float>
{
public:

	/// CTor.
	WhiteLevel() {}

	/// CTor.
	explicit WhiteLevel(const XyPairInt &dimensions) : BwTable(dimensions) {}

	/// CTor.
	WhiteLevel(int width, int height) : BwTable(width, height) {}

	/// Load from file.
	void Load(const std::string &fileName);

	/// Save to file.
	void Save(const std::string &fileName) const;
};

/// Various frame color spaces.
enum ColorSpace { Undefined, Raw10bit, Argb };

/// Store frame data.
template<typename T, ColorSpace colorSpace>
class Frame
{
	friend class Manager;

public:

	typedef typename std::vector<T> buffer_type;
	typedef typename buffer_type::size_type size_type;
	typedef typename buffer_type::iterator iterator;
	typedef typename buffer_type::const_iterator const_iterator;
	typedef unsigned __int64 time_type;

private:

	XyPairInt _dimensions;
	buffer_type _buffer;
	time_type _timeStamp;

public:

	/// CTor.
	Frame(int width, int height) :
		_dimensions(width, height), _buffer(width * height, 0), _timeStamp(0)
	{}

	/// CTor.
	Frame(int width, int height, const buffer_type &source, time_type _timeStamp) :
		_dimensions(width, height), _buffer(source), _timeStamp(timeStamp)
	{}

	/// CTor.
	explicit Frame(const XyPairInt &dimensions) :
		_dimensions(dimensions), _buffer(dimensions.X * dimensions.Y, 0), _timeStamp(0)
	{}

	/// CTor.
	Frame(const XyPairInt &dimensions, const buffer_type &source, time_type _timeStamp) :
		_dimensions(dimensions), _buffer(source), _timeStamp(timeStamp)
	{}

	/// Access frame width.
	int GetWidth() const { return _dimensions.X; }

	/// Access frame height.
	int GetHeight() const { return _dimensions.Y; }

	/// Access frame dimensions.
	XyPairInt GetDimensions() const { return _dimensions; }

	/// Access frame time stamp.
	time_type GetTimeStamp() const { return _timeStamp; }

	/// Access single element.
	T &operator[](size_type index) { return _buffer[index]; }

	/// Access single element.
	const T &operator[](size_type index) const { return _buffer[index]; }

	/// Similar to begin() of STL containers.
	iterator Begin() { return _buffer.begin(); }

	/// Similar to begin() of STL containers.
	const_iterator Begin() const { return _buffer.begin(); }

	/// Similar to end() of STL containers.
	iterator End() { return _buffer.end(); }

	std::vector<T> GetData() { return _buffer; }

	/// Similar to end() of STL containers.
	const_iterator End() const { return _buffer.end(); }
};

/// Stores raw sensor data.
typedef Frame<unsigned short, Raw10bit> RawFrame;

/// Stores ARGB color frame data.
typedef Frame<unsigned int, Argb> ArgbFrame;


template<class T>
class CustomDeleter
{
    ///public deleter, called internally by std::shared_ptr to clean the camera instances
public:
    void operator () (T* pManager) const
    {
        delete pManager;
    }
};

/// The main class, all communication is in here.
class Manager 
{
    friend class CustomDeleter<Manager>;

	/// CTor.
	Manager(int managerNumber);

	

    ///allow to control several modules
	int _managerNumber;

    ///vector storing the manager multiton
  //static std::vector< std::tr1::shared_ptr<Manager> > Instances;

public:

	/// DTor.
	~Manager() { Interface::ManagerDeInit(_managerNumber); }

	/// Set new source for the pipe, any previous defined source is stopped
	/// and replaced with the new one.
	void SetSource(const Source &source);

	void SetSensor(int id);

	void SetBinFile(char* fileName);

	/// Send configuration values to sensor (if source is already set to sensor).
	void ConfigureSensor(const Sensor &sensor);

	/// Add raw frame destination.
	void AddDestination(Destination &destination);

	/// Finish running destination.
	void FinishDestination(const Destination &destination);

	/// Set new pipe parameters, can be done on an active pipe too.
	void SetPipeConfig(const PipeConfig &pipeConfig);

	/// Set black level correction table.
	void SetBlackLevel(BlackLevel &table);

	/// Set white level correction table.
	void SetWhiteLevel(WhiteLevel &table);

	/// Get frame dimensions of current pipe configuration.
	XyPairInt GetFrameDimensions() const;

	/// Read next frame from source. Returns immediately (and true if frames are really read and valid).
	/// Both parameters can be NULL, only valid structures are updated.
	bool GetNextFrame(ArgbFrame *argbFrame, RawFrame *rawFrame);

	/// Set pixel linearization table.
	void LoadPixelLinearization(const std::string &fileName);

	/// Multiton access
	/// Return the Manager Instance with the specified number, starting from 0
	/// Create the instance if it doesn't exist
	static Manager &GetInstance(int managerNumber = 0);

	///return manager instance number
	int GetManagerNumber() const { return _managerNumber;}

	void SetLineCorrection(const Sensor &config, bool state);

	int GetNumberDevices() { return Interface::NumberDevices(); }

	void SetFPGAData(unsigned int address, unsigned int value);
};

} // namespace Core 

#endif // __ANECPP_CORE_H__
