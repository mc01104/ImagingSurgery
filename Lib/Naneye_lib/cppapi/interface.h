#include <stdint.h>

#ifndef	__ANE_INTERFACE_H__
#define __ANE_INTERFACE_H__

// DLL import/__declspec(dllexport) macro.
// DLL import/export macro.
#ifdef	WIN32
#	ifdef	__BUILD_CORE_DLL__
#		define	DLLEXPORT	__declspec(dllexport) _cdecl
#	else
#		define	DLLEXPORT	__declspec(dllimport) _cdecl
#	endif
#else
#	define DLLEXPORT
#endif

#ifdef LINUX
#	define __int64 long long
#endif

// Use flat C interface.
#ifdef	__cplusplus
extern "C" {
#endif

/// Type of error return code. If non-0, this code can be used one time to
/// retrieve an error string using GetErrorText().
typedef int ErrorIndex;

/// Type of source, possible values defined below.
typedef int SourceType;

/// Data source is the hardware (sensor).
const SourceType SourceSensor = 0;

/// Data source is raw file.
const SourceType SourceFile = 1;

/// Data source is a pattern generator.
const SourceType SourcePatternGenerator = 2;

/// Raw image destination (except the pipe). Used to transfer
/// raw images to another location, i.e. store to file.
typedef int DestinationType;

/// Data destination is a file.
const DestinationType DestinationFile = 0;

/// C has no support for 'bool', use this instead.
typedef int Boolean;
const Boolean False = 0;
const Boolean True = 1;

/// Type used to store time for each frame (milliseconds from startup).
typedef 
#ifdef	WIN32 
signed __int64
#else
	long long
#endif
 FrameTime;

/// Pixel value type for ARGB (Alpha/Red/Green/Blue) frame data.
typedef	unsigned int PixelArgb;

/// Pixel value type for raw sensor data (10 bit valid).
typedef unsigned short PixelRaw;

/// Pixel value type for raw sensor data (8 bit valid).
typedef unsigned char PixelRawByte;

const size_t MaxSourceNameLen = 1024;
const size_t MaxDestinationNameLen = 1024;
const size_t LogBufferLength = 1024;

/// Must match on both sides.
const int InterfaceVersion = 0x03000000;

// Pack structures byte-wise.
#pragma	pack(1)

/// Specify source for pipe (sensor, file, ...)
struct SourceConfig
{
	/// Type of source, currently either SourceSensor or SourceFile.
	SourceType Type;

	/// In case (Type==SourceSensor), this is ignored.
	/// For (Type==SourceFile), this is the filename to use a source.
	char SourceName[MaxSourceNameLen];

	///differentiate the different modules when several cameras are connected
	int deviceId;

	static std::string filePath;
};

/// Information about the current source (updated cyclically).
struct SourceInfo
{
	/// Same as in struct SourceConfig.
	SourceType Type;

	/// Same as in struct SourceConfig.
	char SourceName[MaxSourceNameLen];

	/// The source can be pause, currently only possible with file source.
	Boolean Pauseable;

	/// The source can be sought, currently only possible with file source.
	Boolean Seekable;

	/// The source's frames per second.
	int Fps;

	/// Count of frames in source (only applicable to file sources).
	int FrameCount;

	/// Current frame.
	int Current;
};

/// Specify destination of raw frames (to store in video file).
struct DestinationConfig
{
	/// Must be DestinationFile.
	DestinationType Type;

	/// File name to store to.
	char DestinationName[MaxDestinationNameLen];
};

/// Information about destination (updated cyclically).
struct DestinationInfo
{
	/// Same as in DestinationConfig.
	DestinationType Type;

	/// Same as in DestinationConfig.
	char DestinationName[MaxDestinationNameLen];

	/// Already captured frames.
	int FrameCount;
};

/// Core configuration for pixel processing pipe.
/// All values can be updated while the pipe is running, unchanged values must be preserved.
struct PipeConfig
{
	/// Flag to enable or disable skipping of frames 
	/// (if pipe processing is too slow).
	Boolean SkipFrames;

	/// Flag to enable or disable bad pixel replacement.
	Boolean BadPixelReplacement;

	/// Flag to enable or disable black level correction.
	Boolean BlackLevelCorrection;

	/// Flag to enable or disable white level correction.
	Boolean WhiteLevelCorrection;

	/// Flag to enable or disable pixel linearization.
	Boolean PixelLinearization;

	/// Flag to enable or disable DeMosaic of frame data.
	Boolean DeMosaic;

	/// Count of frames to hold in any case in pipe frame buffer.
	/// If skipping is enabled and the limit is reached,
	/// the source drops new incoming frames.
	int SkippingThreshold;

	/// The threshold value to use in BPR (if enabled).
	int BadPixelReplacementThreshold;

	/// Pixel adjustment Pre-DeMosaic.
	float PixelAdjustRed, PixelAdjustGreen1, PixelAdjustGreen2, PixelAdjustBlue;

	/// RGB adjustment Matrix Post-DeMosaic - Red Row.
	float RedMatrixCol0, RedMatrixCol1, RedMatrixCol2;

	/// RGB adjustment Matrix Post-DeMosaic - Green Row.
	float GreenMatrixCol0, GreenMatrixCol1, GreenMatrixCol2;

	/// RGB adjustment Matrix Post-DeMosaic - Blue Row.
	float BlueMatrixCol0, BlueMatrixCol1, BlueMatrixCol2;

	/// Final brightness adjustment of frame.
	float Brightness;
};

/// Struct to store values that can be sent to sensor 
/// (hardware configuration).
struct Sensor
{
	double DAC_DREG0;
    double DAC_DREG1;
    double DAC_DREG2;

    int DAC_DREGEN_REG;
    double DAC_DSTEP1;
    double DAC_DSTEP2;

	/// Gain value (0 - 3).
	int Gain;

	/// Offset value (0 - 3).
	int Offset;

	/// Exposure value (0 - 255).
	int Exposure;

	/// Digipot value (1.8 - 2.4 V).
	double DigiPot;

	//Value between 0-3
	int Vrst_pixel;

	//Value between 0-3
	int Vref_cds;

	//this is gonna be actualized to the nan eye 3A:

	///ADC Gain (0-3)
	int ADCGain;

	/// Vreset_pixel (0-3)
    int VrstPixDAC;

	///CVCCurrent (0-3)
    int CVCCurrent;

	///ADC Mode (0-1)
    int ADCMode;

	///LVDS Current(0-3)
    int LVDSCurrent;

	/// Exposure value for Nan Eye 3A (0 - 127).
    int Exposure3A;

    //Second register

	/// Delay prog (0-127)
    int DelayProg;

	///Mulk_Div (0-1)
    int MCLK_DIV;

	int UpdateRegister;

	int UpdateDigipot;

	int Address;

	//Between 0 and 4095
	int LedValue;

	bool LedState;

	bool DoLineCorrectionChange;
};

// Default packing.
#pragma pack()

int DLLEXPORT NumberDevices();

/// Return version of interface. This must match to guarantee 
/// compatible structure format.
int DLLEXPORT GetDllVersion();

/// Each of the following functions return a value of type ErrorIndex. If this value is not 0, an error
/// has occurred. The return code can then be used here to get the error text. After successful fetching the text,
/// the error is removed from the internal error list. 
/// <errorBuffer> will receive the error text (if large enough, indicated by <errorBufferLength>).
/// The required buffer size is returned, in case the buffer is not large enough or <errorBuffer> is 0.
/// Successful fetching is indicated by returning 0.
int DLLEXPORT GetErrorText(ErrorIndex errorIndex, char *errorBuffer, int errorBufferLength);

/// Init API. Must be called in front of all other functions.
ErrorIndex DLLEXPORT ManagerInit(int ManagerNumber);

ErrorIndex DLLEXPORT SetSensor(int id, int managerNumber);

ErrorIndex DLLEXPORT SetBinFile(char* file, int managerNumber);

ErrorIndex DLLEXPORT ActivateCorrectionMode(unsigned int value, int managerNumber);

/// De-Init the API. Clean up internal resources, no other functions must be called
/// after this (and before calling Init() again).
ErrorIndex DLLEXPORT ManagerDeInit(int managerNumber);

/// Define source of pipe. Only one source can be active at one time. Calling this again will
/// remove the previous source.
ErrorIndex DLLEXPORT SetSource(SourceConfig *config, int managerNumber);

/// Offer information about the current source, can be called cyclically.
ErrorIndex DLLEXPORT GetSourceInfo(SourceInfo *info, int managerNumber);

/// Write the config values to the sensor. Only possible if sensor is active source.
ErrorIndex DLLEXPORT ConfigureSensor(Sensor *config, int managerNumber);

/// Start current source to push frames into the pipe frame buffer.
ErrorIndex DLLEXPORT Start(int managerNumber);

/// Stop the current source to output frames.
ErrorIndex DLLEXPORT Stop(int managerNumber);

/// Pause current source (if possible).
ErrorIndex DLLEXPORT Pause(int managerNumber);

/// Seek to given frame in source (if possible).
ErrorIndex DLLEXPORT Seek(int frame, int managerNumber);

/// Add a new destination for raw frames. Currently this can be a file. 
/// <handle> returns a numeric handle that can be used to gather information (GetDestinationInfo())
/// and to stop using this destination (FinishDestination()).
ErrorIndex DLLEXPORT AddDestination(int *handle, DestinationConfig *config, int managerNumber);

/// Get information about the destination (which is updated cyclically). 
/// <handle> is used to indicate the destination (returned by AddDestination()).
ErrorIndex DLLEXPORT GetDestinationInfo(int handle, DestinationInfo *info, int managerNumber);

/// Stop feeding frames to destination.
/// <handle> is used to indicate the destination (returned by AddDestination()).
ErrorIndex DLLEXPORT FinishDestination(int handle, int managerNumber);

/// Update all pipe relevant configuration parameters. This can be called even if
/// the pipe is active to modify pipe processing.
ErrorIndex DLLEXPORT SetPipeConfig(PipeConfig *config, int managerNumber);

/// Set table to use for black level correction. <width> and <height> must match
/// the current frame dimensions (after cutting!). Each table value is added to
/// its corresponding pixel (and saturated to 10 bit).
ErrorIndex DLLEXPORT SetBlackLevelTable(int width, int height, int *table, int managerNumger);

/// Set table to use for white level correction. <width> and <height> must match
/// the current frame dimensions (after cutting!). Each table value is multiplied (8-bit fixed point arithmetic) to
/// its corresponding pixel.
ErrorIndex DLLEXPORT SetWhiteLevelTable(int width, int height, int *table, int managerNumber);

//ErrorIndex DLLEXPORT SetPixelLinearizationTable(int width, int height, PixelRaw *source, PixelRaw *dest);

/// Return dimensions of resulting frame. All buffer sizes used in the calls below must
/// be designed using this values.
ErrorIndex DLLEXPORT GetFrameDimensions(int *width, int *height, int managerNumber);

/// Pull next frame from pipe buffer. <pixels> can be 0, the whole processing pipe is ignored in this case.
/// This function returns immediately, even if no frame is available. <valid> is True if data is valid.
ErrorIndex DLLEXPORT GetNextFrame(PixelArgb *pixels, PixelRaw *rawPixels, FrameTime *frameTime, PixelRawByte* rawPixelsByte, int8_t* frameCounter, int8_t* pushButton, Boolean *valid, int managerNumber);
/// Read dimensions of black level correction file.
ErrorIndex DLLEXPORT LoadBlackLevelDimensions(const char *fileName, int *width, int *height);
/// Read table from black level correction file, table must be large enough to hold a table.
ErrorIndex DLLEXPORT LoadBlackLevel(Sensor* sensor, const char *fileName, int width, int height, int *table);
/// Store black level correction table into file.
ErrorIndex DLLEXPORT SaveBlackLevel(Sensor* sensor, const char *fileName, int width, int height, int *table);

/// Read dimensions of white level correction file.
ErrorIndex DLLEXPORT LoadWhiteLevelDimensions(const char *fileName, int *width, int *height);
/// Read table from white level correction file, table must be large enough to hold a table.
ErrorIndex DLLEXPORT LoadWhiteLevel(Sensor* sensor, const char *fileName, int width, int height, int *table);
/// Store white level correction table into file.
ErrorIndex DLLEXPORT SaveWhiteLevel(Sensor* sensor, const char *fileName, int width, int height, int *table);

/// Load pixel linearization data to current pipe config.
ErrorIndex DLLEXPORT LoadPixelLinearization(const char *fileName, int managerNumber);

ErrorIndex DLLEXPORT SetLineCorrection(Sensor *config, bool state, int managerNumber);

ErrorIndex DLLEXPORT SetFPGAData(unsigned int address, unsigned int value, int managerNumber);

int DLLEXPORT GetFpgaSensorAddress(int managerNumber);

#ifdef	__cplusplus
} // extern "C"
#endif

#endif // __ANE_INTERFACE_H__