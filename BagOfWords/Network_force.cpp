#include <opencv2\opencv.hpp>
#include <opencv2\xfeatures2d\nonfree.hpp>
#include <opencv2\features2d.hpp>
#include <opencv2/ml.hpp>

#include <iostream> 
#include <stdexcept> 
#include <algorithm>
#include <deque>
#include <mutex>

#include "Main.h"
#include "BOW_lowlevel.h"
#include "FileUtils.h"
#include "Network_force.h"


/*class Network_force
{
public:
	Network_force(::std::string svm_files);
	~Network_force();

	void setBOW(BOW_l bow);

private:
	BOW_l m_bow;

	float m_force;
	std::string m_ipaddress;

	::std::mutex m_mutex_force;

	bool networkForce(void);
	bool processImages(void);
};*/


Network_force::Network_force(::std::string svm_base_path, ::std::string image_path)
{
	m_bow.LoadFromFile(svm_base_path);
	m_running = true;
	m_force_gain = 3.0;
	m_imagepath = image_path;

	m_ipaddress = "192.168.0.12";

	m_kalman = ::cv::KalmanFilter(1,1);

	cv::setIdentity(m_kalman.measurementMatrix);

	cv::setIdentity(m_kalman.processNoiseCov, cv::Scalar::all(0.5));

	//std::cout << m_kalman.measurementMatrix << ::std::endl;
	//std::cout << m_kalman.transitionMatrix << ::std::endl;

	//std::cout << m_kalman.measurementNoiseCov << ::std::endl;
	//std::cout << m_kalman.processNoiseCov << ::std::endl;
}

Network_force::~Network_force()
{

}

void Network_force::runThreads()
{
	::std::thread t_process (&Network_force::processImages, this);
	::std::thread t_network (&Network_force::networkForce, this);
			
	t_process.join();
	t_network.join();
}

void Network_force::setForceGain(float gain)
{
	m_force_gain = gain;
}

bool Network_force::networkForce(void)
{
	/**********
	Declare and initialize connection socket
	**********/
	WSADATA wsaData;
    SOCKET ConnectSocket = INVALID_SOCKET;
    struct addrinfo *result = NULL,
                    *ptr = NULL,
                    hints;
    char *sendbuf = "this is a test";
    char recvbuf[DEFAULT_BUFLEN];
    int iResult;
    int recvbuflen = DEFAULT_BUFLEN;

	iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
    if (iResult != 0) {
        printf("WSAStartup failed with error: %d\n", iResult);
        return 1;
    }

	ZeroMemory( &hints, sizeof(hints) );
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    // Resolve the server address and port
    iResult = getaddrinfo(m_ipaddress.c_str(), DEFAULT_PORT, &hints, &result);
    if ( iResult != 0 ) {
        printf("getaddrinfo failed with error: %d\n", iResult);
        WSACleanup();
        return 1;
    }

	// Attempt to connect to an address until one succeeds
    for(ptr=result; ptr != NULL ;ptr=ptr->ai_next) {

        // Create a SOCKET for connecting to server
        ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype, 
            ptr->ai_protocol);
        if (ConnectSocket == INVALID_SOCKET) {
            printf("socket failed with error: %ld\n", WSAGetLastError());
            WSACleanup();
            return 1;
        }

        // Connect to server.
        iResult = connect( ConnectSocket, ptr->ai_addr, (int)ptr->ai_addrlen);
        if (iResult == SOCKET_ERROR) {
            closesocket(ConnectSocket);
            ConnectSocket = INVALID_SOCKET;
            continue;
        }
        break;
    }

    freeaddrinfo(result);

    if (ConnectSocket == INVALID_SOCKET) {
        printf("Unable to connect to server!\n");
        WSACleanup();
        return 1;
    }
	std::cout << "Successfully connected to server" << std::endl;

	float curForce = 0.0;

	do {

		/*****
		Receive data through the network
		*****/
        iResult = recv(ConnectSocket, recvbuf, recvbuflen, 0);


		m_mutex_force.lock();
		curForce = m_contact;
		cv::Mat prediction = m_kalman.predict();
		m_mutex_force.unlock();

		float f_prediction = prediction.at<float>(0,0);

		char test[5]; 
		sprintf(test,"%.2f",f_prediction);
		/*****
		Send back force data
		*****/
		iResult = send( ConnectSocket, test, 10, 0 );

    } while( (iResult > 0) && m_running);

    // cleanup
    closesocket(ConnectSocket);
    WSACleanup();

	::std::cout << "Network Thread exited successfully" << ::std::endl;
    return 0;
}


bool Network_force::processImages(void)
{
	::std::vector<::std::string> imList;
	int count = getImList(imList,m_imagepath);
	std::sort(imList.begin(), imList.end(), numeric_string_compare);

	std::deque<float> values(5,0);
	float average_val = 0.0;

	::std::vector<::std::string> classes = m_bow.getClasses();

	bool visualization = true;
	char key;

	for(int i=0; i<imList.size();i++)
	{
		float response = 0.0;
		std::string filepath = m_imagepath + "\\" + imList[i];

		::cv::Mat img = ::cv::imread(filepath);

		if (m_bow.predictBOW(img,response)) 
		{
			if (classes[(int) response] == "Free") response = 0.0;
			else response = 1.0;

			float popped = values.front();
			values.pop_front();
			values.push_back(response);

			average_val = (average_val*5.0 - popped + response)/5.0;

			m_mutex_force.lock();
			m_contact = average_val*m_force_gain;
			m_kalman.correct(cv::Mat(1,1,CV_32FC1,cv::Scalar(response)));
			m_mutex_force.unlock();

			if(visualization)
			{
				::cv::putText(img,cv::String(::std::to_string(average_val).c_str()),cv::Point(10,50),cv::FONT_HERSHEY_COMPLEX,1,cv::Scalar(0,255,0));
				::cv::imshow("Image", img);
				key = ::cv::waitKey(10);
				if (key==27) m_running = false;

				if (key=='c') ::cv::waitKey(0);
			}




		}
		else ::std::cout << "Error in prediction" << ::std::endl;

		if (!m_running) break;
	}
	m_running = false;
	return true;
}
