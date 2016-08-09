#pragma once

#include <mutex>

class SharedMutex
{

public:
	SharedMutex()
	{
		m_numReading = 0;
	}

	void writeLock()
	{
		m_mutex_write.lock();
	}

	void writeUnLock()
	{
		m_mutex_write.unlock();
	}

	void readLock()
	{
		m_mutex_read.lock();

		m_numReading ++;
		if (m_numReading ==1) m_mutex_write.lock();

		m_mutex_read.unlock();
	}

	void readUnLock()
	{
		m_mutex_read.lock();

		m_numReading --;
		if (m_numReading ==0) m_mutex_write.unlock();

		m_mutex_read.unlock();

	}



private:

	::std::mutex m_mutex_write;
	::std::mutex m_mutex_read;

	int m_numReading; // number of readers actually reading

};