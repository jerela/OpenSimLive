#pragma once

#include <iostream>
#include <thread>
#include <iomanip>
#include <sstream>
#include <set>
#include <list>

#include <xsensdeviceapi.h>
#include "conio.h" // for non-ANSI _kbhit() and _getch()
#include <xsens/xsmutex.h>



// \brief Stream insertion operator overload for XsPortInfo
std::ostream& operator << (std::ostream& out, XsPortInfo const& p);

// \brief Stream insertion operator overload for XsDevice
std::ostream& operator << (std::ostream& out, XsDevice const& d);




//----------------------------------------------------------------------
// Callback handler for wireless master
//----------------------------------------------------------------------
class WirelessMasterCallback : public XsCallback
{
public:
	typedef std::set<XsDevice*> XsDeviceSet;

	XsDeviceSet getWirelessMTWs() const
	{
		XsMutexLocker lock(m_mutex);
		return m_connectedMTWs;
	}

protected:
	virtual void onConnectivityChanged(XsDevice* dev, XsConnectivityState newState)
	{
		XsMutexLocker lock(m_mutex);
		switch (newState)
		{
		case XCS_Disconnected:		/*!< Device has disconnected, only limited informational functionality is available. */
			std::cout << "\nEVENT: MTW Disconnected -> " << *dev << std::endl;
			m_connectedMTWs.erase(dev);
			break;
		case XCS_Rejected:			/*!< Device has been rejected and is disconnected, only limited informational functionality is available. */
			std::cout << "\nEVENT: MTW Rejected -> " << *dev << std::endl;
			m_connectedMTWs.erase(dev);
			break;
		case XCS_PluggedIn:			/*!< Device is connected through a cable. */
			std::cout << "\nEVENT: MTW PluggedIn -> " << *dev << std::endl;
			m_connectedMTWs.erase(dev);
			break;
		case XCS_Wireless:			/*!< Device is connected wirelessly. */
			std::cout << "\nEVENT: MTW Connected -> " << *dev << std::endl;
			m_connectedMTWs.insert(dev);
			break;
		case XCS_File:				/*!< Device is reading from a file. */
			std::cout << "\nEVENT: MTW File -> " << *dev << std::endl;
			m_connectedMTWs.erase(dev);
			break;
		case XCS_Unknown:			/*!< Device is in an unknown state. */
			std::cout << "\nEVENT: MTW Unknown -> " << *dev << std::endl;
			m_connectedMTWs.erase(dev);
			break;
		default:
			std::cout << "\nEVENT: MTW Error -> " << *dev << std::endl;
			m_connectedMTWs.erase(dev);
			break;
		}
	}
private:
	mutable XsMutex m_mutex;
	XsDeviceSet m_connectedMTWs;
};


//----------------------------------------------------------------------
// Callback handler for MTw
// Handles onDataAvailable callbacks for MTW devices
//----------------------------------------------------------------------
class MtwCallback : public XsCallback
{
public:
	MtwCallback(int mtwIndex, XsDevice* device)
		:m_mtwIndex(mtwIndex)
		, m_device(device)
	{}

	bool dataAvailable() const
	{
		XsMutexLocker lock(m_mutex);
		return !m_packetBuffer.empty();
	}

	XsDataPacket const* getOldestPacket() const
	{
		XsMutexLocker lock(m_mutex);
		XsDataPacket const* packet = &m_packetBuffer.front();
		return packet;
	}

	void deleteOldestPacket()
	{
		XsMutexLocker lock(m_mutex);
		m_packetBuffer.pop_front();
	}

	int getMtwIndex() const
	{
		return m_mtwIndex;
	}

	XsDevice const& device() const
	{
		assert(m_device != 0);
		return *m_device;
	}

protected:
	virtual void onLiveDataAvailable(XsDevice*, const XsDataPacket* packet)
	{
		XsMutexLocker lock(m_mutex);
		// NOTE: Processing of packets should not be done in this thread.

		m_packetBuffer.push_back(*packet);
		if (m_packetBuffer.size() > 300)
		{
			std::cout << std::endl;
			deleteOldestPacket();
		}
	}

private:
	mutable XsMutex m_mutex;
	std::list<XsDataPacket> m_packetBuffer;
	int m_mtwIndex;
	XsDevice* m_device;
};






// ACTUAL XSENSDATAREADER CLASS BEGINS HERE
namespace OpenSimLive {

	class XsensDataReader {

	public:
		// CONSTRUCTORS AND DECONSTRUCTORS
		XsensDataReader();
		~XsensDataReader();

		// PUBLIC METHODS
		void InitiateStartupPhase(); // starts the connection from scratch
		std::vector<XsQuaternion> XsensDataReader::GetQuaternionData(std::vector<XsQuaternion>& quaternionData); // returns IMU orientations as quaternions
		std::vector<XsEuler> XsensDataReader::GetEulerData(std::vector<XsEuler>& eulerData); // returns IMU orientations as Euler angles
		void CloseConnection(); // shuts down the connection
		std::vector<MtwCallback*> GetMtwCallbacks() { return mtwCallbacks_; } // returns a pointer to mtwCallbacks (returns mtwCallbacks_, which is a pointer itself)
		void SetDesiredUpdateRate(int rate) { desiredUpdateRate_ = rate; } // sets the desired orientation measurement frequency
		void SetDesiredRadioChannel(int channel) { desiredRadioChannel_ = channel; } // sets the desired radio channel
		bool GetNewDataAvailable() { return newDataAvailable_; } // returns true if new data is available since the last time we used a getter function for orientations

	protected:
			
	private:
		// PRIVATE METHODS
		int findClosestUpdateRate(const XsIntArray& supportedUpdateRates, const int desiredUpdateRate); // Given a list of update rates and a desired update rate, returns the closest update rate to the desired one
		

		// PRIVATE VARIABLES
		WirelessMasterCallback wirelessMasterCallback_; // Callback for wireless master		
		std::vector<MtwCallback*> mtwCallbacks_; // Callbacks for MTw devices
		int desiredUpdateRate_ = 75; // orientation measurement frequency in hertz
		int desiredRadioChannel_ = 19; // use radio channel 19 for wireless master
		XsDevicePtrArray mtwDevices_;
		XsDevicePtr wirelessMasterDevice_;
		XsControl* control_;
		bool newDataAvailable_; // boolean that indicates if new data is available since the last time we used a getter function to retrieve IMU orientations

	}; // end of class
}