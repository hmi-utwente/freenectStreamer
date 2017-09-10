#include <iostream>
#include <asio.hpp>
#include <thread>
#include <chrono>
#include <cstdlib>
#include <queue>
#include <string> 
#include "json.hpp"

// for convenience
using json = nlohmann::json;

using asio::ip::udp;
using namespace std::chrono;

class IMPRESS_UDPClient
{
public:
	IMPRESS_UDPClient(asio::io_service& io_service) : io_service_(io_service), socket_(io_service, udp::endpoint(udp::v4(), 0)) {
		
	}

	void init(std::string _socketID, bool _isSender, const std::string& host, const std::string& port) {
		socketID = _socketID;
		isSender = _isSender;

		udp::resolver resolver(io_service_);
		udp::resolver::query query(udp::v4(), host, port);
		udp::resolver::iterator iter = resolver.resolve(query);
		serverEndpoint_ = *iter;

		localIP = getLocalIP();
		_listenThread = std::thread(&IMPRESS_UDPClient::DataListener, this);
		_updateThread = std::thread(&IMPRESS_UDPClient::update, this);
	}

	void update() {
		_updateRunning = true;
		while (_updateRunning) {
			milliseconds currentTime = duration_cast<milliseconds>(system_clock::now().time_since_epoch());

			if (connected && currentTime > lastReceivedHB + connectionTimeout) {
				connected = false;
			}

			if (!connected && currentTime>lastRegister+registerInterval) {
				Register();
				lastRegister = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
			}

			if (connected) {
				if (currentTime > lastSentHB + HBInterval) {
					lastSentHB = currentTime;
					Punch();
				}
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
		}
		_updateRunning = false;
	}

	void SendData(unsigned char* data, size_t size) {
		if (connected) {
			socket_.send_to(asio::buffer(data, size), clientEndpoint_);
		}
	}

	void SendData(char* data, size_t size) {
		if (connected) {
			socket_.send_to(asio::buffer(data, size), clientEndpoint_);
		}
	}

	void exit() {
		_listenRunning = false;
		_updateRunning = false;
		_listenThread.join();
		_updateThread.join();
	}

	~IMPRESS_UDPClient()
	{
		socket_.close();
	}

	std::string getLocalIP() {
		std::string _localIP = "noLocalIP";
		try {
			asio::io_service netService;
			udp::resolver   resolver(netService);
			udp::resolver::query query(udp::v4(), "8.8.8.8", "");
			udp::resolver::iterator endpoints = resolver.resolve(query);
			udp::endpoint ep = *endpoints;
			udp::socket socket(netService);
			socket.connect(ep);
			asio::ip::address addr = socket.local_endpoint().address();
			_localIP = addr.to_string();
		}
		catch (std::exception& e) {
			std::cerr << "Could not deal with socket. Exception: " << e.what() << std::endl;
		}
		return _localIP;
	}

	void Register() {
		std::stringstream ss;
		ss << "d{\"packageType\":\"register\",\"socketID\":\"" << socketID << "\",\"isSender\":" << (std::string)(isSender ? "true" : "false") << ",\"localIP\":\"" << localIP << "\"}";
		std::string json = ss.str();
		_sendData(json, serverEndpoint_);
	}

	void Punch() {
		std::string data = "d{\"type\":\"punch\"}";
		_sendData(data, clientEndpoint_);
	}

private:
	asio::io_service& io_service_;
	udp::socket socket_;
	udp::endpoint serverEndpoint_;
	udp::endpoint clientEndpoint_;

	std::string localIP;
	std::string socketID;
	bool isSender;
	bool connected = false;

	std::thread _listenThread;
	bool _listenRunning = false;
	std::thread _updateThread;
	bool _updateRunning = false;

	milliseconds lastRegister = (milliseconds)0;
	milliseconds registerInterval = (milliseconds)2000;
	milliseconds lastSentHB = (milliseconds)0;
	milliseconds HBInterval = (milliseconds)2000;
	milliseconds lastReceivedHB = (milliseconds)0;
	milliseconds connectionTimeout = (milliseconds)5000;

	void _sendData(const std::string& msg, udp::endpoint endpoint) {
		try {
			socket_.send_to(asio::buffer(msg, msg.size()), endpoint);
		}catch(const std::exception& e){
			std::cout << "exception while sending" << std::endl;
		}
	}

	void DataListener() {
		const std::size_t size = 65536;
		char data[size];
		_listenRunning = true;
		while (_listenRunning) {
			try {
				int len = socket_.receive(asio::buffer(data, size));
				HandleReceivedData(data, len);
			}
			catch (std::exception& e) {
				//std::cerr << "Exception while receiving: " << e.what() << std::endl;
			}
		}
		_listenRunning = false;
    }

	void HandleReceivedData(char* data, int len) {
		char magicByte = data[0];

		if (magicByte == 100) {
			json j = json::parse(&data[1], &data[len]);
			std::cout << j << std::endl;
			if (j["type"] == "answer") {
				std::string host = j.at("address").get<std::string>();
				std::string port = std::to_string(j.at("port").get<int>());

				udp::resolver resolver(io_service_);
				udp::resolver::query query(udp::v4(), host, port);
				udp::resolver::iterator iter = resolver.resolve(query);
				clientEndpoint_ = *iter;

				Punch();
				Punch();
			}
			else if (j["type"] == "punch") {
				lastReceivedHB = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
				connected = true;
			}
		}
	}
};

//int main()
//{
//	asio::io_service io_service;
//	IMPRESS_UDPClient client(io_service, "test", false, "impress.mooo.com", "6312");
//
//	while (true) {
//
//	}
//	//client.exit();
//}