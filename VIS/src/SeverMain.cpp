#include "pch.h"
#include "FileStorage.h"
#include "ServerCommon.h"
#include "Visualization.h"

using namespace vis;
using namespace web;
using namespace http;
using namespace experimental;
using namespace listener;

namespace opts = boost::program_options;

const int DEFAULT_PORT = 8080;

/// Sleep the thread until we are signaled to be interrupted or terminated
void sleepUntilKilled()
{
	static std::mutex interruptMutex;
	static std::condition_variable interrupted;

	signal(SIGINT, [](int) { interrupted.notify_one(); });
	signal(SIGTERM, [](int) { interrupted.notify_one(); });

	std::unique_lock<std::mutex> interruptLock(interruptMutex);
	interrupted.wait(interruptLock);
}


/// Create a context based off of command line arguments
/// @param commandArgs the command line arguments
DeviceContext createCtx(const opts::variables_map& commandArgs)
{
	if (commandArgs.count("oni"))
	{
		try
		{
			auto spCamera = MockCamera::make(commandArgs["oni"].as<std::string>(), StructureSensor::cameraIntrinsics());
			auto spControls = std::make_shared<MockPlatformControls>(spCamera);
			return DeviceContext(spCamera, spControls);
		}
		catch (...)
		{
			std::cerr << "Error: Invalid path" << std::endl;
			sleepUntilKilled();
			std::exit(EXIT_FAILURE);
		}
	}
	else if (commandArgs.count("com"))
	{
		try
		{
			std::stringstream comPort;
			comPort << "\\\\.\\COM" << commandArgs["com"].as<int>();
			auto spControls = std::make_shared<ServoPlatformControls>(comPort.str());

			auto spCamera = StructureSensor::acquireCamera();
			return DeviceContext(std::move(spCamera), spControls);
		}
		catch (const std::exception& ex)
		{
			std::cerr << "Error: Could not connect to Camera or Platform" << std::endl;
			std::cerr << "Exception: " << ex.what() << std::endl;
			sleepUntilKilled();
			std::exit(EXIT_FAILURE);
		}
	}
	else
	{
		std::cerr << "--com option must be present when using a physical camera";
		std::exit(EXIT_FAILURE);
	}
}


/// Check that arguments are valid, parsing them or killing the app
/// @param argc arg count
/// @param argv argument values
opts::variables_map checkArgs(int argc, char *argv[])
{
	opts::options_description description("Allowed Options");
	description.add_options()
		("oni",   opts::value<std::string>(),  "Path to an oni file to use as camera data")
		("port", opts::value<int>(), "The HTTP port to listen on")
		("com", opts::value<int>(), "The COM port number to use for platform rotation, unneeded if using a file")
		("visualizer", "Display a visualizer while doing scans");

	try
	{
		opts::variables_map args;
		opts::store(opts::parse_command_line(argc, argv, description), args);
		opts::notify(args);
		return args;
	}
	catch (const opts::error&)
	{
		std::cout << description << std::endl;
		std::exit(EXIT_FAILURE);
	}	
}


int main(int argc, char *argv[])
{
	// Init and create context from args
	if (openni::OpenNI::initialize() != openni::STATUS_OK)
	{
		std::cerr << "Could not initialize OpenNI: " << openni::OpenNI::getExtendedError() << std::endl;
		return EXIT_FAILURE;
	}
	initFileStorage();

	auto args = checkArgs(argc, argv);
	auto ctx = createCtx(args);
	if (args.count("visualizer"))
		ProgressVisualizer::get()->start();

	// Start the server
	int port = args.count("port")
		? args["port"].as<int>()
		: DEFAULT_PORT;
	auto listenUri = uri_builder()
		.set_scheme(L"http")
		.set_host(L"127.0.0.1")
		.set_port(port)
		.to_uri();

	http_listener listener(listenUri);
	listener.open().wait();
	listener.support(AppServer(&ctx));

	auto uriString = listenUri.to_string();
	std::cout << "Now listening on " << std::string(uriString.begin(), uriString.end()) << std::endl;

	sleepUntilKilled();
	listener.close().wait();

	return EXIT_SUCCESS;
}
