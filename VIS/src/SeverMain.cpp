#include "pch.h"
#include "RestEndpoints.h"
#include "Routing.h"

using namespace web;
using namespace http;
using namespace experimental;
using namespace listener;

const int DEFAULT_PORT = 8080;

///
/// Sleep the thread until we are signaled to be interrupted or terminated
///
void sleepUntilKilled()
{
	static std::mutex interruptMutex;
	static std::condition_variable interrupted;

	signal(SIGINT, [](int) { interrupted.notify_one(); });
	signal(SIGTERM, [](int) { interrupted.notify_one(); });

	std::unique_lock<std::mutex> interruptLock(interruptMutex);
	interrupted.wait(interruptLock);
}


int main(int argc, char *argv[])
{
	int port = (argc > 1)
		? boost::lexical_cast<int>(argv[1])
		: DEFAULT_PORT;
	auto listenUri = uri_builder()
		.set_scheme(L"http")
		.set_host(L"127.0.0.1")
		.set_port(port)
		.to_uri();

	experimental::listener::http_listener listener(listenUri);
	listener.open().wait();
	listener.support(vis::RequestRouter(
	{
		vis::HttpRoute(methods::GET, L"/object-scan", &vis::scanObject),
		vis::HttpRoute(methods::GET, L"/mesh-file/all", &vis::listMeshFiles),
		vis::HttpRoute(methods::GET, L"/mesh-file", &vis::downloadMeshFile)
	}));

	auto uriString = listenUri.to_string();
	std::cout << "Now listening on " << std::string(uriString.begin(), uriString.end()) << std::endl;

	sleepUntilKilled();
	listener.close().wait();

	return EXIT_SUCCESS;
}

