/**
@brief Linux Socket for Async Aquisition
@author zhu-ty
@date Jan 18, 2019
*/

#ifndef __GENERIC_CAMERA_DRIVER_LINUX_SOCKET_H__
#define __GENERIC_CAMERA_DRIVER_LINUX_SOCKET_H__

// include std
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <queue>
#include <thread>
#include <memory>
// socket, linux only
#ifndef WIN32
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#endif 
class LxSoc
{
public:
	LxSoc() {};
	~LxSoc() {};
#ifndef WIN32
	int init(int port)
	{
		// init socket
		sockfd = socket(AF_INET, SOCK_STREAM, 0);
		if (sockfd < 0)
			error("ERROR opening socket");
		bzero((char *)&serv_addr, sizeof(serv_addr));
		portno = port;
		serv_addr.sin_family = AF_INET;
		serv_addr.sin_addr.s_addr = INADDR_ANY;
		serv_addr.sin_port = htons(portno);
		if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
			error("ERROR on binding");
		listen(sockfd, 5);
	}
	int waitFor(std::string command)
	{
		int ret = 0;
		printf("Waiting for action command !\n");
		clilen = sizeof(cli_addr);
		newsockfd = accept(sockfd,
			(struct sockaddr *) &cli_addr,
			&clilen);
		if (newsockfd < 0)
			error("ERROR on accept");
		bzero(buffer, 256);
		n = read(newsockfd, buffer, 255);
		if (n < 0) error("ERROR reading from socket");
		if (strcmp(buffer, command.c_str()) == 0) {
			printf("Here is the message: %s\n", buffer);
			ret = 1;
		}
		else {
			printf("Wrong command ! Here is the message: %s\n", buffer);
			ret = -1;
		}
		close(newsockfd);
		close(sockfd);
		return ret;
	}
private:
	int sockfd, newsockfd, portno;
	socklen_t clilen;
	char buffer[256];
	struct sockaddr_in serv_addr, cli_addr;
	int n;
	void error(const char *msg) 
	{
		perror(msg);
		exit(1);
	}
#else
	int init(int port)
	{
		return 0;
	}
	int waitFor(std::string command)
	{
		return 1;
	}
#endif

};

#endif //__GENERIC_CAMERA_DRIVER_LINUX_SOCKET_H__
