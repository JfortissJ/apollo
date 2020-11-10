/******************************************************************************
 * Apollo Autobox Control
 * Copyright (C) 2020 fortiss GmbH
 * Authors: Tobias Kessler, Jianjie Lin
 * 
 * This library is free software; you can redistribute it and/or modify it 
 * under the terms of the GNU Lesser General Public License as published by the 
 * Free Software Foundation; either version 2.1 of the License, or (at your 
 * option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT 
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License 
 * for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License 
 * along with this library; if not, write to the Free Software Foundation, 
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *****************************************************************************/
#ifndef MODULES_BRIDGE_C_COMMON_DESERIALIZE_CHASSIS_C
#define MODULES_BRIDGE_C_COMMON_DESERIALIZE_CHASSIS_C

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <pthread.h>
#include <sys/epoll.h>
#include <sys/resource.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "modules/bridge_c/common/macro.h"
#include "modules/bridge_c/common/bridge_header.h"
#include "modules/bridge_c/common/bridge_proto_diserialized_buf.h"


void* pthread_handle_message(void* pfd)
{
	printf("\n-----------------------Handle_Message---------------------------\n");
	struct sockaddr_in client_addr;
	socklen_t sock_len = (socklen_t) (sizeof(client_addr));
	int bytes = 0;
	int total_recv = 2 * (int) APOLLO_BRIDGE_MARCO_FRAME_SIZE;
	char total_buf[2 * APOLLO_BRIDGE_MARCO_FRAME_SIZE];
	for (size_t i = 0; i < 2 * APOLLO_BRIDGE_MARCO_FRAME_SIZE; ++i)
	{
		total_buf[i] = 0;
	}
	printf("sock_len: %d\n",sock_len);
	bytes = (int) (recvfrom(*(int*) (pfd), total_buf, total_recv,
							0, (struct sockaddr*) &client_addr, &sock_len));
	printf("total_bytes: %d\n", bytes);
	printf("total buffer: %s\n",total_buf);
	if(bytes <= 0 || bytes > total_recv)
	{
		pthread_exit(NULL);
	}
	char header_flag[sizeof(BRIDGE_HEADER_FLAG) + 1];
	for (size_t j = 0; j < sizeof(BRIDGE_HEADER_FLAG) + 1; ++j)
	{
		header_flag[j] = 0;
	}
	size_t offset = 0;
	memcpy(header_flag, total_buf, HEADER_FLAG_SIZE);
	printf("header_flag: %s\n",header_flag);
	if(strcmp(header_flag, BRIDGE_HEADER_FLAG) != 0)
	{
		printf("header flag not match!\n");
		pthread_exit(NULL);
	}
	offset += sizeof(BRIDGE_HEADER_FLAG) + 1;
	
	char header_size_buf[sizeof(hsize) + 1];
	for (size_t k = 0; k < sizeof(hsize) + 1; ++k)
	{
		header_size_buf[k] = 0;
	}
	printf("offset: %ld\n",offset);
	const char* cursor = total_buf + offset;
	memcpy(header_size_buf, cursor, sizeof(hsize));
	printf("header_size_buf: %s\n",header_size_buf);
	hsize header_size = *((hsize*) (header_size_buf));
	if(header_size > APOLLO_BRIDGE_MARCO_FRAME_SIZE)
	{
		printf("header size is more than FRAME_SIZE!\n");
		pthread_exit(NULL);
	}
	offset += sizeof(hsize) + 1;
	
	APOLLO_BRIDGE_HEADER header;
	APOLLO_BRIDGE_HEADER_Item_Init(&header);
	printf("heder_size: %d, offset %ld\n",header_size,offset);
	size_t buf_size = header_size - offset;
	cursor = total_buf + offset;
	printf("buf_size: %ld\n",buf_size);
	if(!APOLLO_BRIDGE_HEADER_Diserialize(&header, cursor, buf_size))
	{
		printf("header diserialize failed!\n");
		pthread_exit(NULL);
	}
	
	// printf("proto name: %s\n", APOLLO_BRIDGE_HEADER_GetMsgName(&header));
	// printf("proto sequence num: %d\n ", APOLLO_BRIDGE_HEADER_GetMsgID(&header));
	// printf("proto total frames: %d\n ", APOLLO_BRIDGE_HEADER_GetTotalFrames(&header));
	// printf("proto frame index: %d\n", APOLLO_BRIDGE_HEADER_GetIndex(&header));
	BridgeProtoDiserializedBuf proto_buf;
	bool hasInit = BridgeProto_Diserialized_Buf_Initialize(&proto_buf, header);
	if(!hasInit)
	{
		pthread_exit(NULL);
	}
	cursor = total_buf + header_size;
	printf("the offset: %d\n",header_size);
	char* buf = BridgeProto_Diserialized_Buf_GetBuf(&proto_buf, APOLLO_BRIDGE_HEADER_GetFramePos(&header));
	memcpy(buf, cursor, APOLLO_BRIDGE_HEADER_GetFrameSize(&header));
	BridgeProto_Diserialized_Buf_UpdateStatus(&proto_buf, APOLLO_BRIDGE_HEADER_GetIndex(&header));
	printf("proto_buf->diserializedBuf.total_size_: %d\n", APOLLO_BRIDGE_HEADER_GetFrameSize(&header));
	if(BridgeProto_Diserialized_Buf_IsReadyDiserialize(&proto_buf))
	{
		BridgeProto_Diserialized_Buf_Chassis_Diserialized(proto_buf.proto_buf_, proto_buf.total_size_);

	}
	pthread_exit(NULL);
}


bool receive(uint16_t port)
{
	
	struct rlimit rt;
	rt.rlim_max = rt.rlim_cur = APOLLO_BRIDGE_MARCO_MAXEPOLLSIZE;
	if(setrlimit(RLIMIT_NOFILE, &rt) == -1)
	{
		printf("set resource limitation failed\n");
		return false;
	}
	
	int listener_sock = socket(AF_INET, SOCK_DGRAM, 0);
	if(listener_sock == -1)
	{
		printf("create socket failed\n");
		return false;
	}
	int opt = SO_REUSEADDR;
	setsockopt(listener_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
	if(fcntl(listener_sock, F_SETFL,
			 fcntl(listener_sock, F_GETFD, 0) | O_NONBLOCK) == -1)
	{
		printf("set nonblocking failed\n");
		return false;
	}
	
	struct sockaddr_in serv_addr;
	serv_addr.sin_family = PF_INET;
	serv_addr.sin_port = htons((uint16_t) port);
	serv_addr.sin_addr.s_addr = INADDR_ANY;
	if(bind(listener_sock, (struct sockaddr*) &serv_addr,
			sizeof(struct sockaddr)) == -1)
	{
		close(listener_sock);
		printf("bind socket failed\n");
		return false;
	}
	int kdpfd = epoll_create(APOLLO_BRIDGE_MARCO_MAXEPOLLSIZE);
	struct epoll_event ev;
	ev.events = EPOLLIN | EPOLLET;
	ev.data.fd = listener_sock;
	if(epoll_ctl(kdpfd, EPOLL_CTL_ADD, listener_sock, &ev) < 0)
	{
		printf("set control interface for an epoll descriptor failed\n");
		close(listener_sock);
		return false;
	}
	
	printf("Ready!\n");
	
	int nfds = -1;
	bool res = true;
	struct epoll_event events[APOLLO_BRIDGE_MARCO_MAXEPOLLSIZE];
	while (true)
	{
		nfds = epoll_wait(kdpfd, events, 10000, -1);
		if(nfds == -1)
		{
			printf("some error occurs while waiting for I/O event\n");
			res = false;
			break;
		}
		
		for (int i = 0; i < nfds; ++i)
		{
			if(events[i].data.fd == listener_sock)
			{
				pthread_t thread;
				pthread_attr_t attr;
				pthread_attr_init(&attr);
				pthread_attr_setscope(&attr, PTHREAD_SCOPE_SYSTEM);
				pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
				if(pthread_create(&thread, &attr, &pthread_handle_message,
								  (void*) (&events[i].data.fd)))
				{
					printf("message handler creation failed\n");
					res = false;
					break;
				}
			}
		}
		if(!res)
		{
			break;
		}
	}
	close(listener_sock);
	return res;
}

int main(int argc, char* argv[])
{
	receive(8900);
	return 0;
}

#endif
