#include<stdio.h>
#include<stdlib.h>
#include<unistd.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include<string.h>
#include<time.h>
#include<stdbool.h>
#include "server.h"
#define RUN_TIME 1800
char mac_addr[18] = "00:16:53:0A:20:AE"; // MAC address of the BT device

int main(int argc, char *argv[])

{ 
        int client_sock = -1; // socket for the BT comm 
	int comm_res,connect_status; 
        struct sockaddr_rc addr_data = {0};
        if(str2ba(mac_addr, &addr_data.rc_bdaddr) == -1) // Convert BT MAC addr to rc_bdaddr 
        	return -1;
        client_sock = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM); // Socket initialization for BT comm
        addr_data.rc_family = AF_BLUETOOTH; // specify its a BT socket
        addr_data.rc_channel = (uint8_t) 1; 
        connect_status = connect(client_sock, (struct sockaddr *)&addr_data, sizeof(addr_data)); 
        if(connect_status == -1) {
        	printf("Error in BT socket initialization \n");
                close(client_sock);
                return -1;
         }
         printf("So far so good \n");
         RefPacket out_packet;
         FILE *fp;
         fp = fopen(argv[1], "w+");
         fprintf(fp,"Sample No., Time (ms), Motor Count \n");
         DataPacket in_packet;
         out_packet.speed = (float)5; 
         comm_res = recv(client_sock, &in_packet,sizeof(DataPacket), 0);
         comm_res = send(client_sock, &out_packet, sizeof(RefPacket), 0);
         if(comm_res != sizeof(RefPacket))
		return -1;
         unsigned long sample_no = 1;
         unsigned long time_elapsed = 0; 
         int k;
         do{
          	comm_res = recv(client_sock, &in_packet,sizeof(DataPacket), 0);
                time_elapsed = time_elapsed + in_packet.delta;
                fprintf(fp,"%ld, %ld, %ld\n",sample_no,time_elapsed,in_packet.count);
                sample_no++;
                k++;
         }while(k < RUN_TIME);
         fclose(fp);
         close(client_sock);
         printf("Application exit \n");
         return 0; 
}   
