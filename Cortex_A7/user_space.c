#include<unistd.h>
#include<fcntl.h>
#include<stdio.h>
#include<signal.h> 
#include<errno.h>
#define MSG_RECEP 100  //Pour 100 messages envoyés depuis le M4


int main(){

	unsigned char msg[496*MSG_RECEP];
	int size;

	int fd=open("/dev/rpmsg-client-user", O_RDONLY);
	int fd_write=open("./data", O_CREAT|O_WRONLY,0666);

	if(fd<0){
		printf("can't open file /dev/rpmsg-client-user \n");
		return -1;	
	}
	
	printf("%d\n", fd_write);
	while(1){
		size=read(fd, msg, sizeof(msg));
		if(size==-1){
			perror("Read function");
			return -1;	
		}		

		if(size>0){
		printf("%d\n",size);

		write(fd_write, msg , sizeof(msg));

		}
	}

	close(fd_write);
	close(fd);
return 0;
}
