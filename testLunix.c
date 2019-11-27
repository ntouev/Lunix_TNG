//A userspace program to test the ioctl method
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <string.h>

#include "lunix-chrdev.h"

int main(int argc, char **argv)
{
    int mode, fd, status;
    pid_t pid;
    size_t bytes_read;
    char buffer;

    if (argc != 3)
    {
        fprintf(stderr, "Usage: ./testLunix /dev/lunixX-XXXX Y[0/1], 1:for raw_data\n");
        return 1;
    }

    mode = atoi(argv[2]);
    if ((mode != 0) & (mode !=1))
    {
        fprintf(stderr,"Third argument has to be 0 or 1, 1 for raw_data\n");
        return 1;
    }

    fd = open(argv[1],O_RDONLY);
    if (fd < 0)
    {
        perror(argv[1]);
        return 1;
    }

    if (mode)
    {
        printf("changed to raw bytes mode\n");
        if (ioctl(fd, LUNIX_IOC_DATA_TYPE_CONVERT, NULL)) //change to raw bytes
        {
            perror("ioctl");
            return 1;
        }
    }

    pid = fork();
    if (pid == 0)
    {
        do
        {
            bytes_read = read(fd, &buffer, 1);
            if (bytes_read < 0)
            {
                perror("read");
                exit(1);
            }

            printf("%c", buffer);
        } while (bytes_read > 0);

        //Unreachable
        perror("reached non reachable point, internal error");
        exit(0);
    }

    wait(&status);

    return 0;
}
