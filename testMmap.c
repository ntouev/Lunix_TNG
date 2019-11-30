/*
 * A program to test mmap method
 */
#include <unistd.h>
#include <stdlib.h>
#include <inttypes.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/mman.h>

struct lunix_msr_data_struct {
    uint32_t magic;
    uint32_t last_update;
    uint32_t values[];
};

int main(int argc, char **argv)
{
    int fd;
    struct lunix_msr_data_struct *addr;
    unsigned int buf_timestamp, my_timestamp = 0;

    if (argc != 2)
    {
        fprintf(stderr, "Usage: ./testMmap /dev/lunixX-XXXX\n");
        return 1;
    }

    fd = open(argv[1], O_RDONLY);
    if (fd < 0)
    {
        perror("open");
        return 1;
    }
    printf("Opened special file %s with fd: %d\n", argv[1], fd);

    addr = mmap(NULL, 20, PROT_READ, MAP_PRIVATE, fd, 0);
    if (addr == MAP_FAILED)
    {
        perror("mmap");
        return 1;
    }

    while(1)
    {
        buf_timestamp = addr->last_update;
        if(buf_timestamp != my_timestamp)
        {
            my_timestamp = buf_timestamp;
            printf("%x\n", addr->values[0]);
        }
    }
    return 0;
}
