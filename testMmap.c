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

int main(void)
{
    struct lunix_msr_data_struct * addr;
    int fd;
    off_t offset, pa_offset;
    int length;
    ssize_t s;
    unsigned int buf_timestamp, our_timestamp = 0;
    unsigned int value;

    if (fd = open("/dev/lunix1-temp", O_RDONLY) < 0)
    {
        perror("open");
        return 1;
    }

    addr = mmap(NULL, 100, PROT_READ, MAP_PRIVATE, fd, 0);
    if (addr == MAP_FAILED)
    {
        perror("mmap");
        return 1;
    }

    usleep(700000);

    while(1)
    {
        buf_timestamp = addr->last_update;
        printf("Our: %u, their: %u\n", our_timestamp, buf_timestamp);
        if(buf_timestamp != our_timestamp)
        {
            our_timestamp = buf_timestamp;
            value = addr->values[0];
            printf("Number read is: %u\n", value);
        }
        usleep(300000);
    }
    return 0;
}
