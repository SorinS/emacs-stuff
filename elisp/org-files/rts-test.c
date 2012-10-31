#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#define JURA_SSI_IOC_MAGIC     'o' // free in ioctl-number.txt
#define JURA_SSI_IOC_SETDTR    _IOW(JURA_SSI_IOC_MAGIC, 1, int)
#define JURA_SSI_IOC_SETRISING _IOW(JURA_SSI_IOC_MAGIC, 2, int)
#define JURA_SSI_IOC_GETDCD    _IOR(JURA_SSI_IOC_MAGIC, 3, int)
#define JURA_SSI_IOC_GETCTS    _IOR(JURA_SSI_IOC_MAGIC, 4, int)
#define JURA_SSI_IOC_SETRTS    _IOW(JURA_SSI_IOC_MAGIC, 5, int)
#define JURA_SSI_IOC_GETRTS    _IOR(JURA_SSI_IOC_MAGIC, 6, int)

int main(void)
{
  int serial_port;
  int rts_up = 0;
  int rts_is_up = 0;

  if ((serial_port = open("/dev/jura_ssi", O_RDWR | O_NDELAY)) == -1)
  {
    fprintf(stderr, "Unable to open serial port: %s\n", strerror(errno));
    exit(EXIT_FAILURE);
  }

  while (1)
  {
    rts_up = !rts_up;
    printf("%s RTS\n", (rts_up) ? "Raising" : "Dropping");
    
    if (ioctl(serial_port, JURA_SSI_IOC_SETRTS, &rts_up) != 0)
    {
      fprintf(stderr, "Unable to manipulate RTS: %s\n", strerror(errno));
    }

    if (ioctl(serial_port, JURA_SSI_IOC_GETRTS, &rts_is_up) != 0)
    {
      fprintf(stderr, "Unable to determine RTS state: %s\n", strerror(errno));
    }
    else
    {
      printf("RTS reports as %s\n", rts_is_up ? "UP" : "DOWN");
    }
    sleep(5);
  }

  return 0;
}
