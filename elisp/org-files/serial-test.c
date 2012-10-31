/******************************************************************************
 *
 * NAME
 *
 *   serial-test
 *
 * SYNOPSIS
 *
 *   serial-test --read|write --232|--422|--485|--ssi --device=device-name
 *               [--baud=57600] [--parodd|--pareven] [--hwfc]
 *
 * DESCRIPTION
 *
 *   Provides a means of testing various forms of serial communications to/form
 * Mark II/IV product types.  In --write mode, the alphabet is transmitted a
 * character at a time, a to z, looping continuously.  In --read mode, the
 * first character received becomes the reference, with subsequent reads being
 * validated against the previous read, to ensure that data arrives in the
 * correct order.
 *
 *   At least one of --read or --write must be supplied, as must a serial mode
 * (--232, --422, --485 or --ssi).  Additionally, the --device-name must 
 * indicate the full path to the serial device.
 *
 * Unless overridden, --baud is 57600, and both parity and hardware flow
 * control are disabled.
 *
 * AUTHOR
 *
 *   Danny Woods
 *
 * COPYRIGHT
 *
 *   (c) Essential Viewing 2008-2009
 *
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <asm/ioctls.h>
#include <sys/time.h>
#include <getopt.h>

static int              serial_port;
static int              ctrl_c_count;
static volatile int     running;
static  char            alphabet[] = "abcdefghijklmnopqrstuvwxyz";

typedef enum _product_type_t { MARK_II, MARK_IV } product_type_t;

static product_type_t determine_product_type();
static int enable_422(product_type_t product_type, int serial_port);
static int enable_485(product_type_t product_type, int serial_port);
static void do_sync_serial_write_test();
static void do_async_serial_write_test();
static int sync_stream(int serial_port, unsigned char * buffer, size_t buflen);
static int send_sync_code(int serial_port);
static void cleanup(int signal);
static void usage(const char * program_name, int exit_code);

#define TIOCSERSETRS485 0x5461
#define JURA_SSI_IOC_MAGIC  'o'
#define JURA_SSI_IOC_SETDTR _IOW(JURA_SSI_IOC_MAGIC, 1, int)

enum serial_mode_t { RS422, RS485, RS232, SYNC_SERIAL } serial_mode = RS485;

int main(int argc, char *argv[])
{
  int                    dtr_bit       = TIOCM_DTR;
  int                    rts_bit       = TIOCM_RTS;
  int                    lsr           = 0;
  int                    c;
  int                    hwfc          = 0;
  char                  *tty           = NULL;
  char                  *cp;
  fd_set                 rfds, wfds;
  struct termios         config;
  product_type_t         product_type;
  int                    raw_baud_rate = 57600;
  speed_t                baud_rate     = B57600;

  enum parity_t { PARITY_NONE, PARITY_ODD, PARITY_EVEN } parity = PARITY_NONE;
  enum operating_mode_t { READ, WRITE } mode                    = READ;
  struct option long_options[]                                  = {
    {"422",    no_argument,       0, '2' },
    {"485",    no_argument,       0, '8' },
    {"232",    no_argument,       0, '3' },
    {"ssi",    no_argument,       0, 's' },
    {"read",   no_argument,       0, 'r' },
    {"write",  no_argument,       0, 'w' },
    {"device", required_argument, 0, 'd' },
    {"pareven", no_argument,      0, 'e' },
    {"parodd", no_argument,       0, 'o' },
    {"hwfc",   no_argument,       0, 'h' },
    {"baud",   required_argument, 0, 'b' },
    {0, 0, 0, 0}
  };
  int   option_index = 0;
  
  while((c = getopt_long(argc, argv, "d:823swrheob:", long_options, &option_index))
        !=
        -1)
  {
    switch(c)
    {
      case 'd': tty           = optarg; break;
      case '8': serial_mode   = RS485; break;
      case '2': serial_mode   = RS422; break;
      case '3': serial_mode   = RS232; break;
      case 's': serial_mode   = SYNC_SERIAL; break;
      case 'w': mode          = WRITE; break;
      case 'r': mode          = READ; break;
      case 'h': hwfc          = 1; break;
      case 'e': parity        = PARITY_EVEN; break;
      case 'o': parity        = PARITY_ODD; break;
      case 'b': raw_baud_rate = atoi(optarg); break;
    }
  }

  if (!tty) usage(argv[0], 1);

  ////////////////////////////////////////////////////////////////////////////
  // Mark II or IV?
  ////////////////////////////////////////////////////////////////////////////

  product_type = determine_product_type();
  printf("Running on Mark%s hardware\n",
         (product_type == MARK_II) ? "II" : "IV");

  switch(raw_baud_rate)
  {
    case 1200: baud_rate   = B1200; break;
    case 2400: baud_rate   = B2400; break;
    case 4800: baud_rate   = B4800; break;
    case 9600: baud_rate   = B9600; break;
    case 19200: baud_rate  = B19200; break;
    case 38400: baud_rate  = B38400; break;
    case 57600: baud_rate  = B57600; break;
    case 115200: baud_rate = B115200; break;
    default:
      fprintf(stderr, "Baud rate must be one of 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200\n");
      exit(EXIT_FAILURE);
  }

  ////////////////////////////////////////////////////////////////////////////
  // Running options summary
  ////////////////////////////////////////////////////////////////////////////

  printf("Operating in %s %s mode at %d baud on %s with %sparity %s and h/w flow control %s...\n",
         (serial_mode == RS422) ? "422" : (serial_mode == RS485) ? "485" : (serial_mode == RS232) ? "232" : "synchronous serial",
         (mode == READ) ? "read" : "write",
         raw_baud_rate,
         tty,
         (parity == PARITY_NONE) ? "" : (parity == PARITY_EVEN) ? "even" : "odd",
         (parity == PARITY_NONE) ? "disabled" : "enabled",
         (hwfc == 1) ? "enabled" : "disabled");

  /////////////////////////////////////////////////////////////////////////////
  // Catch CTRL-C for cleanup.
  /////////////////////////////////////////////////////////////////////////////

  signal(SIGINT, cleanup);

  /////////////////////////////////////////////////////////////////////////////
  // Open and configure the serial port
  /////////////////////////////////////////////////////////////////////////////

  if ((serial_port = open(tty, O_RDWR | O_NOCTTY | O_NDELAY, 0)) < 0)
  {
    perror("open");
    exit(EXIT_FAILURE);
  }

  /////////////////////////////////////////////////////////////////////////////
  // Enable 485/422 for the product that we're running on
  /////////////////////////////////////////////////////////////////////////////

  if (serial_mode == RS422) enable_422(product_type, serial_port);
  else if (serial_mode == RS485) enable_485(product_type, serial_port);

  fcntl(serial_port, F_SETFL, O_NONBLOCK);

  if (serial_mode != SYNC_SERIAL && ioctl(serial_port, TIOCMBIC, &dtr_bit) != 0)
  {
    perror("ioctl (DTR down)");
    exit(EXIT_FAILURE);
  }

  tcgetattr(serial_port, &config);

  ////////////////////////////////////////////////////////////////////////////
  // Config:
  ////////////////////////////////////////////////////////////////////////////

  config.c_cflag &= ~(CSIZE | CSTOPB | PARENB);
  config.c_cflag |=  (CS8 | HUPCL | CLOCAL);

  if (parity == PARITY_EVEN)
  {
    config.c_cflag |= PARENB;
  }
  else if (parity == PARITY_ODD)
  {
    config.c_cflag |= PARENB;
    config.c_cflag |= PARODD;
  }

  if(hwfc) config.c_cflag |= CRTSCTS;
  else     config.c_cflag &= ~CRTSCTS;

  ///////////////////////////////////////////////////////////////////////////
  // Output: no output processing
  // (see ftp://ftp.tyumen.ru/pub/old_stuff/new/doc/ser_config.html)
  ///////////////////////////////////////////////////////////////////////////

  config.c_oflag &= ~OPOST;

  ///////////////////////////////////////////////////////////////////////////
  // Local: raw input
  ///////////////////////////////////////////////////////////////////////////

  config.c_lflag &= ~(ICANON | ECHO | ISIG);

  config.c_cc[VMIN]=1;
  config.c_cc[VTIME]=0;

  cfsetospeed(&config, baud_rate);
  cfsetispeed(&config, baud_rate);

  tcsetattr(serial_port, TCSAFLUSH, &config);

  ////////////////////////////////////////////////////////////////////////////
  // ioctl twiddling to get ready for data send/receipt
  ////////////////////////////////////////////////////////////////////////////

  if (ioctl(serial_port,
            (serial_mode == SYNC_SERIAL) ? JURA_SSI_IOC_SETDTR : TIOCMBIS, 
            &dtr_bit) != 0)
  {
    perror("ioctl (DTR up)");
    exit(EXIT_FAILURE);
  }

  if (serial_mode == RS232)
  {
    if (ioctl(serial_port, TIOCMBIS, &rts_bit) != 0) // Enable inward flow
    {
      perror("ioctl (RTS up)");
      exit(EXIT_FAILURE);
    }
  }

  ////////////////////////////////////////////////////////////////////////////
  // The functionality split: what happens next depends upon whether or not
  // the mode is WRITE or READ.
  ////////////////////////////////////////////////////////////////////////////

  sleep(1);
  running = 1;
  if (mode == WRITE)
  {
    if (serial_mode == SYNC_SERIAL)
      do_sync_serial_write_test();
    else
      do_async_serial_write_test();
  }
  else // mode == READ
  {
    int bytes_read = 0;
    int ref = -1;
    unsigned char byte_buffer[65536];
    int buffer_used = 0;

    while(running)
    {
      int status;
      char ch;
      FD_ZERO(&rfds);
      FD_SET(serial_port, &rfds);

      if (select(serial_port + 1, &rfds, NULL, NULL, NULL) != -1)
      {
        if (FD_ISSET(serial_port, &rfds))
        {
          if (serial_mode == SYNC_SERIAL)
          {
            if ((buffer_used = sync_stream(serial_port, byte_buffer, sizeof(byte_buffer))) <= 0)
            {
              fputs("Failed to synchronise stream\n", stderr);
            }
            else
            {
              printf("\rREAD (ssi) %d bytes     ", buffer_used);
            }
            continue;
          }
          else
          {
            status = read(serial_port, &ch, 1);
          }

          if (status == 1)
          {
            fprintf(stderr, "\rREAD %c: %d", (ch == '\n') ? '#' : ch, ++bytes_read);
            fflush(stdout);

            ///////////////////////////////////////////////////////////////////
            // Check that the character just read is the alphabetic successor
            // of the one received last time, with -1 as a starter flag, and
            // 'a' being the successor to 'z'.
            ///////////////////////////////////////////////////////////////////

            if (ref == -1)
            {
              ref = ch;
            }
            else if ((ch != (ref + 1)) && (ch == 'z' && ref != 'a'))
            {
              fprintf(stderr, "Unexpected character: '%c' does not succeed '%c'\n", ch, ref);
            }

            ref = ch;

            ///////////////////////////////////////////////////////////////////
            // Check that the character read is alphabetic: the other side
            // should only be sending these
            ///////////////////////////////////////////////////////////////////

            if (!isalnum(ch) && ch != '!' && ch != '\n')
            {
              fprintf(stderr, "read (malformed character: '%c')\n", ch);
            }
          }
          else
          {
            perror("select/read");
          }
        }
      }
      else
      {
        printf("Read code: %d; errno: %d\n", status, errno);
        perror("read");
        running = 0;
      }
    }
  }

  close(serial_port);
  serial_port = 0;

  return 0;
}

static int sync_stream(int serial_port, unsigned char * buffer, size_t buflen)
{
  int                   source_bits = 0, dest_bits = 0;
  unsigned long         source_word = 0, dest_word = 0;
  unsigned char *       buffer_ptr  = buffer;
  int                   byte        = 0, synchronised      = 0, bytes_read = 0;
  int                   stop_count  = 0, i;
  unsigned long         mask;
  fd_set                read_fds;

  FD_ZERO(&read_fds);
  FD_SET(serial_port, &read_fds);

  printf("Syncing..."); fflush(stdout);
  while(1)
  {
    while (source_bits < 16)
    {
      if (select(serial_port + 1, &read_fds, NULL, NULL, NULL) > 0)
      {
        if (read(serial_port, &byte, 1) != 1) return -1;
        source_word |= ((unsigned long)byte << source_bits);
        source_bits += 8;
      }
    }
    int bits = 15;
    for (mask = 1 << (bits - 1); mask && (mask & source_word); mask >>= 1) bits--;
    if (bits == 0) // synced!
    { 
      source_word  >>= 16;
      source_bits   -= 16;
      synchronised   = 1;
    }
    else if(!synchronised)
    {
      source_word >>= bits;
      source_bits  -= bits;
    }
    else
    {
      dest_word = (dest_word >> bits) | (source_word << (32 - bits));
      dest_bits += bits;
      source_word >>= bits;
      source_bits  -= bits;
      break;
    }
  }
  puts("Done");
  
  /////////////////////////////////////////////////////////////////////////////
  // We're synched up on something that's not a sync code by this, with
  // perhaps a fragment in dest_word
  // Need to read up until we find a sync code, or until we run
  // out of buffer.
  /////////////////////////////////////////////////////////////////////////////


  while (bytes_read < buflen)
  {
    while (source_bits < 16)
    {
      if (select(serial_port + 1, &read_fds, NULL, NULL, NULL) > 0)
      {
        if (read(serial_port, &byte, 1) != 1)
        {
          return -2;
        }
        source_word |= ((unsigned long)byte << source_bits);
        source_bits += 8;
      }
    }
    
    int bits = 15;

    for (mask = 1 << (bits - 1); mask && (mask & source_word); mask >>= 1) bits--;
    
    if (bits == 0) // another sync code
    {
      break;
    }
    else
    {
      dest_word     = (dest_word >> bits) | (source_word << (32 - bits));
      dest_bits    += bits;
      source_bits  -= bits;
      source_word >>= bits;
    }

    while (dest_bits > 7)
    {
      *buffer_ptr++ = (dest_word >> (32 - dest_bits)) & 0xff;
      bytes_read++;
      dest_bits -= 8;
    }
  }

  return bytes_read;
}

static int enable_485(product_type_t product_type, int serial_port)
{
  switch(product_type)
  {
    case MARK_IV:
    {
      int enable = 1;
      if (ioctl(serial_port, TIOCSERSETRS485, (int*)&enable) != 0)
      {
        perror("Failed to enable MkIV 485 mode");
        return -1;
      }
    }
    break;
    case MARK_II:
    {
      FILE *fp;
      if ((fp = fopen("/proc/evs_gpio/com_422", "w")) != NULL)
      {
        fputc('0', fp);
        fclose(fp);
      }
      else
      {
        perror("Failed to enable MkII 485 mode");
        return -2;
      }
      break;
    }
    default:
      return -3;
  }
  printf("Enabled %s 485 mode\n", (product_type == MARK_IV) ? "Mark IV" : "Mark II");
  return 0;
}

static int enable_422(product_type_t product_type, int serial_port)
{
  switch(product_type)
  {
    case MARK_IV:
    {
      int disable = 0;
      if (ioctl(serial_port, TIOCSERSETRS485, (int*)&disable) != 0)
      {
        perror("Failed to enable MkIV 422 mode by disabling 485 mode");
        return -1;
      }
      break;
    }
    case MARK_II:
    {
      FILE *fp;
      if ((fp = fopen("/proc/evs_gpio/com_422", "w")) != NULL)
      {
        fputc('1', fp);
        fclose(fp);
      }
      else
      {
        perror("Failed to enable MkII 422 mode");
        return -2;
      }
      break;
    }
    default:
      return -3;
  }
  printf("Enabled %s 422 mode\n", (product_type == MARK_IV) ? "Mark IV" : "Mark II");
  return 0;
}

static void cleanup(int signal)
{
  running = 0;
  if (ctrl_c_count++ > 0)
  {
    int dtr_bit = TIOCM_DTR;
    puts("Dropping DTR");
    int result = 0;
    if (serial_port > 0)
    {
      result = 1;
      if (serial_mode == SYNC_SERIAL)
      {
        result = result && 0;
      }
      else
      {
        result = result && (ioctl(serial_port, TIOCMBIC, &dtr_bit) == 0);
      }
    }

    if (!result) perror("ioctl (DTR down, signal handler");
    close(serial_port);
    puts("Exiting");
    exit(1);
  }
  else
  {
    puts("Requesting stop.  Ctrl-C again to force");
  }
}

static void usage(const char * program_name, int exit_code)
{
  fprintf(stderr,
          "Usage: %s --device=/dev/ttyX --read|--write --422|--485|--232|--ssi [--hwfc] [--parodd|--pareven] [--baud=57600]\n",
          program_name);
  exit(exit_code);
}

static product_type_t determine_product_type()
{
  FILE *fp = NULL;
  if (fp = fopen("/proc/evs_gpio/com_422", "r"))
  {
    fclose(fp);
    return MARK_II;
  }
  else
  {
    return MARK_IV;
  }
}

static int send_sync_code(int serial_port)
{
  fd_set wfds;
  int   sync_code_sent = 0;

  while (!sync_code_sent)
  {
    FD_ZERO(&wfds);
    FD_SET(serial_port, &wfds);
    
    if (select(serial_port + 1, NULL, &wfds, NULL, NULL) != -1)
    {
      if (FD_ISSET(serial_port, &wfds))
      {
        unsigned char sync_code[2] = {0xFF, 0x7F};
        
        if (!write(serial_port, sync_code, 2) == 2)
        {
          perror("write (sync code)");
          exit(EXIT_FAILURE);
        }
        else
        {
          sync_code_sent = 1;
        }
      }
      else
      {
        perror("select (sync code)");
        exit(EXIT_FAILURE);
      }
    }
  }
}

static int sync_write(unsigned char * buffer, int length, int serial_port)
{
  int                   bytes_written = 0;
  unsigned char         packet[65541];
  unsigned char *       pptr          = packet;
  fd_set                wfds;

  if (length + 5 > sizeof(packet))
  {
    fprintf(stderr, "Send buffer is too big (%d requested vs %d available) .  Skipping\n", length, sizeof(packet));
    return;
  }

  memcpy(packet + 2, buffer, length);

  packet[0] = 0xFF;
  packet[1] = 0x7F;
  packet[length+2] = 0;
  packet[length+3] = 0xFF;
  packet[length+4] = 0x7F;

  while(pptr < &packet[length + 5])
  {
    FD_ZERO(&wfds);
    FD_SET(serial_port, &wfds);

    if (select(serial_port + 1, NULL, &wfds, NULL, NULL) == -1)
    {
      perror("sync select");
      exit(EXIT_FAILURE);
    }
    if (FD_ISSET(serial_port, &wfds))
    {
      int written = write(serial_port, pptr, &packet[length + 5] - pptr);

      if (written == -1)
      {
        perror("sync write");
        exit(EXIT_FAILURE);
      }
      else
      {
        pptr += written;
        bytes_written += written;
      }
    }
  }

  return bytes_written - 5;
}

static void do_sync_serial_write_test()
{
  int            bytes_written   = 0;
  unsigned char  packet[256]     = {0};
  int            dtr_bit         = 0;
  int            last_chunk_size = 0;
  unsigned char *pptr;
  int            i;

  for (pptr = packet, i = 0; pptr < &packet[256]; *pptr++ = (i++) & 0x7e)
    ;

  for (pptr = &packet[255]; pptr > packet; pptr--)
  {
    int just_written                     = sync_write(pptr, &packet[256] - pptr, serial_port);
    printf("Wrote a chunk of %d bytes\n", just_written);
    for (i = 0; i < just_written; i++) printf("%x ", pptr[i]);
    puts("");
    if (just_written > 0) bytes_written += just_written;
    sleep(5);
  }

  printf("Wrote %d bytes in total\n", bytes_written);

  sleep(30);

  if (ioctl(serial_port, JURA_SSI_IOC_SETDTR, &dtr_bit) != 0)
  {
    fprintf(stderr, "Unable to drop DTR\n");
  }

}


static void do_async_serial_write_test()
{
  char  *cp;
  int    bytes_written = 0;

  for (cp = &alphabet[0];
       running;
       cp = (*cp == 'z') ? &alphabet[0] : cp + 1)
  {
    if (write(serial_port, cp, 1) != 1)
    {
      perror("write");
      exit(EXIT_FAILURE);
    }
    printf("\rWRITE %c: %d", *cp, ++bytes_written);
    fflush(stdout);
  }
}

