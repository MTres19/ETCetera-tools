/****************************************************************************
 * apps/industry/ETCetera-tools/cantest_main.c
 * Electronic Throttle Controller program - CAN bus test utility
 *
 * Copyright (C) 2020  Matthew Trescott <matthewtrescott@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>


#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <nuttx/can/can.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <semaphore.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>

#include "system/readline.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FLAG_HELP         1
#define FLAG_UNRECOGNIZED 2
#define FLAG_GETOPT_ERR   4

/****************************************************************************
 * Private Types
 ****************************************************************************/


/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Defined by the NSH example application */

int nsh_main(int argc, char **argv);

static void print_help(void);
static int filter_candevs(const struct dirent *file);
#ifdef CONFIG_CAN_ERRORS
static void print_errframe(const struct can_msg_s *msg);
#endif
static void print_canmsgs(uint8_t *msgs, int buflen);
static void test_basic_receive(const int canfd);
static void test_add_std_filter(int canfd);
static int parse_mask(const char *msk, void *fltr_ptr, bool extended);
static void test_rtr_transaction(int canfd);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static sem_t g_poll_test_sem;

/****************************************************************************
 * Public Data
 ****************************************************************************/


/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: print_help
 *
 * Description:
 *   Print usage information about cantest.
 ****************************************************************************/

static void print_help(void)
{
  printf( "cantest - validate NuttX CAN drivers and the ETCetera CAN support.\n"
          "Usage: cantest [--help|-h] [--dev|-d <device>]\n"
          "       --help: Print this information.\n"
          "       --dev:  Use CAN device <device>. The default behavior is to\n"
          "               search /dev and select the first available device.\n");
}

/****************************************************************************
 * Name: filter_candevs
 *
 * Description:
 *   Checks if a file in /dev seems to be a CAN device file
 *   (for use with scandir)
 *
 * Input parameters:
 *   file - struct durent from scandir
 *
 * Returned value:
 *   1 if the filename looks like a CAN device, 0 otherwise.
 ****************************************************************************/

static int filter_candevs(const struct dirent *file)
{
  if (file->d_type != DT_CHR)
    return 0;

  if (strncmp(file->d_name, "can", 3) == 0)
    return 1;

  return 0;
}

/****************************************************************************
 * Name: print_errframe
 *
 * Description:
 *   Prints human-readable error messages for the frame to standard output.
 *
 * Input parameters:
 *   msg - Pointer to struct can_msg_s with msg->cm_hdr.ch_error set.
 ****************************************************************************/

#ifdef CONFIG_CAN_ERRORS
static void print_errframe(const struct can_msg_s *msg)
{
  if (msg->cm_hdr.ch_error)
    {
      printf("Error report:\n");

      if (msg->cm_hdr.ch_id & CAN_ERROR_TXTIMEOUT)
        {
          printf("  TX timeout\n");
        }
      if (msg->cm_hdr.ch_id & CAN_ERROR_LOSTARB)
        {
          printf("  Lost arbitration\n");
        }
      if (msg->cm_hdr.ch_id & CAN_ERROR_CONTROLLER)
        {
          printf("  Controller error(s): ");
          if (msg->cm_data[1] & CAN_ERROR1_RXOVERFLOW)
            {
              printf("RX overflow, ");
            }
          if (msg->cm_data[1] & CAN_ERROR1_TXOVERFLOW)
            {
              printf("TX overflow, ");
            }
          if (msg->cm_data[1] & CAN_ERROR1_RXWARNING)
            {
              printf("RX warning level, ");
            }
          if (msg->cm_data[1] & CAN_ERROR1_TXWARNING)
            {
              printf("TX warning level, ");
            }
          if (msg->cm_data[1] & CAN_ERROR1_RXPASSIVE)
            {
              printf("RX passive level, ");
            }
          if (msg->cm_data[1] & CAN_ERROR1_TXPASSIVE)
            {
              printf("TX passive level");
            }
          if (!msg->cm_data[1])
            {
              printf("Unspecified");
            }
          printf("\n");
        }
      if (msg->cm_hdr.ch_id & CAN_ERROR_PROTOCOL)
        {
          printf("  Protocol error(s): ");
          if (msg->cm_data[2] & CAN_ERROR2_BIT)
            {
              printf("Single bit error, ");
            }
          if (msg->cm_data[2] & CAN_ERROR2_FORM)
            {
              printf("Framing format, ");
            }
          if (msg->cm_data[2] & CAN_ERROR2_STUFF)
            {
              printf("Bit-stuffing error, ");
            }
          if (msg->cm_data[2] & CAN_ERROR2_BIT0)
            {
              printf("Send dominant failed, ");
            }
          if (msg->cm_data[2] & CAN_ERROR2_BIT1)
            {
              printf("Send recessive failed, ");
            }
          if (msg->cm_data[2] & CAN_ERROR2_OVERLOAD)
            {
              printf("Bus overload, ");
            }
          if (msg->cm_data[2] & CAN_ERROR2_ACTIVE)
            {
              printf("Active error announcement, ");
            }
          if (msg->cm_data[2] & CAN_ERROR2_TX)
            {
              printf("General TX error");
            }
          if (!msg->cm_data[2])
            {
              printf("Unspecified");
            }
          printf("\n");
        }
      if (msg->cm_hdr.ch_id & CAN_ERROR_TRANSCEIVER)
        {
          printf("  Transceiver error\n");
        }
      if (msg->cm_hdr.ch_id & CAN_ERROR_NOACK)
        {
          printf("  No ACK received\n");
        }
      if (msg->cm_hdr.ch_id & CAN_ERROR_BUSOFF)
        {
          printf("  Bus off\n");
        }
      if (msg->cm_hdr.ch_id & CAN_ERROR_BUSERROR)
        {
          printf("  Bus error\n");
        }
      if (msg->cm_hdr.ch_id & CAN_ERROR_RESTARTED)
        {
          printf("  Controller restarted\n");
        }
      if (msg->cm_hdr.ch_id & CAN_ERROR_INTERNAL)
        {
          printf("  Stack-internal error\n");
        }
    }
  else
    {
      printf("Not an error frame.\n");
    }

  fflush(stdout);
}
#endif /* CONFIG_CAN_ERRORS */

/****************************************************************************
 * Name: print_canmsgs
 *
 * Description:
 *   Treats the buffer as a packed array of struct can_msg_s. (i.e. unused
 *   data bits from the first struct are part of the next struct.
 *
 * Input paramters:
 *   msgs - Packed array of struct can_msg_s returned by read().
 *   buflen - Number of bytes returned by read() (not the actual size of the
 *            buffer)
 ****************************************************************************/

static void print_canmsgs(uint8_t *msgs, int buflen)
{
  int offset = 0;
  struct can_msg_s *msg;

  while (true)
    {
      msg = (struct can_msg_s *)((uint8_t *)msgs + offset);
      offset += CAN_MSGLEN(msg->cm_hdr.ch_dlc);

      /* Offset is the position of the first byte of the next frame in the
       * buffer, so it is also the length of the messages read so far.
       */

      if (offset > buflen)
        {
          break;
        }

#ifdef CONFIG_CAN_ERRORS
      if (msg->cm_hdr.ch_error)
      {
        print_errframe(msg);
        continue;
      }
#endif

      if (msg->cm_hdr.ch_rtr)
        {
          printf("RMT ");
        }
      else
        {
          printf("DAT ");
        }

#ifdef CONFIG_CAN_EXTID
      if (msg->cm_hdr.ch_extid)
      {
        printf("EXT ");
      }
      else
      {
        printf("STD ");
      }
#else
      printf("STD ");
#endif

      printf("ID (dec): %lu DLC (dec): %hhu",
            msg->cm_hdr.ch_id, msg->cm_hdr.ch_dlc);

      if (! msg->cm_hdr.ch_rtr)
        {
          printf(" DATA (hex):");
          for (int i = 0; i < msg->cm_hdr.ch_dlc; ++i)
          {
            printf(" %x", msg->cm_data[i]);
          }
        }

      printf("\n");
      fflush(stdout);
    }
}

/****************************************************************************
 * Name: test_basic_receive
 *
 * Description:
 *   Prints CAN messages until the user types "q"
 *
 * Input parameters:
 *   canfd - Open file descriptor for the CAN device (not related to FDCAN).
 ****************************************************************************/

static void test_basic_receive(const int canfd)
{
  struct pollfd fds[] = {
    {.fd = canfd,         .events = POLLIN, .revents = 0},
    {.fd = STDIN_FILENO,  .events = POLLIN, .revents = 0}
  };
  int ret;
  struct can_msg_s msgbuf;
  char input;
  static int cnt = 0;

  printf("Listening for CAN frames. Type Q to quit.\n");
  while (true)
    {
      ret = poll(fds, 2, -1);
      if (ret < 0)
        {
          printf("poll() failed: %d\n", errno);
          break;
        }
      else if (ret == 0)
        {
          printf("poll() returned 0 unexpectedly.\n");
          break;
        }
      else
        {
          ++cnt;

          for (int i = 0; i < 2; ++i)
            {
              if (fds[i].revents & ~POLLIN)
                {
                  printf("poll() set unexpected flags %x on fd %d",
                        fds[i].revents, fds[i].fd);
                  break;
                }
            }

          if (fds[0].revents & POLLIN)
            {
              ret = read(canfd, &msgbuf, sizeof(struct can_msg_s));
              if (ret < CAN_MSGLEN(0) || ret > sizeof(struct can_msg_s))
                {
                  printf("read() of CAN device returned %d, \n", ret);
                  if (ret < 0)
                    {
                      printf("errno is %d\n", errno);
                    }
                  break;
                }
              else
                {
                  print_canmsgs((uint8_t *)&msgbuf, ret);
                }
            }
          if (fds[1].revents & POLLIN)
            {
              ret = read(STDIN_FILENO, &input, sizeof(char));
              if (ret != 1)
                {
                  printf("read() of stdin returned %d, \n", ret);
                  if (ret < 0)
                    {
                      printf("errno is %d\n", errno);
                    }
                  break;
                }
              else
              {
                if (input == 'Q' || input == 'q')
                  {
                    printf("Quit.\n");
                    break;
                  }
              }
            }
        }
    }
}

/****************************************************************************
 * Name: test_add_std_filter
 *
 * Description:
 *   Allows user to add filters (this removes the default accept-anything
 *   filter)
 *
 * Input parameters:
 *   canfd - Open file descriptor for the CAN device (not related to FDCAN)
 ****************************************************************************/

static void test_add_std_filter(int canfd)
{
  int ret;
  char selection[13] = {0};
  struct canioc_stdfilter_s filter;

  fputs("Standard mask filter: enter a message ID as 1's and 0's with\n"
        "don't cares represented by X's. (example: 11X001X)\n"
        "Unspecified bits are treated as must-match leading zeros.\n", stdout);

  while (true)
    {
      fputs("Enter a filter, or Q to quit: ", stdout);
      fflush(stdout);
      std_readline(selection, 13);

      if (strcmp(selection, "Q\n") == 0 || strcmp(selection, "q\n") == 0)
        {
          puts("Quit");
          break;
        }
      else
        {
          ret = parse_mask(selection, &filter, false);
          if (ret < 0)
            {
              printf("Error parsing mask: %d\n", errno);
            }

          ret = ioctl(canfd, CANIOC_ADD_STDFILTER, &filter);
          if (ret < 0)
            {
              printf("Error adding filter: %d\n", errno);
            }
          else
            {
              printf("Added filter %d.\n", ret);
            }
        }
    }
}

/****************************************************************************
 * Name: test_add_ext_filter
 *
 * Description:
 *   Allows the user to add filters (this removes the default accept-antyhing
 *   filter)
 *
 * Input parameters:
 *   canfd - Open file descriptor for the CAN device (not related to FDCAN)
 ****************************************************************************/

#ifdef CONFIG_CAN_EXTID
static void test_add_ext_filter(int canfd)
{
  int ret;
  char selection[31] = {0};
  struct canioc_extfilter_s filter;

  fputs("Extended mask filter: enter a message ID as 1's and 0's with\n"
        "don't cares represented by X's (example: 11X001X) \n"
        "Unspecified bits are treated as must-match leading zeros.\n", stdout);

  while (true)
    {
      fputs("Enter a filter, or Q to quit: ", stdout);
      fflush(stdout);
      std_readline(selection, 13);

      if (strcmp(selection, "Q\n") == 0 || strcmp(selection, "q\n") == 0)
        {
          puts("Quit");
          break;
        }
      else
        {
          ret = parse_mask(selection, &filter, true);
          if (ret < 0)
            {
              printf("Error parsing mask: %d\n", errno);
              continue;
            }

          ret = ioctl(canfd, CANIOC_ADD_EXTFILTER, &filter);
          if (ret < 0)
            {
              printf("Error adding filter: %d\n", errno);
            }
          else
            {
              printf("Added filter %d.\n", ret);
            }
        }
    }
}
#endif

/****************************************************************************
 * Name: test_rtr_transaction
 *
 * Description:
 *
 * Input parameters:
 *   canfd - Open file descriptor for the CAN device (not related to FDCAN).
 *
 ****************************************************************************/

static void test_rtr_transaction(int canfd)
{
  int ret;
  long id;
  char selection[31] = {0};
  struct can_msg_s    xpectd_msg;
  struct canioc_rtr_s rmt_req = {
    .ci_timeout = {
      .tv_sec = 10,
      .tv_nsec = 1000
    },
    .ci_msg = &xpectd_msg
  };

  while (true)
    {
      fputs("Enter X for extended ID, S for standard ID, followed by the\n"
            "message ID in decimal. (e.g. S1520) Enter Q to quit: ", stdout);
      fflush(stdout);
      std_readline(selection, 31);

      switch (selection[0])
      {
        case 'X':
#ifdef CONFIG_CAN_EXTID
          xpectd_msg.cm_hdr.ch_extid = true;
          break;
#else
          puts("Extended ID support disabled in this build.");
          continue;
#endif
        case 'S':
#ifdef CONFIG_CAN_EXTID
          xpectd_msg.cm_hdr.ch_extid = false;
#endif
          break;
        case 'Q':
        case 'q':
          puts("Quit");
          return;
        default:
          puts("Unexpected starting character.");
          continue;
      }

      if (selection[1] < '0' || selection[1] > '9')
        {
          puts("Not a valid message ID.");
          continue;
        }

      id = atol(selection + 1);

#ifdef CONFIG_CAN_EXTID
      if (!xpectd_msg.cm_hdr.ch_extid && id > CAN_MAX_STDMSGID)
        {
          puts("Standard message ID too large.");
          continue;
        }
      else if (xpectd_msg.cm_hdr.ch_extid && id > CAN_MAX_EXTMSGID)
        {
          puts("Extended message ID too large.");
          continue;
        }
#else
      if (id > CAN_MAX_STDMSGID)
      {
        puts("Standard message ID too large.");
        continue;
      }
#endif

      xpectd_msg.cm_hdr.ch_id = id;

      fputs("Enter the data length code to request in decimal,\n"
            "or Q to quit: ", stdout);
      fflush(stdout);
      std_readline(selection, 31);

      if (selection[0] < '0' || selection[0] > '9')
      {
        if (selection[0] == 'q' || selection[0] == 'Q')
          {
            return;
          }
        else
          {
            puts("Not a valid DLC");
            continue;
          }
      }

      xpectd_msg.cm_hdr.ch_dlc = atoi(selection);

      if (xpectd_msg.cm_hdr.ch_dlc > 8)
      {
        puts("DLC too large.");
        continue;
      }

      puts("Sending request and waiting for response...");
      fflush(stdout);
      ret = ioctl(canfd, CANIOC_RTR, &rmt_req);

      if (ret < 0)
      {
        printf("Request failed: %d\n", errno);
      }
      else
      {
        puts("Response received:");
        print_canmsgs((uint8_t *)&xpectd_msg,
                      CAN_MSGLEN(xpectd_msg.cm_hdr.ch_dlc));
      }
    }
}

/****************************************************************************
 * Name: parse_mask
 *
 * Description:
 *   Converts a string like 11X001X into an ID and mask in the passed filter
 *   struct. If the string is shorter than the number of bits in the message
 *   ID, then it is the same as if they had been explicitly specified as 0's.
 *
 * Input parameters:
 *   msk      - User input string
 *   fltr_ptr - Pointer to either struct canioc_stdfilter_s or
 *              struct canioc_extfilter_s
 *   extended - Whether to interpret the struct and the filter as extended or
 *              standard
 *
 * Return value: 0 on success, -1 with errno set if an error occurred.
 ****************************************************************************/

static int parse_mask(const char *msk, void *fltr_ptr, bool extended)
{
  int i;
  struct canioc_stdfilter_s *sfilter = fltr_ptr;
  if (!extended)
    {
      memset(sfilter, 0, sizeof(struct canioc_stdfilter_s));
      sfilter->sf_type = CAN_FILTER_MASK;
    }

#ifdef CONFIG_CAN_EXTID
  struct canioc_extfilter_s *xfilter = fltr_ptr;
  if (extended)
    {
      memset(xfilter, 0, sizeof(struct canioc_extfilter_s));
      xfilter->xf_type = CAN_FILTER_MASK;
    }
#else
  if (extended)
    {
      errno = EINVAL;
      return -1;
    }
#endif

  if (msk[0] == '\0' || msk[0] == '\r' || msk[0] == '\n')
    {
      puts("No filter given.");
      errno = EINVAL;
      return -1;
    }

  for (i = 0; msk[i] != '\0' && msk[i] != '\r' && msk[i] != '\n'; ++i)
    {
      if ((extended && i > 28) || (!extended && i > 10))
      {
        puts("Unexpected extra characters at end of filter.");
        errno = EINVAL;
        return -1;
      }

      switch (msk[i])
        {
          case '1':
              if (extended)
                {
#ifdef CONFIG_CAN_EXTID
                  xfilter->xf_id1 |= 1 << (28 - i);
#endif
                }
              else
                {
                  sfilter->sf_id1 |= 1 << (10 - i);
                }

              /* id2 stores the mask... the bit must be set regardless whether
              * this is a 0 or a 1, so fall through.
              */

          case '0':
              if (extended)
                {
#ifdef CONFIG_CAN_EXTID
                  xfilter->xf_id2 |= 1 << (28 - i);
#endif
                }
              else
                {
                  sfilter->sf_id2 |= 1 << (10 - i);
                }

              break;

          case 'X':
          case 'x':
              /* Don't care - leave mask bit as zero */
              break;

          default:
              printf("Unexpected character in filter: %c\n", msk[i]);
              errno = EINVAL;
              return -1;
        }
    }

  if (extended)
    {
#ifdef CONFIG_CAN_EXTID
      /* Shift right if the user didn't type out all 29 bits. */
      xfilter->xf_id1 >>= (29 - i);
      xfilter->xf_id2 >>= (29 - i);

      /* Require the ID to match 0's for unspecified most-significant bits. */
      xfilter->xf_id2 |= (0xffffffff << i);
#endif
    }
  else
    {
      /* Shift right if the user the user didn't type out all 29 bits. */
      sfilter->sf_id1 >>= (11 - i);
      sfilter->sf_id2 >>= (11 - i);

      /* Require the ID to match 0's for unspecifed most-significant bits. */
      sfilter->sf_id2 |= (0xffffffff << i);
    }

  return OK;
}

/*****************************************************************************
 * Name: test_del_filter
 *
 * Description:
 *   Prompts the user to select a filter number to delete.
 *
 * Input parameters:
 *   canfd - Open file descriptor for the CAN character device
 *   extended - Whether to use CANIOC_DEL_EXTFILTER or CANIOC_DEL_STDFILTER
 ****************************************************************************/

static int test_del_filter(int canfd, bool extended)
{
  int ret;
  char selection[5] = {0};

#ifndef CONFIG_CAN_EXTID
  if (extended)
  {
    puts("Extended ID support disabled in this build.");
    errno = EINVAL;
    return -1;
  }
#endif

  fputs("Delete ", stdout);

  if (extended)
    fputs("extended", stdout);
  else
    fputs("standard", stdout);

  fputs(" filter: enter the filter number returned when the\n"
        "filter was added.\n", stdout);

  while (true)
    {
      fputs("Enter a filter number, or Q to quit: ", stdout);
      fflush(stdout);
      std_readline(selection, 5);

      if (strcmp(selection, "Q\n") == 0 || strcmp(selection, "q\n") == 0)
        {
          puts("Quit");
          ret = 0;
          break;
        }
      else
        {
          ret = atoi(selection);

#ifdef CONFIG_CAN_EXTID
          if (extended)
            ret = ioctl(canfd, CANIOC_DEL_EXTFILTER, ret);
          else
#endif
            ret = ioctl(canfd, CANIOC_DEL_STDFILTER, ret);

          if (ret < 0)
            {
              printf("Error deleting filter: %d\n", errno);
            }
          else
            {
              puts("Deleted filter.");
            }
        }
    }

  return ret;
}

/****************************************************************************
 * Name: test_tx_burst
 *
 * Description:
 *   Sends a series of 17 messages as quickly as possible so that the TX
 *   rate of the hardware can be measured.
 *
 * Input parameters:
 *   canfd - Open file descriptor for the CAN character device
 ****************************************************************************/

static void test_tx_burst(int canfd)
{
  uint8_t msgs[17 * CAN_MSGLEN(8)];
  struct can_msg_s *ptr;
  int ret;

  for (int i = 0; i < 17; ++i)
  {
    ptr = (struct can_msg_s *)(msgs + i * CAN_MSGLEN(8));
    ptr->cm_hdr.ch_id = i;
    ptr->cm_hdr.ch_dlc = 8;
    ptr->cm_hdr.ch_extid = 1;
  }
  ret = write(canfd, msgs, 17 * CAN_MSGLEN(8));
  printf("%d bytes written\n", ret);
  fflush(stdout);
}

/****************************************************************************
 * Name: test_poll_bug
 *
 * Description:
 *   Continually blocks in read() on the CAN device file to demonstrate the
 *   poll setup bug.
 ****************************************************************************/

static void *test_poll_bug(void *arg)
{
  int canfd = (int)arg;
  struct can_msg_s msg;
  while (true)
    {
      read(canfd, &msg, sizeof(struct can_msg_s));
    }
  
  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: main
 *
 * Description:
 *   cantest main function
 *
 ****************************************************************************/

int main(int argc, char **argv)
{
  /* For getopt_long */
  int opt;
  int opt_idx = 0;
  const char short_opts[] = "hd:";
  static const struct option long_opts[] =
    {
      { "help", no_argument,        NULL, 'h' },
      { "dev",  required_argument,  NULL, 'd' },
      { 0, 0, 0, 0}
    };

  uint32_t flags = 0;
  char   dev[10] = "/dev/";
  int         fd;
  int         ret;
  int         exitcode = OK;

  while (-1 != (opt = getopt_long(argc, argv, short_opts, long_opts, &opt_idx)))
    {
      switch(opt)
        {
          case 'h':
            flags |= FLAG_HELP;
            break;
          case 'd':
            strncpy(dev, optarg, 9);
            break;
          case '?':
            if (optopt)
                printf("Unrecognized option \"%c.\"\n", optopt);
            else
                printf("Unrecognized option \"%s.\"\n", argv[optind - 1]);

            flags |= FLAG_UNRECOGNIZED;
            break;
          default:
            flags|= FLAG_GETOPT_ERR;
            break;
        }
    }

  if (optind < argc)
    {
      printf("Unrecognized extra arguments given.");
      flags |= FLAG_UNRECOGNIZED;
    }

  if (flags & FLAG_HELP)
    {
      print_help();
    }
  else if (flags & FLAG_UNRECOGNIZED)
    {
      printf("Use --help for a list of options.\n");
    }

  if (flags & FLAG_GETOPT_ERR)
    {
      printf("Error with getopt.\n");
    }

  if (flags & FLAG_UNRECOGNIZED || flags & FLAG_GETOPT_ERR)
    return EINVAL;
  else if (flags & FLAG_HELP)
    return OK;

  /* CAN device selection ***************************************************/
  if (strncmp(dev, "/dev/", 9) == 0)
    {
      struct dirent **devs;
      int numdevs;
      numdevs = scandir("/dev", &devs, filter_candevs, alphasort);

      if (numdevs == -1)
        {
          printf("Error searching for CAN devices in /dev: %d\n", errno);
          return errno;
        }
      else if (numdevs == 0)
        {
          printf("No CAN devices found in /dev.\n");
          return ENODEV;
        }
      else
        {
          strncat(dev, devs[0]->d_name, 4);

          for (int i = 0; i < numdevs; ++i)
            {
              free(devs[i]);
            }
          free(devs);
        }
    }

  /* Opening the CAN device *************************************************/
  printf("Opening CAN device %s.\n", dev);
  fd = open(dev, O_RDWR);

  if (fd < 0)
    {
      printf("Error opening CAN device %s: %d\n", dev, errno);
      return errno;
    }

  while (true)
    {
      char selection[3] = {0};


      printf("Type Q to quit or select a test to run:\n"
             " 1. Basic receive\n"
             " 2. Print setup info\n"
             " 3. Add standard filters\n"
             " 4. Add extended filters\n"
             " 5. Remove standard filters\n"
             " 6. Remove extended filters\n"
             " 7. Perform a remote-request-response transaction\n"
             " 8. Send a burst of TX messages\n"
             " 9. Test for can_poll() bug\n"
             "\n\n");

      fputs("Please select an option (1/2/3/4/5/6/7/8/9/Q): ", stdout);
      fflush(stdout);
      ret = std_readline(selection, 3);

      if (ret < 0)
      {
        printf("Readline error, exiting\n");
        break;
      }
      else if (strcmp(selection, "Q\n") == 0 || strcmp(selection, "q\n") == 0)
      {
        printf("Quit.\n");
        break;
      }
      else if (strcmp(selection, "1\n") == 0)
      {
        test_basic_receive(fd);
      }
      else if (strcmp(selection, "2\n") == 0)
      {
        puts("Not implemented yet.");
      }
      else if (strcmp(selection, "3\n") == 0)
      {
        test_add_std_filter(fd);
      }
      else if (strcmp(selection, "4\n") == 0)
      {
#ifdef CONFIG_CAN_EXTID
        test_add_ext_filter(fd);
#else
        puts("Extended ID support was disabled in this build.");
#endif
      }
      else if (strcmp(selection, "5\n") == 0)
      {
        test_del_filter(fd, false);
      }
      else if (strcmp(selection, "6\n") == 0)
      {
        test_del_filter(fd, true);
      }
      else if (strcmp(selection, "7\n") == 0)
      {
        test_rtr_transaction(fd);
      }
      else if (strcmp(selection, "8\n") == 0)
      {
        test_tx_burst(fd);
      }
      else if (strcmp(selection, "9\n") == 0)
      {
        pthread_t poll_test_thd;
        sem_init(&g_poll_test_sem, 0, 0);
        pthread_create(&poll_test_thd, NULL, test_poll_bug, (void *)fd);
        pthread_yield();
        test_basic_receive(fd);
      }
      else
      {
        printf("Invalid selection.\n");
      }
    }

  ret = close(fd);
  if (ret < 0)
    {
      printf("Error closing CAN device: %d\n", errno);
    }

  return exitcode;
}
