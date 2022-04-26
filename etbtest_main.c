/****************************************************************************
 * apps/industry/ETCetera-tools/etbtest_main.c
 * Electronic Throttle Controller program - CAN bus test utility
 *
 * Copyright (C) 2022  Matthew Trescott <matthewtrescott@gmail.com>
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
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <sys/boardctl.h>
#include <arch/board/board.h>

#include "system/readline.h"

int main(int argc, char **argv)
{
  int ret;
  printf("Enabling 5V0LIN_SENSE...\n");
  ret = boardctl(BOARDIOC_5V0LIN_SENSE_EN, 0);
  if (ret == OK) 
    {
      printf("5V0LIN_SENSE enabled successfully. Waiting 1 second to arm.\n");
    }
  else
    {
      printf("5V0LIN_SENSE failed to start. Aborting.");
      return ret;
    }
  sleep(1);
  boardctl(BOARDIOC_ARM_READY, 0);
  
  while(true)
    {
      char selection[4] = {0};
      
      printf("Type Q to quit or enter a duty cycle as a percentage:"
             "\n\n");
      
      fputs("Please select an option (duty cyle/Q): ", stdout);
      fflush(stdout);
      ret = std_readline(selection, 4);
      
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
      else
        {
          ret = atoi(selection);
          if (ret <= 0 || ret > 30)
          {
            printf("Invalid duty cycle. Duty cycle may not be >30.\n");
          }
          else
          {
            ret = boardctl(BOARDIOC_ETB_DUTY, ret);
          }
        }
    }
    
    boardctl(BOARDIOC_ETB_STOP, 0);
    return ret;
}
