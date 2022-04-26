/****************************************************************************
 * apps/industry/ETCetera-tools/cantest_main.c
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

#include <sys/boardctl.h>
#include <arch/board/board.h>

#include "system/readline.h"

int main(int argc, char **argv)
{
  int ret;
  while(true)
    {
      char selection[3] = {0};
      
      printf("Type Q to quit or select a test to run:\n"
             " 1. Command 0 deg position (500us pulse)\n"
             " 2. Command 90 deg position (1100us pulse)\n"
             " 3. Command 180 deg position (1700us pulse)\n"
             "\n\n");
      
      fputs("Please select an option (1/2/3/Q): ", stdout);
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
          boardctl(BOARDIOC_DRS_ANGLE, 0);
          boardctl(BOARDIOC_DRS_START, 0);
        }
      else if (strcmp(selection, "2\n") == 0)
        {
          boardctl(BOARDIOC_DRS_ANGLE, 90);
          boardctl(BOARDIOC_DRS_START, 0);
        }
      else if (strcmp(selection, "3\n") == 0)
        {
          boardctl(BOARDIOC_DRS_ANGLE, 180);
          boardctl(BOARDIOC_DRS_START, 0);
        }
      else
        {
          printf("Invalid selection.\n");
        }
    }
    
    boardctl(BOARDIOC_DRS_STOP, 0);
    return ret;
}
