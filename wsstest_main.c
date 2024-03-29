/****************************************************************************
 * apps/industry/ETCetera-tools/wsstest_main.c
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
  int ret = 0;
  printf("Enabling wheel speed feeds.\n");
  ret = boardctl(BOARDIOC_WSS_ENABLE, 0);
  return ret;
}
