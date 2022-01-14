/**
* Copyright 2018 Woods Hole Oceanographic Institution
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/
#include "ds_util/median.h"

namespace ds_util
{
void sort(int n, double ra[])
{
  int l, j, ir, i;
  double rra;

  l = (n >> 1) + 1;
  ir = n;
  for (;;)
  {
    if (l > 1)
      rra = ra[--l];
    else
    {
      rra = ra[ir];
      ra[ir] = ra[1];
      if (--ir == 1)
      {
        ra[1] = rra;
        return;
      }
    }
    i = l;
    j = l << 1;
    while (j <= ir)
    {
      if (j < ir && ra[j] < ra[j + 1])
        ++j;
      if (rra < ra[j])
      {
        ra[i] = ra[j];
        j += (i = j);
      }
      else
        j = ir + 1;
    }
    ra[i] = rra;
  }
}

double median(double x[], int n)
{
  double store[100];
  int n2, n2p;
  int i;
  double result;
  for (i = 0; i < n; i++)
    store[i + 1] = x[i];
  sort(n, store);
  n2p = (n2 = n / 2) + 1;
  result = (n % 2 ? store[n2p] : 0.5 * (store[n2] + store[n2p]));
  return (result);
}
}
