// ScaleTime.c
// Fast linear interpolation
// This is a fast replacement for:
//   Xi = interp1(1:size(X, 1), X, Ti, 'linear');
// See ScaleTime.m for a detailed description.
//
// Xi = ScaleTime(X, Ti)
// Xi = ScaleTime(X, initial, final, number)
//
// Compile with:
//   mex -O ScaleTime.c
//
// Tested: Matlab 6.5, 7.7, BCC5.5, LCC2.4/3.8, WinXP
// Author: Jan Simon, Heidelberg, (C) 2009 J@n-Simon.De

/*
% $JRev: R0b V:002 Sum:E8E22C5D Date:30-Sep-2009 16:15:47 $
% $File: Tools\Mex\Source\ScaleTime.c $
% History:
% 001: 29-Sep-2009 09:20, MEX version of ScaleTime, new index input.
*/

#include "mex.h"
#include <stdlib.h>
#include <math.h>

mxArray *CreateOutput(int *MX, int *NX, int numTI);

void CoreFull(double *X, int MX, int nRow,
              int iniT, int finT, int numT,
              int fullStep, double fracStep, double iniFrac, double *R);

void CoreFrac(double *X, int MX, int nRow,
              int iniT, int finT, int numT,
              int dummy, double fracStep, double iniFrac, double *R);

void CoreVec(double *X, int MX, int NX, double *T, int nT, double *R);

// Main function ===============================================================
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  int           MX, NX, numTI, doTranspose = 0;
  double        iniTD, finTD, numTD, fracStepD, fullStepD, iniFracD;
  const mxArray *X;

  // Proper number of arguments
  if (nlhs > 1) {
    mexErrMsgTxt("*** ScaleTime[mex]: 1 output allowed.");
  }

  // Get input matrix:
  X  = prhs[0];
  MX = mxGetM(X);  // (M x N) matrix
  NX = mxGetN(X);
  if (!mxIsDouble(X) || mxIsComplex(X)) {
    mexErrMsgTxt("*** ScaleTime[mex]: Input array must be a real matrix.");
  }

  // Either one vector or 2 scalars to specify the interpolation times:
  if (nrhs == 4) {
    // 3 scalars: Ti = iFrame:((fFrame - iFrame) / (N - 1)):fFrame -------------
    if (mxGetNumberOfElements(prhs[1]) != 1 || !mxIsDouble(prhs[1]) ||
        mxGetNumberOfElements(prhs[2]) != 1 || !mxIsDouble(prhs[2]) ||
        mxGetNumberOfElements(prhs[3]) != 1 || !mxIsDouble(prhs[3])) {
      mexErrMsgTxt("*** ScaleTime[mex]: " \
                   "3 scalars required as input to specify frames.");
    }

    iniTD = mxGetScalar(prhs[1]);
    finTD = mxGetScalar(prhs[2]);
    numTD = floor(mxGetScalar(prhs[3]) + 0.5);  // rounding
    numTI = (int)numTD;

    // Reply the empty matrix if no interpolation steps are wanted:
    if (numTI < 1 || iniTD > finTD) {
      plhs[0] = mxCreateDoubleMatrix(0, 0, mxREAL);
      return;
    }

    // Calculate step size and fractional part of first point:
    if (numTI > 1) {
      fracStepD = modf((finTD - iniTD) / (numTD - 1.0), &fullStepD);
    } else {
      fracStepD = 0.0;
      fullStepD = 0.0;
    }
    iniFracD = fmod(iniTD, 1.0);

    // Create output matrix and interpolate:
    plhs[0] = CreateOutput(&MX, &NX, numTI);

    // Check limits:
    if (iniTD < 1.0 || (int)finTD > MX) {
      mexErrMsgTxt("*** ScaleTime[mex]: Interpolation frames out of range.");
    }

    if (fullStepD != 0.0) {  // Step size >= 1.0:
      CoreFull(mxGetPr(X), MX, NX, (int)iniTD, (int)finTD, numTI,
               (int)fullStepD, fracStepD, iniFracD, mxGetPr(plhs[0]));

    } else {                 // Step size < 1.0:
      CoreFrac(mxGetPr(X), MX, NX, (int)iniTD, (int)finTD, numTI,
               0, fracStepD, iniFracD, mxGetPr(plhs[0]));
    }

  } else if (nrhs == 2) {  // Ti as vector: ------------------------------------
    numTI = mxGetNumberOfElements(prhs[1]);
    if (numTI != 0) {
      // Create a row vector or a matrix as output:
      plhs[0] = CreateOutput(&MX, &NX, numTI);

      CoreVec(mxGetPr(X), MX, NX, mxGetPr(prhs[1]), numTI, mxGetPr(plhs[0]));

    } else {
      plhs[0] = mxCreateDoubleMatrix(0, 0, mxREAL);
    }

  } else {
    mexErrMsgTxt("*** ScaleTime[mex]: 2 or 4 inputs required.");
  }

  return;
}

// *****************************************************************************
mxArray *CreateOutput(int *MX, int *NX, int numTI)
{
  // Transpose if X is a row vector:
  if (*MX == 1 && *NX > 1) {
    *MX = *NX;
    *NX = 1;
    return mxCreateDoubleMatrix(1, numTI, mxREAL);
  } else {
    return mxCreateDoubleMatrix(numTI, *NX, mxREAL);
  }
}

// *****************************************************************************
void CoreFull(double *X, int MX, int NX,
          int iniT, int finT, int numT,
          int fullStep, double fracStep, double iniFrac, double *R)
{
  // Method for step size greater than 1.0. Then the step is divided in an
  // integer and a fractional part. While the integer part determines the index
  // of the elements of X, the fractional part determines the weight between the
  // neighboring values.

  double *Xp, frac;
  int    iT, iN;

  for (iN = 0; iN < NX; iN++) {
    Xp   = X + iniT - 1;
    frac = iniFrac;
    for (iT = 1; iT < numT; iT++) {  // Start from 1: Last T on boundary?
      *R++  = *Xp * (1.0 - frac) + *(Xp + 1) * frac;
      Xp   += fullStep;
      if ((frac += fracStep) >= 1.0) {
        Xp++;
        frac -= 1.0;
      }
    }

    // Last step on the boundary?
    if (Xp - X == MX - 1) {
      *R++ = *Xp;
    } else {
      *R++  = *Xp * (1.0 - frac) + *(Xp + 1) * frac;
    }
    X += MX;
  }

  return;
}

// *****************************************************************************
void CoreFrac(double *X, int MX, int NX,
          int iniT, int finT, int numT,
          int dummy, double fracStep, double iniFrac, double *R)
{
  // Method for step size lower than 1.0.
  // Same as for >= 1.0 except for the omitted line "Xp += fullStep;".
  // This is not remarkably faster for LCC and BCC, but perhaps modern compilers
  // can benefit from this.
  //
  // To my surprise, this function works 3 times faster, if the [dummy] argument
  // is defined in the input - for BCC5.5. Ironically, for the LCC3.8 and LCC2.4
  // the situation was the other way around. If somebody could explain this,
  // please contact me!

  double *Xp, frac;
  int    iT, iN;

  for (iN = 0; iN < NX; iN++) {
    Xp   = X + iniT - 1;
    frac = iniFrac;
    for (iT = 1; iT < numT; iT++) {  // Start from 1: Last T on boundary?
      *R++ = *Xp * (1.0 - frac) + *(Xp + 1) * frac;
      if ((frac += fracStep) >= 1.0) {
        Xp++;
        frac -= 1.0;
      }
    }

    // Last step on the boundary?
    if (Xp - X == MX - 1) {
      *R++ = *Xp;
    } else {
      *R++  = *Xp * (1.0 - frac) + *(Xp + 1) * frac;
    }

    X += MX;
  }
}

// *****************************************************************************
void CoreVec(double *X, int MX, int NX, double *T, int nT, double *R)
{
  // Interpolation frames as vector.
  // Only the first and last elements of T are tested for exceeding the limits.

  double *Tp, *TEnd, *fracList, *fracP, floorT;
  int iN, *indexList, *indexP, *indexEnd;

  if (*T < 1.0 || *(T + nT - 1) > MX) {
    mexErrMsgTxt("*** ScaleTime[mex]: Interpolation frames out of range.");
  }
  if ((indexList = (int *)mxMalloc(nT * sizeof(int))) == NULL) {
    mexErrMsgTxt("*** ScaleTime[mex]: Cannot get memory for index list.");
  }
  if ((fracList = (double *)mxMalloc(nT * sizeof(double))) == NULL) {
    mexErrMsgTxt("*** ScaleTime[mex]: Cannot get memory for fractions list.");
  }

  // First row of X - store the indices and fractions:
  TEnd   = T + (nT - 1);  // Last T needs checking of boundary
  fracP  = fracList;
  indexP = indexList;
  for (Tp = T; Tp < TEnd; Tp++) {
    floorT  = floor(*Tp);
    *indexP = (int) floorT;
    *fracP  = *Tp - floorT;
    *R++    = X[*indexP - 1] * (1.0 - *fracP) + X[*indexP] * *fracP;
    fracP++;
    indexP++;
  }

  // Check if last interpolation frame is on the boundary:
  floorT = floor(*Tp);
  if (floorT == MX) {
    *indexP = (int) floorT - 1;
    *fracP  = 1.0;
    *R++    = X[*indexP];
  } else {  // Last frame not on boundary:
    *indexP = (int) floorT;
    *fracP  = *Tp - floorT;
    *R++    = X[*indexP - 1] * (1.0 - *fracP) + X[*indexP] * *fracP;
  }

  // 2nd to final row:
  indexEnd = indexList + nT;
  for (iN = 1; iN < NX; iN++) {
    X     += MX;
    fracP  = fracList;
    for (indexP = indexList; indexP < indexEnd; indexP++) {
      *R++ = X[*indexP - 1] * (1.0 - *fracP) + X[*indexP] * *fracP;
      fracP++;
    }
  }

  mxFree(indexList);
  mxFree(fracList);

  return;
}
