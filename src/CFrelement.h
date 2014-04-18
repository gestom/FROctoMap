#ifndef CFRELEMENT_H
#define CFRELEMENT_H

#include <iostream>
#include <vector>
#include <complex>	
#include <algorithm> 
#include <iterator> 
#include <complex>	// goes along with fftw
#include <fftw3.h>
#include <string.h>
#include "CTimer.h"
	
/**
@author Tom Krajnik
*/

using namespace std;

typedef struct
{
	float amplitude;
	float phase;
	unsigned int frequency;
}SFrelement;

typedef struct
{
	fftw_plan direct;
	fftw_plan inverse;
	double *probability;
	double *signal;
	fftw_complex *coeffs;	
}SPlan;

class CFrelement
{
public:
  CFrelement(int order=0);
  ~CFrelement();

  void reconstruct(unsigned char* signal,SPlan *plan);

  /*state estimation: retrieves the state*/
  float estimate(int timeStamp);

  /*state estimation: retrieves the state*/
  unsigned char retrieve(int timeStamp);

  /*fills with values*/
  void fill(unsigned char values[],int number);

  /*adds stuff*/
  void add(unsigned char value);

  int getLength();

  void build(unsigned char* signal,int signalLength,int modelOrder,SPlan *plan);

  /*clears all*/
  void clear();
  void print();

  /*changes the model order*/
  void update(int modelOrder,SPlan *plan);

  double *signal;

private:
	SFrelement *frelements;
	unsigned int *outlierSet;
	unsigned int outliers;
	unsigned int order;
	float gain;
	unsigned int signalLength;
};

#endif
