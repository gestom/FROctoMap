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
#define MAX_OUTLIERS   40000
#define MAX_FRELEMENTS 15 

using namespace std;

typedef struct
{
	float amplitude;
	float phase;
	unsigned int frequency;
}SFrelement;

class CFrelement
{
public:
  CFrelement();
  ~CFrelement();

  void reconstruct(unsigned char* signal,int signalLength);

  /*state estimation: retrieves the state*/
  float estimate(int timeStamp);

  /*state estimation: retrieves the state*/
  unsigned char retrieve(int timeStamp);

  /*fills with values*/
  void fill(unsigned char values[],int number);

  /*gets length in terms of values measured*/
  int getLength();

  /*gets length in terms of values measured*/
  void add(unsigned char value);

  void build(unsigned char* signal,int signalLength,int modelOrder);

  /*clears all*/
  void clear();
  void print();

  /*changes the model order*/
  void update(int number);

  double *signal;

private:
	std::vector<SFrelement> frelements;
	std::vector<unsigned int> outlierSet;
	int signalLength;
	float gain;
};

#endif
