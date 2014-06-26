#ifndef CFREMENGRID_H
#define CFREMENGRID_H

#include "CFrelement.h"
	
/**
@author Tom Krajnik
*/

using namespace std;

class CFremenGrid
{
public:
  CFremenGrid(int size);
  ~CFremenGrid();

  /*state estimation: estimates the state of the i-th element*/
  float estimate(unsigned int index,unsigned int timeStamp);

  /*state estimation: retrieves the state of the i-th element*/
  unsigned char retrieve(unsigned int index,unsigned int timeStamp);

  /*fills with values*/
  void fill(unsigned char values[],unsigned int timeStamp);

  void add(unsigned int index,unsigned char value);

  /*clears all*/
  void clear();
  void print();

  /*sets the octomap's origin*/
  void setPose(float x, float y);
  void setName(const char* n);

  /*changes the model order*/
  void update(int number,int signalLength);
  void reconstruct(int number,unsigned char *reconstructed);
  void print(int number);
  void save(const char*name,bool lossy = false);
  void load(const char*name);
  double *signal;

//private:
	char name[100];
	CFrelement **cellArray;
	int numCells;
	int xDim,yDim,zDim;
	int order;
	unsigned int signalLength;
	CFFTPlan *plan;
	float positionX,positionY;
};

#endif
