#include <iostream>
#include <fstream>	
#include <cstdlib>	
#include "CFremenGrid.h"
#include "CTimer.h"

using namespace std;

int main(int argc,char *argv[])
{
	unsigned char *signal = (unsigned char*)malloc(10000000);
	unsigned char *reconstructed = (unsigned char*)malloc(10000000);
	int signalLength = 0;
	ifstream file ("/home/gestom/catkin/weekbin.txt",ifstream::in);
	int x;
	while (file.good())
	{
		file >> x;
		signal[signalLength++]=x;
	}
	file.close();
	CTimer timer;
	int gridSize = atoi(argv[2]);
	int modelOrder = atoi(argv[1]);
	signalLength=atoi(argv[3]);

	CFremenGrid grid(gridSize);
	for(int i=0;i<gridSize;i++)
	{
		for(int j=0;j<signalLength;j++)
		{ 
			grid.add(i,signal[j]);
		}
	}
	grid.update(modelOrder,signalLength);
	cout << "Model update time " << timer.getTime()/atoi(argv[2])/1000 << " ms." << endl;
	grid.print(0);
	grid.reconstruct(0,reconstructed);

	int err = 0;
	int pos = 0;
	for (int i=0;i<signalLength;i++) err+=abs((int)signal[i]-(int)reconstructed[i]);
	for (int i=0;i<signalLength;i++) pos+=(int)signal[i];
	cout << "Perfect: " << err << " " << pos << endl;
	free(signal);
	free(reconstructed);
	return 0;
}
