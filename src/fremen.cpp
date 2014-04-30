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
	if (argc != 4 && argc != 5) {
		fprintf(stderr,"fremen filename fremen_order grid_cells max_signal_length\n");
		return -1;
	}
	int modelOrder = atoi(argv[2]);
	int gridSize = atoi(argv[3]);
	
	//read the input file
	ifstream file (argv[1],ifstream::in);
	int x;
	while (file.good())
	{
		file >> x;
		signal[signalLength++]=x;
	}
	file.close();
	signalLength--;

	fprintf(stdout,"Signal length: %i ",signalLength);
	if (argc == 5){
		signalLength = atoi(argv[4]);
		fprintf(stdout," manually reduced to %i ",signalLength);
	}
	fprintf(stdout,"\n");
	if (signalLength == 0) {
		fprintf(stdout,"Nothing to calculate - is the file correct ?\n");
		return 1;	
	}

	CTimer timer;
	timer.start();
	CFremenGrid grid(gridSize);
	for(int j=0;j<signalLength;j++)
	{
		for(int i=0;i<gridSize;i++)
		{ 
			grid.add(i,signal[j]);
		}
		if ((j%100)==0) cout << "Filled " << j << endl;
	}
	cout << "Model filled " << endl;
	getc(stdin);
	timer.reset();
	grid.update(modelOrder,signalLength);
	cout << "Model update time " << timer.getTime()/atoi(argv[2])/1000 << " ms." << endl;
	grid.print(0);
	grid.save("Grid-all.grid");
	grid.save("Grid-small.grid",true);

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
