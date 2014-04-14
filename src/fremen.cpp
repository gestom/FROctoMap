#include <iostream>
#include <fstream>	
#include <cstdlib>	
#include "CFrelement.h"
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
	// the code you wish to time goes here
	CFrelement frelement;
	frelement.build(signal,signalLength,atoi(argv[1]));
	//for (int i=0;i<signalLength;i++) frelement.add(signal[i]);
	frelement.print();
	frelement.reconstruct(reconstructed,signalLength);

//	timer.reset();
//	for (int i=0;i<signalLength;i++) frelement.retrieve(i); 
//	cout << "Primitive reconstruction " << timer.getTime()/1000 << endl;
 
	int err = 0;
	for (int i=0;i<signalLength;i++) err+=abs((int)signal[i]-(int)reconstructed[i]);
	cout << "Perfect: " << err << endl;
	return 0;
}
