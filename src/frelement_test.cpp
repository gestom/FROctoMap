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
	CFrelement *frelement[atoi(argv[2])];
	for (int i=0;i<atoi(argv[2]);i++) frelement[i] = new CFrelement();
	fprintf(stdout,"Element size: %i B\n",(int)sizeof(CFrelement));

	CTimer timer;
	timer.reset();
	timer.start();
	signalLength=atoi(argv[3]);

	CFFTPlan plan;
	plan.prepare(signalLength);

	cout << "Plan preparation time " << timer.getTime() << " us." << endl; 

	timer.reset();
	for(int i=0;i<atoi(argv[2]);i++){
		 if (i%100 == 0) cout << "Calculating " << i << " of " << atoi(argv[2]) << endl;
		 frelement[i]->build(signal,signalLength,&plan);
	}
	cout << "Model build time " << timer.getTime()/atoi(argv[2])/1000 << " ms." << endl;
	timer.reset();
	for(int i=0;i<atoi(argv[2]);i++){
		 if (i%100 == 0) cout << "Updating " << i << " of " << atoi(argv[2]) << endl;
		frelement[i]->update(atoi(argv[1]),&plan);
	}
	cout << "Model update time " << timer.getTime()/atoi(argv[2])/1000 << " ms." << endl;
	//for (int i=0;i<signalLength;i++) frelement.add(signal[i]);
	frelement[0]->print();
	frelement[0]->reconstruct(reconstructed,&plan);
	
//	timer.reset();
//	for (int i=0;i<signalLength;i++) cout << "Est " << (float) frelement[0]->estimate(i) <<endl; 
//	cout << "Primitive reconstruction " << timer.getTime()/1000 << endl;
 
	int err = 0;
	int pos = 0;
	for (int i=0;i<signalLength;i++) err+=abs((int)signal[i]-(int)reconstructed[i]);
	for (int i=0;i<signalLength;i++) pos+=(int)signal[i];
	cout << "Perfect: " << err << " " << pos << endl;
	free(signal);
	free(reconstructed);
	return 0;
}
