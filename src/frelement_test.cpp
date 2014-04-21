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
	if (argc != 4 && argc != 5) {
		fprintf(stderr,"frelement_test filename fremen_order num_elements max_signal_length\n");
		return -1;
	}
	int order = atoi(argv[2]);
	int frelements = atoi(argv[3]);
	ifstream file (argv[1],ifstream::in);

	//read the input file
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

	CFrelement *frelement[frelements];
	for (int i=0;i<frelements;i++) frelement[i] = new CFrelement();
	CTimer timer;
	timer.reset();
	timer.start();

	CFFTPlan plan;
	plan.prepare(signalLength);

	cout << "Plan preparation time " << timer.getTime() << " us." << endl; 

	timer.reset();
	for(int i=0;i<frelements;i++){
		 if (i%100 == 0) cout << "Calculating " << i << " of " << frelements << endl;
		 frelement[i]->build(signal,signalLength,&plan);
	}
	cout << "Model build time " << timer.getTime()/frelements/1000 << " ms." << endl;
	timer.reset();
	for(int i=0;i<frelements;i++){
		 if (i%100 == 0) cout << "Updating " << i << " of " << frelements << endl;
		frelement[i]->update(order,&plan);
	}
	cout << "Model update time " << timer.getTime()/frelements/1000 << " ms." << endl;
	frelement[0]->print();
	float eval = frelement[0]->reconstruct(reconstructed,&plan,false);
	for (int i = 0;i<signalLength;i++){
//		 reconstructed[i] = frelement[0]->retrieve(i);
//		 cout << "Reconstructed: " << frelement[0]->estimate(i) << endl;
	}
	//for (int i=0;i<signalLength;i++) cout << (int)signal[i] << (int)reconstructed[i] << endl;
	
	int err = 0;
	int pos = 0;
	for (int i=0;i<signalLength;i++) err+=abs((int)signal[i]-(int)reconstructed[i]);
	for (int i=0;i<signalLength;i++) pos+=(int)signal[i];
	//cout << "Model precision: " << eval << " Lossless: " << err << " " << pos << endl;
	cout << "Model reconstruction: " << err << " " << pos << endl;
	for (int i=0;i<frelements;i++) delete frelement[i];
	free(signal);
	free(reconstructed);
	return 0;
}
