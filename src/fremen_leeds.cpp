#include <iostream>
#include <fstream>	
#include <cstdlib>	
#include "CFrelement.h"
#include "CTimer.h"
#define MAX_SIGNAL_LENGTH 10000000

using namespace std;

int main(int argc,char *argv[])
{
	unsigned char *signal = (unsigned char*)malloc(MAX_SIGNAL_LENGTH);
	unsigned char *reconstructed = (unsigned char*)malloc(MAX_SIGNAL_LENGTH);
	float *estimated = (float *)malloc(MAX_SIGNAL_LENGTH);
	if (argc != 6) {
		fprintf(stderr,"fremen_leeds input_file model_order start_day end_day output_file\n");
		return -1;
	}

	//read the input parameters
	FILE *file=fopen(argv[1],"r");
	int order = atoi(argv[2]);
	int dataStart  = atoi(argv[3]);
	int dataLength = atoi(argv[4]);
	int signalLength = 0;
	//read the input file
	int x;
	if (file != NULL){
		fseek(file,dataStart,SEEK_SET);
		while (feof(file)==0 && signalLength < dataLength-dataStart){	
			fscanf(file,"%i\n",&x);
			signal[signalLength++] = x;
		}
	}else{
		fprintf(stdout,"File %s not found\n",argv[1]);
		return 1;	
	}
	fclose(file);
	fprintf(stdout,"Data of length %i starting at %i read from file %s \n",signalLength,dataStart,argv[1]);
	if (signalLength == 0) {
		fprintf(stdout,"Nothing to calculate - is the file correct ?\n");
		return 1;	
	}

	CFrelement element;
	CTimer timer;
	timer.reset();
	timer.start();

	CFFTPlan plan;
	plan.prepare(signalLength);

	cout << "Plan preparation time " << timer.getTime() << " us." << endl; 

	timer.reset();
	element.build(signal,signalLength,&plan);

	cout << "Model build time " << timer.getTime()/1000 << " ms." << endl;
	timer.reset();
	element.update(order,&plan);

	cout << "Model update time " << timer.getTime()/1000 << " ms." << endl;
	element.print();
	float eval = element.reconstruct(reconstructed,&plan,false);
	element.estimate(estimated,&plan,0);

	file=fopen(argv[5],"w");
	for (int i = 0;i<signalLength;i++) fprintf(file,"%.4f\n",estimated[i]);
	fclose(file);
	//for (int i=0;i<signalLength;i++) cout << (int)signal[i] << (int)reconstructed[i] << endl;
	
	int err = 0;
	int pos = 0;
	for (int i=0;i<signalLength;i++) err+=abs((int)signal[i]-(int)reconstructed[i]);
	for (int i=0;i<signalLength;i++) pos+=(int)signal[i];
	//cout << "Model precision: " << eval << " Lossless: " << err << " " << pos << endl;
	cout << "Model reconstruction: " << err << " " << pos << endl;
	free(signal);
	free(reconstructed);
	free(estimated);
	return 0;
}
