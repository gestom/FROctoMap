#include "CFrelement.h"

using namespace std;
static bool debug = false; 

bool fremenSort(SFrelement i,SFrelement j) 
{ 
	return (i.amplitude>j.amplitude); 
}

CFrelement::CFrelement()
{
	outlierSet = NULL;
	frelements = NULL;
	signalLength = gain = outliers = order = 0;
}

CFrelement::~CFrelement()
{
	free(frelements);
	free(outlierSet);
}

void CFrelement::build(unsigned char* signal,int signalLengthi,CFFTPlan *plan)
{
	signalLength = signalLengthi;
	int fftLength = signalLength/2+1;
	fftw_complex *coeffs;
	double *probability,*fftSignal;
	CTimer timer;
	timer.start();
	int tim = 0;
	unsigned char *reconstructed = (unsigned char*)malloc(signalLength*sizeof(unsigned char));

	if (order > 0){
		probability = plan->probability;
		fftSignal = plan->signal;
		coeffs = plan->coeffs;

		for (int i = 0;i<signalLength;i++) fftSignal[i] = signal[i];
		timer.reset();
		//cout << "FFT preparation time " << timer.getTime() << endl;

		/*calculation of the spectral model*/
		fftw_execute_dft_r2c(plan->direct,fftSignal,coeffs);
		if (debug) cout << "FFT calculation time " << timer.getTime() << endl;
		timer.reset();
		SFrelement *tmpFrelements = (SFrelement *)malloc(fftLength*sizeof(SFrelement));
		for(int i=0;i<fftLength;i++)
		{
			tmpFrelements[i].amplitude = (coeffs[i][0]*coeffs[i][0]+coeffs[i][1]*coeffs[i][1]);
			tmpFrelements[i].frequency = i;
		}
		partial_sort(tmpFrelements,tmpFrelements+order+1,tmpFrelements+(fftLength-1),fremenSort);
		tim = timer.getTime();
		//sort(tmpFrelements,tmpFrelements+(fftLength-1),fremenSort);

		gain = sqrt(tmpFrelements[0].amplitude)/signalLength;
		for(int i=1;i<order+1;i++){
			frelements[i-1].amplitude = sqrt(tmpFrelements[i].amplitude)/signalLength;
			frelements[i-1].phase = atan2(coeffs[tmpFrelements[i].frequency][1],coeffs[tmpFrelements[i].frequency][0]);
			frelements[i-1].frequency = tmpFrelements[i].frequency;
		}
		free(tmpFrelements);
		tim = timer.getTime();
		if (debug) cout << "Spectrum recovery time " << tim << endl;

		reconstruct(reconstructed,plan);
	}else{
		int sum = 0;
		for (int i = 0;i<signalLength;i++) sum += signal[i];
		gain = ((float) sum)/signalLength;
		if (gain < 0.5){ 
			memset(reconstructed,0,signalLength*sizeof(unsigned char)); 
		}else{
			memset(reconstructed,1,signalLength*sizeof(unsigned char));
		} 
	}
	/*calculation of the outlier set*/
	int j=0;
	unsigned char flip = 0;
	for (int i = 0;i<signalLength;i++)
	{
		if (signal[i] != reconstructed[i]^flip){
			 flip = 1-flip;
			 unsigned int* outlierSetTmp = (unsigned int*)realloc(outlierSet,(outliers+1)*(sizeof(int)));
			 if (outlierSetTmp == NULL) fprintf(stderr,"Failed to reallocate the outlier set!\n"); 
			 outlierSet = outlierSetTmp;
			 outlierSet[outliers++] = i;
		}
	}
	free(reconstructed);

	return;
}

void CFrelement::update(int modelOrder,CFFTPlan *plan)
{
	unsigned char *reconstructed = (unsigned char*)malloc(signalLength*sizeof(unsigned char));
	reconstruct(reconstructed,plan);
	free(frelements);
	//free(outlierSet);
	order = modelOrder;
	outliers = 0;
	free(frelements);
	frelements = (SFrelement*) malloc(order*sizeof(SFrelement));
	if (frelements == NULL) fprintf(stderr,"Failed to reallocate spectral components!\n");
	build(reconstructed,signalLength,plan);
	free(reconstructed);
}

void CFrelement::reconstruct(unsigned char* signal,CFFTPlan *plan)
{
	CTimer timer;
	timer.start();

	int fftLength = signalLength/2+1;

	fftw_complex *coeffs;
	double *probability;

	probability = plan->probability; 
	coeffs = plan->coeffs;
	
	/*reconstructing the frequency spectrum*/
	if (order > 0){	
		memset(coeffs,0,fftLength*sizeof(fftw_complex));
		coeffs[0][0] = gain;
		for (int i=0;i<order;i++){
			coeffs[frelements[i].frequency][0] = frelements[i].amplitude*cos(frelements[i].phase);
			coeffs[frelements[i].frequency][1] = frelements[i].amplitude*sin(frelements[i].phase);
		}
		//cout << "IFFT preparation " << timer.getTime() << endl;

		fftw_execute_dft_c2r(plan->inverse,coeffs,probability);
	}else{
		for (int i = 0;i<signalLength;i++) probability[i] = gain;
	}
	if (debug) cout << "IFFT calculation " << timer.getTime() << endl;

	/*application of the outlier set*/
	int j=0;
	unsigned char flip = 0;
	//for (int i = 0;i<signalLength;i++) cout << "Pro " << probability[i] << " " << estimate(i) << endl;
	timer.reset();
	if (outliers > 0){
		int flipPos = outlierSet[j];
		for (int i = 0;i<signalLength;i++)
		{
			if (flipPos == i){
				flip = 1-flip;
				j++;
				if (j >= outliers) j = outliers-1; 
				flipPos = outlierSet[j];
			}
			signal[i] = ((probability[i]>0.5)^flip);
		}
	}else{
		for (int i = 0;i<signalLength;i++) signal[i] = probability[i]>0.5;
	}
	if (debug) cout << "Signal reconstruction time " << timer.getTime() << endl;

	return;
}

/*fills with values*/
void CFrelement::fill(unsigned char values[],int number)
{
}

/*gets length in terms of values measured*/
int CFrelement::getLength()
{
	return signalLength;
}

void CFrelement::add(unsigned char value)
{
	if (((estimate(signalLength) > 0.5)^((outliers%2)==1))!=value)
	{
		unsigned int* outlierSetTmp = (unsigned int*)realloc(outlierSet,(outliers+1)*(sizeof(unsigned int)));
		if (outlierSetTmp==NULL) fprintf(stderr,"Failed to reallocate the outlier set!\n"); 
		outlierSet = outlierSetTmp;
		outlierSet[outliers++] = signalLength;
	}
	signalLength++;
	return; 
}

/*text representation of the FREMEN model*/
void CFrelement::print()
{
	int errs = 0;
	for (int i=0;i<outliers/2;i++) errs+=(outlierSet[2*i+1]-outlierSet[2*i]);
	if (outliers%2 == 1) errs+=signalLength-outlierSet[outliers-1];
	std::cout << "Model order " << order << " prior: " << gain << " error: " << ((float)errs/signalLength) << " size: " << sizeof(this)<< endl;
	for (int i = 0;i<order;i++){
		std::cout << "Frelement " << i << " " << frelements[i].amplitude << " " << frelements[i].phase << " " << frelements[i].frequency << " " << endl;
	}
	std::cout << "Outlier set size " << outliers << ":";
	for (int i = 0;i<outliers;i++) std::cout << " " << outlierSet[i];
	std::cout << endl; 
}

/*retrieves a boolean*/
unsigned char CFrelement::retrieve(int timeStamp)
{
	int i = 0;
	for (i= 0;i<outliers;i++){
		if (timeStamp < outlierSet[i]) break;
	}
	return (estimate(signalLength) > 0.5)^(i%2);
}
 
float CFrelement::estimate(int timeStamp)
{
	float time = (float)timeStamp/signalLength;
	float estimate = gain;
	for (int i = 0;i<order;i++){
		estimate+=2*frelements[i].amplitude*cos(frelements[i].phase+time*frelements[i].frequency*2*M_PI);
	}
	return estimate;
}