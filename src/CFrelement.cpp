#include "CFrelement.h"

using namespace std;

bool fremenSort(SFrelement i,SFrelement j) 
{ 
return (i.amplitude>j.amplitude); 
}

CFrelement::CFrelement(int orderi)
{
	order = orderi;
	frelements = (SFrelement*) malloc(order*sizeof(SFrelement));	
	outlierSet=NULL;
	gain = 0;
	signalLength++;
}

CFrelement::~CFrelement()
{
}

void CFrelement::build(unsigned char* signal,int signalLengthi,int modelOrder,SPlan *plan)
{
	signalLength = signalLengthi;
	order = modelOrder;
	int fftLength = signalLength/2+1;
	fftw_complex *coeffs;
	double *probability,*fftSignal;
	CTimer timer;
	timer.start();

	probability = plan->probability;
	fftSignal = plan->signal;
	coeffs = plan->coeffs;

	for (int i = 0;i<signalLength;i++) fftSignal[i] = signal[i];
	timer.reset();
	cout << "FFT preparation time " << timer.getTime() << endl;

	/*calculation of the spectral model*/
	fftw_execute_dft_r2c(plan->direct,fftSignal,coeffs);
	cout << "FFT calculation time " << timer.getTime() << endl;

	SFrelement *tmpFrelements = (SFrelement *)malloc(fftLength*sizeof(SFrelement));
	for(int i=0;i<fftLength;i++)
	{
		complex<double> x (coeffs[i][0],coeffs[i][1]);
		tmpFrelements[i].amplitude = abs(x);
		tmpFrelements[i].frequency = i;
	}
	sort(tmpFrelements,tmpFrelements+(fftLength-1),fremenSort);

	gain = tmpFrelements[0].amplitude/signalLength;
	for(int i=1;i<order+1;i++){
		frelements[i-1].amplitude = tmpFrelements[i].amplitude/signalLength;
		frelements[i-1].phase = atan2(coeffs[tmpFrelements[i].frequency][1],coeffs[tmpFrelements[i].frequency][0]);
		frelements[i-1].frequency = tmpFrelements[i].frequency;
	}
	free(tmpFrelements);

	cout << "Spectrum recovery time " << timer.getTime() << endl;

	/*signal reconstruction*/
	unsigned char *reconstructed = (unsigned char*)malloc(signalLength*sizeof(unsigned char));
	reconstruct(reconstructed,plan);

	/*calculation of the outlier set*/
	int j=0;
	unsigned char flip = 0;
	for (int i = 0;i<signalLength;i++)
	{
		if (signal[i] != reconstructed[i]^flip){
			 flip = 1-flip;
			 unsigned int* outlierSetTmp = (unsigned int*)realloc(outlierSet,(outliers+1)*(sizeof(int)));
			 if (outlierSetTmp==NULL) fprintf(stderr,"Failed to reallocate the outlier set!\n"); 
			 outlierSet = outlierSetTmp;
			 outlierSet[outliers++] = i;
		}
	}
	free(reconstructed);
	//fftw_free(coeffs);
	//free(signal); 
	return;
}

void CFrelement::update(int modelOrder,SPlan *plan)
{
	unsigned char *reconstructed = (unsigned char*)fftw_malloc(signalLength);
	reconstruct(reconstructed,plan);
	build(reconstructed,signalLength,modelOrder,plan);
	free(reconstructed);
}

void CFrelement::reconstruct(unsigned char* signal,SPlan *plan)
{
	CTimer timer;
	timer.start();

	int fftLength = signalLength/2+1;

	fftw_complex *coeffs;
	double *probability;

	probability = plan->probability; 
	coeffs = plan->coeffs;
	//*inverseFFTPlan = fftw_plan_dft_c2r_1d(signalLength,coeffs,probability,FFTW_ESTIMATE);

	/*reconstructing the frequency spectrum*/
	memset(coeffs,1,fftLength*sizeof(fftw_complex));
	coeffs[0][0] = gain;
	for (int i=0;i<order;i++){
		coeffs[frelements[i].frequency][0] = frelements[i].amplitude*cos(frelements[i].phase);
		coeffs[frelements[i].frequency][1] = frelements[i].amplitude*sin(frelements[i].phase);
	}
	cout << "IFFT preparation " << timer.getTime() << endl;

	fftw_execute_dft_c2r(plan->inverse,coeffs,probability);
	cout << "IFFT calculation " << timer.getTime() << endl;

	/*application of the outlier set*/
	int j=0;
	unsigned char flip = 0;
	//for (int i = 0;i<signalLength;i++) cout << probability[i] << " " << estimate(i) << endl;
	if (outliers > 0){
		for (int i = 0;i<signalLength;i++)
		{
			if (outlierSet[j] == i){
				flip = 1-flip;
				j++;
			}
			signal[i] = ((probability[i]>0.5)^flip);
		}
	}else{
		for (int i = 0;i<signalLength;i++) signal[i] = probability[i]>0.5;
	}
	cout << "Signal reconstruction time " << timer.getTime() << endl;

	fftw_free(coeffs);
	free(probability); 
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

  /*gets length in terms of values measured*/
void CFrelement::add(unsigned char value)
{
	if (((estimate(signalLength) > 0.5)^((outliers%2)==1))!=value)
	{

		unsigned int* outlierSetTmp = (unsigned int*)realloc(outlierSet,(outliers+1)*(sizeof(int)));
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

/*clears all*/
void CFrelement::clear()
{
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
