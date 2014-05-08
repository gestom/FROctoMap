#include "CFremenGrid.h"

using namespace std;

CFremenGrid::CFremenGrid(int size)
{
	numCells = size;
	order = 0;
	cellArray = (CFrelement**) malloc(numCells*sizeof(CFrelement*));
	for (int i=0;i<numCells;i++) cellArray[i] = new CFrelement();
	plan = new CFFTPlan();
}

CFremenGrid::~CFremenGrid()
{
	for (int i=0;i<numCells;i++) free(cellArray[i]);
	delete plan;
	free(cellArray);
}


void CFremenGrid::update(int order,int signalLengthi)
{
	signalLength = signalLengthi;
	plan->prepare(signalLength);
	cout << "Signal length " << signalLength << " of " << cellArray[0]->getLength() << endl;
	for (int i=0;i<numCells;i++){
		if (i%100==0)cout << "Updating cell " << i << " of " << numCells << endl;
		cellArray[i]->update(order,plan);
	}
}

void CFremenGrid::save(const char* name,bool lossy)
{
	FILE* f=fopen(name,"w");
	fwrite(&numCells,sizeof(int),1,f);
	fwrite(&signalLength,sizeof(int),1,f);
	for (int i=0;i<numCells;i++) cellArray[i]->save(f,lossy);
	fclose(f);
}

void CFremenGrid::load(const char* name)
{
	int ret = 0;
	signalLength = 0;
	FILE* f=fopen(name,"r");
	for (int i=0;i<numCells;i++){
		 free(cellArray[i]);
		 fprintf(stdout,"Cells %i %i\n",i,sizeof(CFrelement*),signalLength);
	}
	free(cellArray);
	ret = fread(&numCells,sizeof(unsigned int),1,f);
	ret = fread(&signalLength,sizeof(unsigned int),1,f);
	fprintf(stdout,"Cells %i, signal length %i\n",numCells,signalLength);
	cellArray = (CFrelement**) malloc(numCells*sizeof(CFrelement*));
	for (int i=0;i<numCells;i++) cellArray[i] = new CFrelement();
	for (int i=0;i<numCells;i++){
		cellArray[i]->load(f);
		cellArray[i]->signalLength = signalLength;
	}
	fclose(f);
}

void CFremenGrid::print(int number)
{
	if (cellArray[number]->order > 0 || cellArray[number]->outliers > 0){
		printf("Cell: %i ",number);
		cellArray[number]->print();
	}
}

void CFremenGrid::reconstruct(int number,unsigned char *reconstructed)
{
	cellArray[number]->reconstruct(reconstructed,plan);
}

void CFremenGrid::add(unsigned int index,unsigned char value)
{
	cellArray[index]->add(value);
}

unsigned char CFremenGrid::retrieve(unsigned int index,unsigned int timeStamp)
{
	return cellArray[index]->retrieve(timeStamp);
}

float CFremenGrid::estimate(unsigned int index,unsigned int timeStamp)
{
	return cellArray[index]->estimate(timeStamp);
}
