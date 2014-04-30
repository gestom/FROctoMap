#include "CFremenGrid.h"

using namespace std;

CFremenGrid::CFremenGrid(int size)
{
	numCells = size;
	order = 0;
	cellArray = (CFrelement**) malloc(numCells*sizeof(CFrelement));
	for (int i=0;i<numCells;i++) cellArray[i] = new CFrelement();
	plan = new CFFTPlan();
}

CFremenGrid::~CFremenGrid()
{
	for (int i=0;i<numCells;i++)  delete cellArray[i];
	delete plan;
	free(cellArray);
}


void CFremenGrid::update(int order,int signalLengthi)
{
	signalLength = signalLengthi;
	plan->prepare(signalLength);
	cout << "Signal length " << signalLength << " of " << cellArray[0]->getLength() << endl;
	for (int i=0;i<numCells;i++){
		if (i%100==0)cout << "Udating cell " << i << " of " << numCells << endl;
		cellArray[i]->update(order,plan);
	}
}

void CFremenGrid::save(const char* name,bool lossy)
{
	FILE* f=fopen(name,"w");
	for (int i=0;i<numCells;i++) cellArray[i]->save(f,lossy);
	fclose(f);
}

void CFremenGrid::load(const char* name)
{
	FILE* f=fopen(name,"r");
	for (int i=0;i<numCells;i++) cellArray[i]->load(f);
	fclose(f);
}

void CFremenGrid::print(int number)
{
	cellArray[number]->print();
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
