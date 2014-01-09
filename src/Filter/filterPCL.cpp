#include "../include/Filter/filterPCL.h"
#include <memory>
#include <stdexcept>

using namespace handest;

FilterPCL::Ptr filter;

FilterPCL::FilterPCL(void) : name("Filter") {

}

const std::string& FilterPCL::getName() const {
	return name;
}


void FilterPCL::FilterScene(Point3D::Cloud& input, Point3D::Cloud& output) const {

    float Y ;
    float Cb;
    float Cr;
    float i_out=0;
    Point3D::Cloud temp;
    float av=0;

    // filtr w przestrzeni kolorow YCbCr
    for(int i=0;i<input.size();i++){

		//konwersja przestrzeni RGB na YCbCr
        Y=0.299*(float)input[i].colour.r+0.587*(float)input[i].colour.g+0.114*(float)input[i].colour.b;
        Cb=128-0.168935*(float)input[i].colour.r-0.331665*(float)input[i].colour.g+0.50059*(float)input[i].colour.b;
        Cr=128+0.499813*(float)input[i].colour.r-0.418531*(float)input[i].colour.g-0.081282*(float)input[i].colour.b;
        if(Cb>=77 && Cb <=127 && Cr>=133 && Cr<=173){ //przedzial kolorow dla skory w przestrzeni YCbCr
            temp[i_out].colour.r=input[i].colour.r;
            temp[i_out].colour.g=input[i].colour.g;
            temp[i_out].colour.b=input[i].colour.b;
            temp[i_out].position.x=input[i].position.x;
            temp[i_out].position.y=input[i].position.y;
            temp[i_out].position.z=input[i].position.z;
            av+=temp[i_out].position.z; //srednia wartosc glebii
            i_out++; //inkrementacja licznika elementów chmury wyjsciowej
        }
	}
    av=av/i_out; //srednia wartosc glebii
	i_out=0;

    //filtracja wg glebii
	for(int i=0;i<temp.size();i++){
		if (abs(temp[i].position.z)-av<300){ // w milimetrach
			output[i_out].colour.r=temp[i].colour.r;
            output[i_out].colour.g=temp[i].colour.g;
            output[i_out].colour.b=temp[i].colour.b;
			output[i_out].position.x=temp[i].position.x;
			output[i_out].position.y=temp[i].position.y;
			output[i_out].position.z=temp[i].position.z;
			i_out++;
		}
    }

}

handest::Filter* handest::createFilterPCL(void) {
	filter.reset(new FilterPCL());
	return filter.get();
}

