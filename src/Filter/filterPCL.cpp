#include "../include/Filter/filterPCL.h"
#include <memory>
#include <stdexcept>

using namespace handest;

FilterPCL::Ptr filter;

FilterPCL::FilterPCL(void) : name("Filter") {

}

///DB komentarz
const std::string& FilterPCL::getName() const {
	return name;
}

///DB komentarz
void FilterPCL::FilterScene(Point3D::Cloud& input, Point3D::Cloud& output_cloud) const {
    std::cout<<"Rozmiar filtrowanej chmury: "<<input.size()<<"\n";///DB remove
    float Y ;
    float Cb;
    float Cr;

    int i_out=0;
    Point3D temp;
    Point3D output;
    Point3D::Cloud temp_cloud;
    int av=0;

    // filtr w przestrzeni kolorow YCbCr
    for(int i=0;i<input.size();i++){///DB 'size_t' zamiast 'int'
        //konwersja przestrzeni RGB na YCbCr
        Y=0.299*(float)input[i].colour.r+0.587*(float)input[i].colour.g+0.114*(float)input[i].colour.b;
        Cb=128-0.168935*(float)input[i].colour.r+0.331665*(float)input[i].colour.g+0.50059*(float)input[i].colour.b;
        Cr=128+0.499813*(float)input[i].colour.r-0.418531*(float)input[i].colour.g-0.081282*(float)input[i].colour.b;
        if(Cb>=80 && Cb <=135 && Cr>=131 && Cr<=185){ //przedzial kolorow dla skory w przestrzeni YCbCr
         // if(Cb>=77 && Cb <=127 && Cr>=133 && Cr<=173){ //przedzial kolorow dla skory w przestrzeni YCbCr
            temp.colour.r=input[i].colour.r;
            temp.colour.g=input[i].colour.g;
            temp.colour.b=input[i].colour.b;
            temp.position.x=input[i].position.x;
            temp.position.y=input[i].position.y;
            temp.position.z=input[i].position.z;
            av+=int(temp.position.z*1000); //srednia wartosc glebii
            temp_cloud.push_back(temp);
            i_out++; //inkrementacja licznika elementów chmury wyjsciowej
        }
    }
    std::cout<<"Rozmiar chmury po pierwszej czesci filtracji: "<<temp_cloud.size()<<"\n";///DB remove

    av=(float)av/(float)i_out; //srednia wartosc glebii
	i_out=0;

    //filtracja wg glebii
    for(int i=0;i<temp_cloud.size();i++){
        if (abs(1000*temp_cloud[i].position.z-av)<300){ // w milimetrach
            output.colour.r=temp_cloud[i].colour.r;
            output.colour.g=temp_cloud[i].colour.g;
            output.colour.b=temp_cloud[i].colour.b;
            output.position.x=temp_cloud[i].position.x;
            output.position.y=temp_cloud[i].position.y;
            output.position.z=temp_cloud[i].position.z;
            output_cloud.push_back(output);
		}
    }

      std::cout<<"Rozmiar chmury po kompletnej filtracji: "<<output_cloud.size()<<"\n";///DB remove

}

handest::Filter* handest::createFilterPCL(void) {
	filter.reset(new FilterPCL());
	return filter.get();
}

