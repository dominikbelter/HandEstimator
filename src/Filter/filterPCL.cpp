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
/*
void FilterPCL::FilterScene(Point3D::Cloud& input,Point3D::Cloud& output) const {

}*/

void FilterPCL::FilterScene(Point3D::Cloud& input, Point3D::Cloud& output) const {
float Y ;
    float Cb;
    float Cr;
    // filtr w przestrzeni kolorów YCbCr
    for(int i=0;i<input.size();i++){
        Y=0.299*(float)input[i].colour.r+0.587*(float)input[i].colour.g+0.114*(float)input[i].colour.b;
        Cb=128-0.168935*(float)input[i].colour.r-0.331665*(float)input[i].colour.g+0.50059*(float)input[i].colour.b;
        Cr=128+0.499813*(float)input[i].colour.r-0.418531*(float)input[i].colour.g-0.081282*(float)input[i].colour.b;
        if(Cb>=77 && Cb <=127 && Cr>=133 && Cr<=173){
            output[i].colour.r=255;
            output[i].colour.g=255;
            output[i].colour.b=255;
        }
    }

    // dzia³anie na pliku z dysku

    /*
    float Y ;
    float Cb;
    float Cr;
    unsigned char r;
    unsigned char g;
    unsigned char b;

    bitmap_image image("sciezka_pliku");
    const unsigned int height = image.height();
    const unsigned int width  = image.width();

    for (std::size_t y = 0; y < height; ++y)
       {
          for (std::size_t x = 0; x < width; ++x)
          {
             image.get_pixel(x,y,r,g,b);
             Y=0.299*r+0.587*g+0.114*b;
             Cb=128-0.168935*r-0.331665*g+0.50059*b;
             Cr=128+0.499813*r-0.418531*g-0.081282*b;
             // Y > 80, 85 < Cb < 135, and 135 < Cr < 180 // warianty przedzia³ów
             //  77 <= Cb <= 127, and 133 <= Cr <= 173
             if(Cb>=77 && Cb <=127 && Cr>=133 && Cr<=173)
                 image.set_pixel(x,y,255,255,255);
          }
       }

    image.save_image("sciezka_pliku_out");
    */

    // fragment testowy

    /*for(int i=0;i<input.size();i++){
        if(input[i].colour.r<150)
            output[i].colour.r=0;
    }*/

}

handest::Filter* handest::createFilterPCL(void) {
	filter.reset(new FilterPCL());
	return filter.get();
}

