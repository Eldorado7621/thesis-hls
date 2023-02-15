#include <cmath>
#include <ap_axi_sdata.h>
#include <ap_int.h>
#include <hls_stream.h>
#include "RRT_Star.hpp"

using namespace std;

typedef ap_axis<128,2,5,6> axis_data;
typedef ap_axis<16,2,5,6> axiso_data;

typedef struct{
	float Position_x;
	float Position_y;
	int Location_List;

}Struct_RRT_Star;


typedef union{
	Struct_RRT_Star struct_RRT_Star; //pt struct alias
	__int128_t reg;   //integer alias
}Node_Pt;

//nt nearest(hls::stream<axis_data> &dataInStream,Node_Rand x_rand,int N);

//int nearest(hls::stream<axis_data> &dataInStream,float x_rand_inpx,float x_rand_inpy,int N);

int nearest(hls::stream<axis_data> &dataInStream,Node_Rand x_rand,int N,
		int choose,float oup[3],float Squared_r_n,hls::stream<axiso_data> &XNearStream);
