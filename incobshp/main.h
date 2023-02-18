#include <cmath>
#include <ap_axi_sdata.h>
#include <ap_int.h>
#include <hls_stream.h>
#include "RRT_Star.hpp"

using namespace std;

typedef short int int16;

typedef ap_axis<64,2,5,6> axis_data;
typedef ap_axis<32,2,5,6> axiso_data;

typedef struct{
	__fp16 Position_x;
	__fp16 Position_y;
	short int Location_List;

}Struct_RRT_Star;


typedef union{
	Struct_RRT_Star struct_RRT_Star; //pt struct alias
	long reg;   //integer alias
}Node_Pt;

//nt nearest(hls::stream<axis_data> &dataInStream,Node_Rand x_rand,int N);

//int nearest(hls::stream<axis_data> &dataInStream,float x_rand_inpx,float x_rand_inpy,int N);

bool nearest(hls::stream<axis_data> &dataInStream,Node_Rand x_rand,short int N,
		short int choose,__fp16 oup[3],__fp16 Squared_r_n,
		__fp16 Squared_R_Robot,
		hls::stream<axiso_data> &XNearStream
);
