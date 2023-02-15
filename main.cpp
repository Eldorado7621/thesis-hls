#include "main.h"


#define Node 10000
#define SIZE_NEAR 1000

float Cal_Squared_Dist( Node_Rand &x_A, Node_RRT_Pos &x_B)
{
	float D = 0;
	Point_2 A( x_A.Position.x, x_A.Position.y );
	Point_2 B( x_B.Position.x, x_B.Position.y );
	//std::cout<<"kkk"<<x_B.Position.x<<"y:"<<x_B.Position.y<<endl;
	D = A.squared_distance( B );
	return D;
}
float Cal_Dist( Node_RRT_Pos &x_A, Node_Rand &x_B )
{
	float D = 0;
	Point_2 A( x_A.Position.x, x_A.Position.y );
	Point_2 B( x_B.Position.x, x_B.Position.y );
	D = sqrt( A.squared_distance( B ) );
	return D;
}
static float Cal_Squared_Dist( Node_RRT_Pos &_A, Node_RRT_Pos &_B )
{
	float D = 0;
	Point_2 A( _A.Position.x, _A.Position.y );
	Point_2 B( _B.Position.x, _B.Position.y );
	D = A.squared_distance( B );
	return D;
}
Node_RRT_Pos A_StreamReader( Node_RRT_Pos &x_nearest, Node_Rand &x_rand )
{
	Node_RRT_Pos tem;
	float aux = Cal_Dist( x_nearest, x_rand );
	float rad = 1;
	float t = rad/aux;

	if( aux <= rad ) tem.Position.Insert_Position( x_rand.Position.x, x_rand.Position.y );
	else
	{
		tem.Position.x = ( x_rand.Position.x - x_nearest.Position.x )*t +
				x_nearest.Position.x;
		tem.Position.y = ( x_rand.Position.y - x_nearest.Position.y )*t +
				x_nearest.Position.y;
	}

	//tem.Point = Point_2( tem.Position.x, tem.Position.y );
	return tem;
}


//int nearest(hls::stream<dataFmt> &dataInStream,float x_rand_inpx,float x_rand_inpy,int N)
//int nearest(hls::stream<axis_data> &dataInStream,Node_Rand x_rand,float x_new_posx,float x_new_posy,int N)
int nearest(hls::stream<axis_data> &dataInStream,Node_Rand x_rand,int N,
		int choose,float oup[3],float Squared_r_n,
		hls::stream<axiso_data> &XNearStream)
{

#pragma HLS INTERFACE axis register both port=dataInStream
#pragma HLS INTERFACE s_axilite port=x_rand bundle=CTRL_BUS
#pragma HLS INTERFACE s_axilite port=N bundle=CTRL_BUS
#pragma HLS INTERFACE s_axilite port=choose bundle=CTRL_BUS
#pragma HLS INTERFACE s_axilite port=oup bundle=CTRL_BUS
#pragma HLS INTERFACE s_axilite port=Squared_r_n bundle=CTRL_BUS
	//#pragma HLS INTERFACE axis register both port=dataOutStream
#pragma HLS INTERFACE axis register both port=XNearStream
#pragma HLS INTERFACE s_axilite port=return bundle=CTRL_BUS

	axis_data input_stream;
	axiso_data output_stream;
	/*float x[2];
x[0]=2.1;
x[1]=2.3;*/
	Node_Pt node_pt; //union of point struct and int64_t register
	Node_RRT_Pos RRT_Star[Node];
	Node_RRT_Pos x_new_pos;

	int Pos_Nearest_Node=0;
	float Dist;
	Dist= 0;
	float Tem_Dist;
	Tem_Dist= 0;
	//int j=1;
	int w=0;

	int XNearCnt=0;
	int rtndval=0;


	short int x_near[SIZE_NEAR];
	float calc_sqr_dst;

	//write the input to the RRT_Star
	//int i=0;


	if(choose==1)
	{
		for(int i=0;i<N;i++)
		{
#pragma HLS PIPELINE
			input_stream=dataInStream.read();
			node_pt.reg=input_stream.data;

			RRT_Star[i].Position.x=node_pt.struct_RRT_Star.Position_x;
			RRT_Star[i].Position.y=node_pt.struct_RRT_Star.Position_y;

			/*}
	for(int i=0;i<N;i++)
	{*/

			Tem_Dist = Cal_Squared_Dist( x_rand, RRT_Star[i]);
			if( w == 0)
			{
				Dist = Tem_Dist;
				Pos_Nearest_Node = w;
			}
			else{
				if( Tem_Dist < Dist )
				{
					Dist = Tem_Dist;
					Pos_Nearest_Node = w;
				}
			}
			w++;


		}

		x_new_pos = A_StreamReader( RRT_Star[Pos_Nearest_Node], x_rand);
		oup[0]=x_new_pos.Position.x;
		oup[1]=x_new_pos.Position.y;
		oup[2]=(float)Pos_Nearest_Node;
		rtndval=Pos_Nearest_Node;

		for(int j=0;j<10;j++)
		{
			x_near[j]=0;
			output_stream.data=x_near[j];

			output_stream.keep=input_stream.keep;

			output_stream.strb=input_stream.strb;

			output_stream.user=input_stream.user;

			output_stream.id=input_stream.id;

			output_stream.dest=input_stream.dest;

			if(j==10-1)
				output_stream.last=1;
			else
				output_stream.last=0;
			XNearStream.write(output_stream);


		}

	}



	else if(choose==2)
	{
		for(int i=0;i<N;i++)
		{
#pragma HLS PIPELINE

			input_stream=dataInStream.read();
			node_pt.reg=input_stream.data;

			RRT_Star[i].Position.x=node_pt.struct_RRT_Star.Position_x;
			RRT_Star[i].Position.y=node_pt.struct_RRT_Star.Position_y;
			RRT_Star[ i ].Location_List=node_pt.struct_RRT_Star.Location_List;

			calc_sqr_dst=Cal_Squared_Dist( x_rand, RRT_Star[i] );
			if(  calc_sqr_dst< Squared_r_n )
			{
				x_near[XNearCnt]= RRT_Star[i ].Location_List;
				XNearCnt++;
			}
		}
		rtndval=XNearCnt;
		for(int j=0;j<XNearCnt;j++)
		{
			output_stream.data=x_near[j];

			output_stream.keep=input_stream.keep;

			output_stream.strb=input_stream.strb;

			output_stream.user=input_stream.user;

			output_stream.id=input_stream.id;

			output_stream.dest=input_stream.dest;

			if(j==XNearCnt-1)
				output_stream.last=1;
			else
				output_stream.last=0;
			XNearStream.write(output_stream);


		}
	}

	return rtndval;



}
