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

bool Colition( Point_2 &A, __fp16 Squared_R_Robot, Segment_3 Edges[MAX_EDGES])
{
	bool Bool = false;
	for (int i=0;i<MAX_EDGES;i++)
	{
		if( Edges[i].squared_distance( A ) < Squared_R_Robot) return true;



	}
	return Bool;
}

bool Obstacle_Free( Position_Holonomic &A, Position_Holonomic &B, __fp16 Squared_R_Robot,
		Segment_3 Edges[MAX_EDGES] )
{
	bool Bool = true;
	int N = 5;
	Position_Holonomic Tem_Positon;
	float Delta_x = (A.x - B.x)/(N*1.0);
	float Delta_y = (A.y - B.y)/(N*1.0);

	for( int i = 0; i <= N; i++)
	{
		Tem_Positon.x = Delta_x*i + B.x;
		Tem_Positon.y = Delta_y*i + B.y;
		Point_2 t( Tem_Positon.x, Tem_Positon.y );
		if( Colition( t, Squared_R_Robot, Edges ) )
		{

			Bool = false;
			return Bool;
		}
	}
	return Bool;
}

static void Near(    int X_near[SIZE_NEAR], Node_RRT_Pos RRT_Star_Pos[Node], Node_RRT_Pos &x_new,
				__fp16 Squared_r_n, short int N , short int &X_near_count)
{
	X_near_count=0;
	for( int i=0; i<N; i++ )
	{
		if( Cal_Squared_Dist( x_new, RRT_Star_Pos[i] ) < Squared_r_n )
		{
			X_near[X_near_count]= RRT_Star_Pos[i ].Location_List;
			X_near_count++;
		}
	}
}


//int nearest(hls::stream<dataFmt> &dataInStream,float x_rand_inpx,float x_rand_inpy,int N)
//int nearest(hls::stream<axis_data> &dataInStream,Node_Rand x_rand,float x_new_posx,float x_new_posy,int N)
bool nearest(hls::stream<axis_data> &dataInStream,Node_Rand x_rand,short int N,
		short int choose,float oup[3],__fp16 Squared_r_n,
		__fp16 Squared_R_Robot,
		hls::stream<axiso_data> &XNearStream
)
{

#pragma HLS INTERFACE axis register both port=dataInStream
#pragma HLS INTERFACE s_axilite port=x_rand bundle=CTRL_BUS
#pragma HLS INTERFACE s_axilite port=N bundle=CTRL_BUS
#pragma HLS INTERFACE s_axilite port=choose bundle=CTRL_BUS
#pragma HLS INTERFACE s_axilite port=oup bundle=CTRL_BUS
#pragma HLS INTERFACE s_axilite port=Squared_r_n bundle=CTRL_BUS
#pragma HLS INTERFACE s_axilite port=Squared_R_Robot bundle=CTRL_BUS

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

	float A;
	float B;

	Segment_3 Edges[MAX_EDGES];
	Edges[0].Source=Point_2(-10,10);
	Edges[0].Target=Point_2(10,10);

	Edges[1].Source=Point_2(10,10);
	Edges[1].Target=Point_2(10,-10);

	Edges[2].Source=Point_2(10,-10);
	Edges[2].Target=Point_2(-10,-10);

	Edges[3].Source=Point_2(-10,-10);
	Edges[3].Target=Point_2(-10,10);
	for(int i=0;i<4;i++)
	{
		A = ( Edges[i].Target.y() - Edges[i].Source.y() );
		B = ( Edges[i].Target.x() - Edges[i].Source.x() );
		Edges[i].AABB = A*A + B*B;
	}

	int Pos_Nearest_Node=0;
	float Dist;
	Dist= 0;
	float Tem_Dist;
	Tem_Dist= 0;
	//int j=1;
	int w=0;

	short int XNearCnt=0;
	int rtndval=0;


	 int x_near[SIZE_NEAR];
	float calc_sqr_dst;

	//write the input to the RRT_Star
	//int i=0;



	for(int i=0;i<N;i++)
	{
#pragma HLS PIPELINE
		input_stream=dataInStream.read();
		node_pt.reg=input_stream.data;

		RRT_Star[i].Position.x=node_pt.struct_RRT_Star.Position_x;
		RRT_Star[i].Position.y=node_pt.struct_RRT_Star.Position_y;
		RRT_Star[i].Location_List=node_pt.struct_RRT_Star.Location_List;


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
	oup[2]=(float)RRT_Star[N-1].Location_List;
	rtndval=Pos_Nearest_Node;

	bool is_obstacle_free=Obstacle_Free( RRT_Star[Pos_Nearest_Node].Position, x_new_pos.Position,
			Squared_R_Robot, Edges );
	//oup[0]=(float)is_obstacle_free;


	if(is_obstacle_free)
	{

		Near( x_near, RRT_Star, x_new_pos, Squared_r_n, N,XNearCnt );

		for(int j=0;j<XNearCnt;j++)
		{
			output_stream.data=(int)RRT_Star[j].Position.x;

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
	else
	{
		for(int j=0;j<10;j++)
		{

			output_stream.data=0;

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

	return is_obstacle_free;





	/* if(choose==2)
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
	}*/

	//return rtndval;



}


