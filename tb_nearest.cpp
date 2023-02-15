#include "main.h"

int main()
{
	axis_data tb_input_stream;
	hls::stream<axis_data>tb_dataInStream;
	Node_Pt node_pt;
	Node_RRT_Pos nrrt[1000];
	Node_Rand xrand;
	Node_Rand xrandn[1000];
	xrand.Position.x=11.2;
	xrand.Position.y=13.2;
	for(int i=0; i<100;i++)
			{
	xrandn[i].Position.x=1*(i);
		xrandn[i].Position.y=i*1.12;
			}
	int pos;
	float oup[3];
	int rtdnv[1];
	//pos=nearest(tb_dataInStream, xrand, 100);
	for (int j=0;j<100;j++)
	{
		for(int i=0; i<j;i++)
		{

			node_pt.struct_RRT_Star.Position_x=i;
			node_pt.struct_RRT_Star.Position_y=i*1.12;
			tb_input_stream.data=node_pt.reg;
			if(i==j-1)
				tb_input_stream.last=1;
			else
				tb_input_stream.last=0;
			tb_dataInStream.write(tb_input_stream);

		}


		int rtndval=nearest(tb_dataInStream,xrandn[j],j,2, oup,2.3);




		//nearest(tb_dataInStream, xrandn[i],10,pos);
		std::cout<<"position:"<<rtndval<<endl;
	}



}
