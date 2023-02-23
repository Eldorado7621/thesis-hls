 #include "RRT_Star.hpp"

#include "xil_printf.h"


using namespace std;
class Auxiliary
{
    public:
		inline static char inputData[96024];
		inline static char buff[256];
		inline static  FIL* fptr;

		static int  Read_xrand(Node_RRT x_rand[Node]){
			int Status;
			xil_printf("SD card init  \n");

			Status = SD_Init();

			if(Status != XST_SUCCESS)
				xil_printf("SD card init failed");
			//xrand_x
			memset(inputData,0,sizeof(inputData));
			memset(buff,0,sizeof(buff));
			xil_printf("SD card1  \n");
			fptr = openFile("XRANDX.txt",'r');
			if(fptr == 0)
				printf("Opening information xrand_x failed\n\r");
			Status = nReadFile("XRANDX.txt",(u32)inputData);

			if (Status != XST_SUCCESS) {
				print("read information xrand_x failed\n\r");
				return XST_FAILURE;
			}

			int count=0;
			int node_index=0;
			for(int i=0; inputData[i] != '\0'; i++)
			{
				buff[count]=inputData[i];
				count++;
				if(inputData[i]=='\n')
				{
					buff[count-1]='\0';
					x_rand[node_index].Position.x= stof( buff );
					node_index++;
					//x.push_back( 5 );
					count=0;

				}
			}
			//close the file
			closeFile(fptr);

			//X_RAND_Y
			memset(inputData,0,sizeof(inputData));
			memset(buff,0,sizeof(buff));

			fptr = openFile("XRANDY.txt",'r');
			if(fptr == 0)
				printf("Opening information xrand_x failed\n\r");
			Status = nReadFile("XRANDY.txt",(u32)inputData);

			if (Status != XST_SUCCESS) {
				print("read information xrand_x failed\n\r");
				return XST_FAILURE;
			}

			count=0;
			node_index=0;
			for(int i=0; inputData[i] != '\0'; i++)
			{


				buff[count]=inputData[i];
				count++;
				if(inputData[i]=='\n')
				{
					buff[count-1]='\0';
					x_rand[node_index].Position.y= stof( buff );
					node_index++;
					//x.push_back( 5 );
					count=0;

				}


			}
			//close the file
			closeFile(fptr);

			return 0;

		}

    	static int  Read_data( Position_Holonomic &Pos_Init, Position_Holonomic &Pos_Goal,
    			int &Total_Num_Node, float &Radio_Robot, float &Radio_Goal,
				float &Gamma_RRT, float &Mu, Polygon_2 &Geo, string Name)
    	{
    		list<int>  mylistt = {6, 1, 2, 3, 4};




    		int Status;
    		xil_printf("SD card init  \n");

    		Status = SD_Init();

    		if(Status != XST_SUCCESS)
    			xil_printf("SD card init failed");

    		memset(inputData,0,sizeof(inputData));
    		memset(buff,0,sizeof(buff));

    		fptr = openFile("info.txt",'r');

    		Status = nReadFile("info.txt",(u32)inputData);
    		if (Status != XST_SUCCESS) {
    	 	 print("file read failed\n\r");
    		    	 return XST_FAILURE;
    		}
    		int count=0;
    		int line_count=0;
    		int i = 1;
    		int Num_H = 0;
    		list<int>x;
    		list<int>y;
    		for(int i=0; inputData[i] != '\0'; i++)
    		{
    			buff[count]=inputData[i];

    			if(inputData[i]=='\n')
    			{
    				buff[count]='\0';
    				switch(line_count) {
    				case 0:
    					Pos_Init.x=stof( buff );
    					break;
    				case 1:
    					Pos_Init.y =stof( buff );
    					break;
    				case 2:
    					Pos_Goal.x =stof( buff );
    					break;
    				case 3:
    					Pos_Goal.y =stof( buff );
    					break;
    				case 4:
    					Total_Num_Node =stof( buff );
    					break;
    				case 5:
    					Radio_Robot=stof( buff );
    					break;
    				case 6:
    					Radio_Goal =stof( buff );
    					break;
    				case 7:
    					Gamma_RRT =stof( buff );
    					break;
    				case 8:
    					Mu =stof( buff );
    					break;
    				case 9:
    					Num_H =stof( buff );
    					break;

    				default:
    					break;
    					// code block
    				}
    				line_count++;
    				count=0;
    			}
    			else
    				count++;
    		}
    		closeFile(fptr);


    		//Information pole x
    		memset(inputData,0,sizeof(inputData));
    		fptr = openFile("infopx.txt",'r');
    		if(fptr == 0)
    			printf("Opening information pole x failed\n\r");
    		Status = nReadFile("infopx.txt",(u32)inputData);
    		if (Status != XST_SUCCESS) {
    			print("read nformation pole x failed\n\r");
    			return XST_FAILURE;
    		}

    		count=0;
    		for(int i=0; inputData[i] != '\0'; i++)
    		{
    			buff[count]=inputData[i];
    			count++;
    			if(inputData[i]=='\n')
    			{
    				buff[count-1]='\0';
    				x.push_back( stoi( buff ) );
    				//x.push_back( 5 );
    				count=0;

    			}

    		}
    		//close the file
    		closeFile(fptr);
    		/*for (auto g : x)
    		   xil_printf("xx:%d",g);*/

    		//information pole y
    		memset(inputData,0,sizeof(inputData));
    		fptr = openFile("infopy.txt",'r');
    		if(fptr == 0)
    			printf("Opening information pole y failed\n\r");
    		Status = nReadFile("infopy.txt",(u32)inputData);
    		if (Status != XST_SUCCESS) {
    			print("read nformation pole y failed\n\r");
    			return XST_FAILURE;
    		}

    		count=0;
    		for(int i=0; inputData[i] != '\0'; i++)
    		{
    			buff[count]=inputData[i];
    			count++;
    			if(inputData[i]=='\n')
    			{
    				buff[count-1]='\0';
    				y.push_back( stof( buff ) );
    				count=0;

    			}

    		}
    		//close the file
    		closeFile(fptr);
/*    		for (auto g : y)
    		    xil_printf("yy:%d",g);
    		//x.pop_back();
    		//y.pop_back();
    		for (auto g : y)
    		    xil_printf("ypy:%d",g);*/
    		Polygon_2 polygon;
    		//cout<<"xsize"<<x.size()<<endl;
            if( x.size() == y.size() )
            {
              list<int>::iterator i_x = x.begin();
              list<int>::iterator i_y = y.begin();
              while( i_x != x.end() )
              {
                  polygon.push_back( Point_2( *i_x, *i_y ) );
                  i_x++;
                  i_y++;
              }
              //if( polygon.is_clockwise_oriented() ) polygon.reverse_orientation();
            }
            else
     	     cout<<"Error 1"<<endl;

  	      Geo = polygon;


    		return 0;


    	}
    	static void Save_Trajectory( vector<Node_RRT> &G, int i, string Name )
    	{
    					FIL* fptrx;
    		    		FIL* fptry;
    		    		xil_printf("saving trajectory to SD card...\n\r");

    		    		char xdataBuffer[32]={0};
    		    		char ydataBuffer[32]={0};

    		    		fptry = openFile("trajvy.txt",'w');
    		    		fptrx = openFile("trajvx.txt",'w');

    		    		if(fptrx == 0)
    		    			printf("trajectoryx File opening failed\n\r");
    		    		if(fptry == 0)
    		    			printf("trajectoryy File opening failed\n\r");

    		    		 while( i != 0 )
    		    		{
    		    			sprintf(xdataBuffer,"%0.4f\n",G.at( i ).Position.x);
    		    			//sprintf(ydataBuffer,"%0.4f\n",G.at( i ).Position.y);
    		    			i = G[i].Location_Parent;

    		    			writeFile(fptrx, strlen(xdataBuffer),(u32) xdataBuffer);
    		    			//writeFile(fptry, strlen(ydataBuffer),(u32) ydataBuffer);

    		    		}
    		    		 sprintf(xdataBuffer,"%0.4f\n",G.at( i ).Position.x);
    		    		 //sprintf(ydataBuffer,"%0.4f\n",G.at( i ).Position.y);
    		    		 i = G[i].Location_Parent;

    		    		 writeFile(fptrx, strlen(xdataBuffer),(u32) xdataBuffer);
    		    		 //writeFile(fptry, strlen(ydataBuffer),(u32) ydataBuffer);

    		    		closeFile(fptrx);
    		    		closeFile(fptry);
    }


};

