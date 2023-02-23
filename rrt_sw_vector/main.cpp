#include "RRT_Star_Dist.cpp"
#include "Auxiliary.cpp"
#include "xil_printf.h"

#include <xtime_l.h>
#include <time.h>

FIL* fptr;

int main()
{

    //Version that uses a distance cost funtion
    //Version that uses Kd-tree
    cout << "Main" <<endl;
	XTime sw_processor_start, sw_processor_end;
    srand ( 0 );
    unsigned t0, t1;
    int Total_Num_Node;//Number of nodes in the RRT*
    float R_Robot;//Robot's radio
    float Radio_Goal;//Radio goal
    float Gamma_rrt;//Constant Gamma
    float Dim = 2;//Dimension of space
    float Mu;//Constant Mu
    Position_Holonomic Pos_Init; //Initial position of robot
    Position_Holonomic Pos_Goal; //Position of Goal
    Polygon_2 Environment; //Environment


    string Name = "1"; //Select Environment
    cout<<"Environment: "<<Name<<endl;
    bool Verbose = true;



    Auxiliary::Read_data( Pos_Init, Pos_Goal, Total_Num_Node, R_Robot, Radio_Goal,
	    Gamma_rrt, Mu, Environment, Name );

    Node_RRT x_rand[Node];
    Auxiliary::Read_xrand(x_rand);

    vector<Node_RRT> RRT_Star( Total_Num_Node ); //RRT*
    Region RRT_Star_Region( Environment, R_Robot );

    XTime_GetTime(&sw_processor_start);
    RRT_Star_Dist::Build_RRT_Star( RRT_Star, Pos_Init, Total_Num_Node, R_Robot, Gamma_rrt,
		    RRT_Star_Region, Dim, Mu, Environment,x_rand, Verbose);//Build RRT*

    int Best_Node = RRT_Star_Dist::Best_Trajectory(RRT_Star, Pos_Goal, Radio_Goal);

    XTime_GetTime(&sw_processor_end);

      float sw_processing_time=1000000.0*(sw_processor_end-sw_processor_start)/(COUNTS_PER_SECOND);
      cout << "Execution Time: " << sw_processing_time << endl;

    /*if( Best_Node != -1 ) Auxiliary:: Save_Trajectory( RRT_Star, Best_Node, Name );
    RRT_Star_Dist::Save_Graph( RRT_Star, Name );//Save RRT
    float sw_processing_time=1000000.0*(sw_processor_end-sw_processor_start)/(COUNTS_PER_SECOND);
//    //double time = ( double( t1 - t0 ) / CLOCKS_PER_SEC );
    cout << "Execution Time: " << sw_processing_time << endl;
    cout<<"done: "<<Name<<endl;*/
    //cout << "Execution Time: " << time << endl;

    /*for(int i=9000;i<10000;i++)
    	cout<<RRT_Star[i].Position.x<<endl;
    float sw_processing_time=1000000.0*(sw_processor_end-sw_processor_start)/(COUNTS_PER_SECOND);
    cout << "Execution Time: " << sw_processing_time << endl;

*/
    return 0;
}
