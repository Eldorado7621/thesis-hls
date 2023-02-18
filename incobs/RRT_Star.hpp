#include <cmath>
#include "Aux_Lib.h"
#include <hls_half.h>


class Position_Holonomic
    {
        public:
            float x;
            float y;
            Position_Holonomic()//builder
            {
                x = 0;
                y = 0;
            }
            Position_Holonomic(float _x, float _y)//builder
            {
                x = _x;
                y = _y;
            }
            ~Position_Holonomic()//destroyer
            {
                //cout<< "Destructor Position_Holonomic"<<endl;
            }
            void Insert_Position(float _x, float _y)
            {
                x = _x;
                y = _y;
            }
            void Copy(Position_Holonomic &A)
            {
                x = A.x;
                y = A.y;
            }
    };

class Node_Rand
{
public:
	Position_Holonomic Position;

	Node_Rand()
	{

	}
	~Node_Rand()//destroyer
	{
		//cout<<"Destructor Node_RRT"<<endl;
	}

};

class Node_RRT_Pos
    {
        public:
	Position_Holonomic Position;//Position of the Node

           int Location_List;//Location of the Node in the Graph

           // std::list<int> List_Children;//Node's children list
          // int waste;

            Node_RRT_Pos()//builder
            {

                Location_List = 0;

            }
            ~Node_RRT_Pos()//destroyer
            {
                //cout<<"Destructor Node_RRT"<<endl;
            }

    };
