#include <cmath>
#define MAX_EDGES 4
#define VERTICES 4

class Point_2
    {
	private:
	    float _x;
	    float _y;
	public:
	    Point_2()
	    {
		_x = 0.f;
		_y = 0.f;
	    }
	    ~Point_2(){}
	    Point_2( const float _x, const float _y )
	    {
		this->_x = _x;
		this->_y = _y;
	    }
	    float x() const
	    {
		return _x;
	    }
	    float y() const
	    {
		return _y;
	    }
	    float squared_distance( Point_2 &p )
	    {
		return pow( _x - p.x(), 2 ) + pow( _y - p.y(), 2 );
	    }
    };
class Segment_3
{
	//Note Ax + By + C = 0
	//A = ( y_2 - y_1 )
	//B = -( x_2 - x_1 )
	//C = ( x_2 - x_1 )y_1 - ( y_2 - y_1 )x_1
public:
	Point_2 Source;
	Point_2 Target;
	float AABB;

public:

	~Segment_3(){}

	Point_2 source() const
	{
		return Source;
	}
	Point_2 target() const
	{
		return Target;
	}
	float squared_distance( Point_2 &p )
	{
		float U = ( ( p.x() - Source.x() ) * ( Target.x() - Source.x() ) +
				( p.y() - Source.y() ) * ( Target.y() - Source.y() ) ) / AABB;
		if( U >= 0 && U <= 1 )
			return pow( ( ( Target.x() - Source.x() ) * ( p.y() - Source.y() ) -
					( Target.y() -Source.y() ) * ( p.x() - Source.x() ) ), 2 )/ AABB;
		else
			if( U > 1 )
				return Target.squared_distance( p );
			else
				return Source.squared_distance( p );
	}
};
