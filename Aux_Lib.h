#include <cmath>

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
