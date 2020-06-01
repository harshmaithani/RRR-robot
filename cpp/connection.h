#ifndef CONNECTION_H
#define CONNECTION_H

#include <vector>
class Connection
{
    public:
        Connection(){}
        ~Connection(){}
	
        int open();
        int close();

        void send(std::vector<unsigned char>& , float &, float &, float &);
    	//send data to the robot. use explicit pointer conversion

        void receive(const std::vector<unsigned char>& , float &, float &, float &);
	    //receive state of the robot. record to data. use explicit pointer conversion
};

#endif
