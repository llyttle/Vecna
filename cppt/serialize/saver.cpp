#include<iostream>
#include<sstream>
#include<string>
#include<boost/archive/binary_iarchive.hpp>
#include<boost/archive/binary_oarchive.hpp>

using namespace std;

class Piano {
    private:
        friend class boost::serialization::access;
        string brand;
        int price;
        struct
}