#include "include/gazebo_utils.hpp"

namespace steelhead_gazebo
{
    
    Eigen::Vector6d GetSdfVector(bool* status, sdf::ElementPtr _sdf, std::string param, Eigen::Vector6d def)
    {
        Eigen::Vector6d _vector = def;
        int idx = 0;
        double val;

        std::string sdf_data = GetSdfElement<std::string>(status, _sdf, param, "");

        if (*status == false)
        {
            gzdbg << param << ": \n" << _vector << std::endl;
            return _vector; 
        }

        std::istringstream iss(sdf_data);

        while (iss >> val && idx < MAX_DIMENSION)
        {
            _vector(idx++) = val;
        }
        if (idx == 0)
        {
            gzerr << "Vector parameter '" << param << "' contained no values. Using default values.\n";
        }
        else if (idx < MAX_DIMENSION)
        {
            gzdbg << "Vector parameter '" << param << "' contained " << idx << " values; using defaults for remaining " << (MAX_DIMENSION - idx) << ".\n";
        }
        *status = true;
        gzdbg << param << ": \n" << _vector << std::endl;

        return _vector;
    }


    Eigen::Matrix6d GetSdfMatrix(bool* status, sdf::ElementPtr _sdf, std::string param, Eigen::Matrix6d def)
    {
        Eigen::Matrix6d _matrix = def;
        int r_idx = 0, c_idx = 0;
        double val;

        std::string sdf_data = GetSdfElement<std::string>(status, _sdf, param, "");

        if (*status == false)
        {
            gzdbg << param << ": \n" << _matrix << std::endl;
            return _matrix;
        }

        std::istringstream iss(sdf_data);

        while (iss >> val)
        {
            _matrix(r_idx, c_idx++) = val;
            if (c_idx == MAX_DIMENSION)
            {
                c_idx = 0;
                r_idx++;
            }
        }
        if (r_idx != MAX_DIMENSION) 
        {
            gzerr << "Matrix did not fully populate, continuing...\n";
        }
        *status = true;
        gzdbg << param << ": \n" << _matrix << std::endl;

        return _matrix;
    }

}