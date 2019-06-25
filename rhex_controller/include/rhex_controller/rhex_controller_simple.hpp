#ifndef RHEX_CONTROLLER_RHEX_CONTROLLER_SIMPLE_HPP
#define RHEX_CONTROLLER_RHEX_CONTROLLER_SIMPLE_HPP

// For M_PI constant
#define _USE_MATH_DEFINES
#include <array>
#include <cassert>
#include <cmath>
#include <vector>
#define PI 3.14159265
#define ARRAY_DIM 100

namespace rhex_controller {

    class RhexControllerSimple {
    public:
        typedef std::array<double, ARRAY_DIM> array_t;

        RhexControllerSimple() {}

        RhexControllerSimple(const std::vector<double>& ctrl, std::vector<int> broken_legs)
            : _broken_legs(broken_legs)
        {
            set_parameters(ctrl);

            if (_Kp.size() == 0)
                set_pd(5., 0.1);
        }

        void set_parameters(const std::vector<double>& ctrl)
        {
//            std::cout << "ctrl size: ";
//            std::cout << ctrl.size() << std::endl;

            assert(ctrl.size() == 48);
            _controller = ctrl;
        }

        void set_pd(double Kp, double Kd)
        {	
        	_Kp.clear();
        	_Kd.clear();
			// TODO 54, dof, could vary with leg removal
            for(int i=0; i < 54 ;i++){
            	_Kp.push_back(Kp);
            	_Kd.push_back(Kd);
            }
        }

        const std::vector<double>& parameters() const
        {
            return _controller;
        }

        void set_broken(const std::vector<int> broken_legs)
        {
            _broken_legs = broken_legs;
        }

        const std::vector<int>& broken_legs() const
        {
            return _broken_legs;
        }

        std::vector<double>  pos(double t) 
        {
        	// TODO
            assert(_controller.size() == 48);
            // a bit messy but creates 2 numbers ratio and other which are between 0 and 1 all the parameters about offset phase and other information is controlled by the control signal
            double ratio = 0;
            double help = 0;
            double temp = 0;
            double other = 0;

            help = remainder(double(t), double(0.75)) / (0.75);
            help = help + 0.5;
            
            ratio = (help < _controller[0]) ? help * _controller[1] * 2 : _controller[1] + (help - _controller[0]) * (1 - _controller[1]) * 2;
            ratio = ratio + ((1 - _controller[1]) / 2);

            if(ratio > 1){
                ratio = ratio-1;
            }

            temp = ((help + _controller[4]) > 1) ? help - _controller[4] : help + _controller[4];
            other = (temp < _controller[2]) ? temp * _controller[3] * 2 : _controller[3] + (temp - _controller[2]) * (1 - _controller[3]) * 2;
            other += ((1 - _controller[3]) / 2);

            if (other > 1){
                other = other-1;
            }

            std::vector<double> tau(48,0);
            
            // tau is the single target position vector and is updated here
            for(int i = 0; i < 6; i++){
                if((i % 2) == 0){
                    tau[i]= ratio * 2 * PI;
                } else {
                    tau[i]= other * 2 * PI;
                }
            }
            
            // the proportional controller and integral controller are updated here
            set_pd(_controller[5], _controller[6]);

            // they are then changed during their slow part or fast part of rotation
            if (help > _controller[0]){
                _Kp[0+6] = _controller[7];
                _Kp[2+6] = _controller[7];
                _Kp[4+6] = _controller[7];
            }

            if (temp > _controller[2]){
                _Kp[1+6] = _controller[7];
                _Kp[3+6] = _controller[7];
                _Kp[5+6] = _controller[7];
            }

            return tau;
        }

        std::vector<double> get_Kp(void){
        	return _Kp;
        }

        std::vector<double> get_Kd(void){
        	return _Kd;
        }

    protected:
        std::vector<double> _controller;
        std::vector<int> _broken_legs;
        std::vector<double> _Kp;
        std::vector<double> _Kd;
    };
} // namespace rhex_controller

#endif
