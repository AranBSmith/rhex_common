#ifndef RHEX_CONTROLLER_RHEX_CONTROLLER_CPG_HPP
#define RHEX_CONTROLLER_RHEX_CONTROLLER_CPG_HPP

#define _USE_MATH_DEFINES
#include <array>
#include <cassert>
#include <cmath>
#include <vector>
#define PI 3.14159265
#define CTRL_SIZE 6

namespace rhex_controller {

    class RhexControllerCPG {
    public:

        RhexControllerCPG() {}

        RhexControllerCPG(const std::vector<double>& ctrl)
        {
            set_parameters(ctrl);
        }

        void set_parameters(const std::vector<double>& ctrl)
        {
            assert(ctrl.size() == CTRL_SIZE);

            _freq = 0.2;
            _duty_ratio = 0.5;
            _thetlg = -PI / 3;
            _thettg = -2 * PI / 3;
            _amp = PI;
            _last_time = 0;
            _dt = 0.0;

            _phase = ctrl;

            // set up phase vector to start at correct position.
            // the start position is between the start & end of stance period
            _start_phase = _thettg - _thetlg;

            _phase = std::vector<double>(6,0); // reset in case of carry over.
//            for(int i = 0; i < CTRL_SIZE; ++i)
//                _phase[i] = _start_phase;

            // set up weights between each of the legs, 1 for each except itself.
            // [[0,1,1,1,1,1],[1,0,1,1,1,1],[1,1,0,1,1,1],[1,1,1,0,1,1],[1,1,1,1,0,1],[1,1,1,1,1,0]]
            _weights.resize(6, std::vector<double>(6, 0));
            for(size_t i = 0; i < CTRL_SIZE; ++i)
            {
                for(size_t j = 0; j < CTRL_SIZE; ++j)
                {
                    if (i != j)
                    {
                        _weights[i][j] = 10;
                    }
                }
            }

            // set up phase bias.
            // [[0,pi,0,pi,0,pi],[-pi,0,-pi,0,-pi,0],[0,pi,0,pi,0,pi],[-pi,0,-pi,0,-pi,0],[0,pi,0,pi,0,pi],[-pi,0,-pi,0,-pi,0]]
            _phase_bias.resize(6, std::vector<double>(6,0));
            for(size_t i = 0; i < CTRL_SIZE; ++i)
            {
                for(size_t j = 0; j < CTRL_SIZE; ++j)
                {
                    if (i == 0 || i % 2 == 0)
                    {
                        if (j == 0 || j % 2 == 0)
                            _phase_bias[i][j] = 0;
                        else
                            _phase_bias[i][j] = PI;
                    }
                    else
                    {
                        if (j == 0 || j % 2 == 0)
                            _phase_bias[i][j] = -PI;
                        else
                            _phase_bias[i][j] = 0;
                    }
                }
            }

            calc_vars();
        }

        // intermediate variables calculated based on start and end of stance phase
        void calc_vars()
        {
            _Sf = (2 * PI - (_thetlg - _thettg)) / ((1 - _duty_ratio) * 2 * PI);
            _Ss = transform((_thetlg - _thettg) / (_duty_ratio * 2 * PI));
            _thettg_in = transform(_thettg - _duty_ratio * 2 * PI) + 2 * PI; // this value needs to be in the interval [-pi, pi], thus add 2pi

        }

        // transforms values in the range -pi to pi.
        double transform(double x)
        {
//            if (x > PI)
//            {
//                double t = 0;

//                if (fmod(floor(x / PI), 2) == 1)
//                    t = fmod(x, (2 * PI)) - 2 * PI;
//                else
//                    t = fmod(x, (2 * PI));

//                if (std::isnan(t)) {
//                    std::cout << "ERROR: transform(double x): ";
//                    std::cout << " x: " << x;
//                    std::cout << " x/2PI : " << (x / (2*PI));
//                    std::cout << " fmod1: " << fmod(x , (2 * PI));
//                }

//                return t;
//            }

//            else if (x < -PI)
//            {

//            }
            double y = x;
            if ( x < -PI ){
                double z = floor((x+PI) / (2 * PI));
                y = x + (-z*2*PI);
            }

            else if (x > PI){
                double z = ceil((x-PI) / (2 * PI));
                y = x - (z*2*PI);
            }

            return y;
        }

        double dp(size_t idx){
            double phasediff = 2 * PI * _freq;

            for (size_t i = 0; i < _phase.size(); ++i)
            {
                if (i != idx)
                    phasediff += _amp * _weights[idx][i] * sin(_phase[i] - _phase[idx] - _phase_bias[idx][i]);
            }
            return phasediff;
        }

        void update_values()
        {
            for ( size_t i = 0; i < _phase.size(); ++i){
                //std::cout<<_phase[i]<<std::endl;
            }

            for (size_t i = 0; i < _phase.size(); ++i){
                //std::cout<<_phase.size()<<std::endl;
                _phase[i] = _phase[i] + dp(i) * _dt;
             }

        }

        // transform a value into its respective swing or stance value.
        double mono_transform(double x)
        {
            // std::cout<< "mono transform received: " << x << std::endl;
            double y = 0;
            if (transform(x) <= _thettg)
                y = _thettg + (transform(x) - _thettg) * _Sf;
            else if (transform(x) <= _thettg_in)
                y = _thettg + (transform(x) - _thettg) * _Ss;
            else
                y = _thetlg + (transform(x) - _thettg_in) * _Sf;

            if (std::isnan(y)){
                std::cout << "ERROR: produced nan value from mono_transform: ";
                std::cout << "Sf: " << _Sf << " ";
                std::cout << "Ss: " << _Ss << " ";
                std::cout << "Thettg: " << _thettg << " ";
                std::cout << "Thettlg:  " << _thetlg << " ";
                std::cout << "Thetg_in:  " << _thettg_in << " ";
                std::cout << "transform x:  " << transform(x) << " ";
                std::exit(0);
            }

            // std::cout << "Returning: " << transform(y) << std::endl;
            return transform(y); //+ (PI/2);
        }

        // make const?

        const std::vector<double>& get_phase() const
        {
            return _phase;
        }

        const std::vector<double>& parameters() const
        {
            return _phase;
        }

        // use cpg to return values
        std::vector<double> pos(double t)
        {
//            for ( size_t i = 0; i < _phase.size(); ++i){
//                std::cout<<_phase[i]<<std::endl;
//            }
            assert(_phase.size() == CTRL_SIZE);

            // t treated as current time.
            _dt = t - _last_time;
            //std::cout << "CPG delta time: " << _dt << std::endl;
            update_values();
            _last_time = t;
            return get_phase();
        }

    protected:
        double _thetlg;
        double _thettg;
        double _thettg_in;
        double _Sf;
        double _Ss;
        double _amp;
        double _freq;
        double _last_time;
        double _dt;
        double _duty_ratio;
        double _start_phase;
        std::vector<std::vector<double> > _phase_bias;
        std::vector<std::vector<double> > _weights;
        std::vector<double> _phase;
    };
} // namespace rhex_controller

#endif // RHEX_CONTROLLER_CPG

