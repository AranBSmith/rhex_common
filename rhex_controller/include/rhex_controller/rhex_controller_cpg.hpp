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

            _freq = 0.4;
            _duty_ratio = 0.5;
            _thetlg = -PI / 6;
            _thettg = -5 * PI / 6;
            _amp = PI;

            _controller = ctrl;

            // set up phase vector to start at correct position.
            // the start position is between the start & end of stance period
            _start_phase = _thettg - _thetlg;

            for(size_t i = 0; i < CTRL_SIZE; ++i)
                _phase.push_back(_start_phase);

            // set up weights between each of the legs, 1 for each except itself.
            // [[0,1,1,1,1,1],[1,0,1,1,1,1],[1,1,0,1,1,1],[1,1,1,0,1,1],[1,1,1,1,0,1],[1,1,1,1,1,0]]
            _weights.resize(6, std::vector<double>(6,0));
            for(size_t i = 0; i < CTRL_SIZE; ++i)
            {
                for(size_t j = 0; j < CTRL_SIZE; ++j)
                {
                    if (i != j)
                    {
                        _weights[i][j] = 1;
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
        }

        // intermediate variables calculated based on start and end of stance phase
        void calc_vars()
        {
            _Sf = (2 * PI - (_thetlg - _thettg)) / ((1 - _duty_ratio) * 2 * PI);
            _Ss = transform((_thetlg - _thettg) / (_duty_ratio * 2 * PI));
            _thettg_in = transform(_thettg - _dr * 2 * pi) + 2 * PI; // this value needs to be in the interval [-pi, pi], thus add 2pi

        }

        // transforms values in the range -pi to pi.
        double transform(double x)
        {
            if (x > PI)
            {
                double t = 0;

                if (floor(x / PI) % 2 == 1)
                    t = (x % (2 * PI)) - 2 * PI;
                else
                    t = (x % (2 * PI));

                return t;
            }

            return x;
        }

        double dp(idx){
            double phasediff = 2 * PI * _f;

            for (int i = 0; i < _phase.size; i++)
            {
                if (i != idx)
                    phasediff += _amp * _weights[idx][i] * sin(_phase[i] - _phase[idx] - _phase_bias[idx][i]);
            }
            return phasediff;
        }

        void update_values()
        {
            for (int i = 0; i < _phase.size; i++)
                _phase[i] = _phase[i] + dp(i) * _dt;
        }

        // transform a value into its respective swing or stance value.
        double mono_tran(x)
        {
            y = 0;
            if (transform(x) <= _thettg)
                y = _thettg + (translate(x) - _thettg) * _Sf;
            else if (translate(x) <= _thettg_in)
                y = _thettg + (translate(x) - _thettg) * _Ss;
            else
                y = _thetlg + (translate(x) - _thettg_in) * _Sf;

            return y;
        }

        std::vector<double>& output()
        {
            std::vector<double> act;
            for (int i = 0; i < _phase.size; i++)
                act.push_back(_mono_transform(_phase[i]));
            return act;
        }

        std::vector<double>& get_phase()
        {
            return _phase;
        }

        const std::vector<double>& parameters() const
        {
            return _controller;
        }

        // use cpg to return values
        // TODO
        std::vector<double>  pos(double t)
        {
            assert(_controller.size() == CTRL_SIZE);
            _dt = t;

            return tau;
        }

    protected:
        std::vector<double> _controller;

        double _thetlg;
        double _thettg;
        double _thettg_in;
        double _Sf;
        double _Ss;
        double _amp;
        double _freq;
        double _dt;
        double _duty_ratio;
        double _start_phase;
        std::vector<vector<double> > _phase_bias;
        std::vector<vector<double> > _weights;
        std::vector<vector<double> > _phase;
    };
} // namespace rhex_controller

#endif // RHEX_CONTROLLER_CPG

