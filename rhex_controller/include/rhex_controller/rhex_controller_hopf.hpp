#ifndef RHEX_CONTROLLER_RHEX_CONTROLLER_HOPF_HPP
#define RHEX_CONTROLLER_RHEX_CONTROLLER_HOPF_HPP

#define _USE_MATH_DEFINES
#include <array>
#include <cassert>
#include <cmath>
#include <vector>

#define PI 3.14159265
#define CTRL_SIZE 10
#define DOF 6
#define AMP 1
#define IGNORE 1000

#define F 3
#define K 40
#define SIGMA 3

namespace rhex_controller {

    class RhexControllerHopf {
    public:

        RhexControllerHopf() {}

        RhexControllerHopf(const std::vector<double>& ctrl)
        {
            set_parameters(ctrl);
        }

        void set_parameters(const std::vector<double>& ctrl)
        {
            assert(ctrl.size() == CTRL_SIZE);

            _A = AMP;
            _f = ctrl[0] * F;
            _k = ctrl[1] * K;
            _sigma = ctrl[2] * SIGMA;
            _stance_angle = ctrl[3];
            _stance_offset = ctrl[4];
            _last_time = 0;
            _dt = 0.0;

            _fully_connected = false;

            _swing.resize(DOF, false);

            // [[-1,-1],[-1,-1],[-1,-1],[-1,-1],[-1,-1],[-1,-1]] unstable initial state
            // [[1,-1],[1,-1],[1,-1],[-1,1],[-1,1],[-1,1]] more stable
            // _state.resize(DOF, std::vector<double>(2, -1));

            _state = std::vector<std::vector<double> > {{1,-1},{1,-1},{1,-1},{-1,1},{-1,1},{-1,1}};

            _k_weights = std::vector<double>(DOF, _k);

            // set up weights between each of the legs, 1 for each except it_
            // [[0,1,1,1,1,1],[1,0,1,1,1,1],[1,1,0,1,1,1],[1,1,1,0,1,1],[1,1,1,1,0,1],[1,1,1,1,1,0]]
            _sigma_weights.resize(DOF, std::vector<double>(DOF, 0));
            for(size_t i = 0; i < DOF; ++i)
            {
                for(size_t j = 0; j < DOF; ++j)
                {
                    if (i != j)
                    {
                        _sigma_weights[i][j] = _sigma;
                    }
                }
            }

            // set up phase bias.
            // [[0,PI,0,PI,0,PI],[-PI,0,-PI,0,-PI,0],[0,PI,0,PI,0,PI],[-PI,0,-PI,0,-PI,0],[0,PI,0,PI,0,PI],[-PI,0,-PI,0,-PI,0]]
            // [[*,PI,0,PI,*,*], [-PI,*,*,*,PI,*], [0,*,*,*,*,PI], [-PI,*,*,*,PI,0], [*,-PI,*,-PI,*,*], [*,*,-PI,0,*,*]]

            if (!_fully_connected){
//                _phase_bias = std::vector<std::vector<double> > {
//                    {IGNORE, PI, 0, PI, IGNORE, IGNORE},
//                    {-PI, IGNORE, IGNORE, IGNORE, PI, IGNORE},
//                    {0,IGNORE,IGNORE,IGNORE,IGNORE,PI},
//                    {-PI,IGNORE,IGNORE,IGNORE,PI,0},
//                    {IGNORE,-PI,IGNORE,-PI,IGNORE,IGNORE},
//                    {IGNORE,IGNORE,-PI,0,IGNORE,IGNORE}
//                };

                double phase14 = ctrl[5] * PI;
                double phase12 = ctrl[6] * PI;
                double phase13 = ctrl[7] * PI;
                double phase45 = ctrl[8] * PI;
                double phase46 = ctrl[9] * PI;

                double phase52 = phase45 - phase14 - phase12;
                double phase63 = phase46 - phase14 - phase13;

                _phase_bias = std::vector<std::vector<double> > {
                    {IGNORE, IGNORE, IGNORE, -phase14, IGNORE, IGNORE},
                    {-phase12, IGNORE, IGNORE, IGNORE, phase52, IGNORE},
                    {-phase13, IGNORE, IGNORE, IGNORE, IGNORE, phase63},
                    {phase14, IGNORE, IGNORE, IGNORE, IGNORE, IGNORE},
                    {IGNORE, -phase52, IGNORE, phase45, IGNORE, IGNORE},
                    {IGNORE, IGNORE, -phase63, phase46, IGNORE, IGNORE}
                };
            }

            else {
                _phase_bias.resize(DOF, std::vector<double>(DOF, 0));
                for(size_t i = 0; i < DOF; ++i)
                {
                    for(size_t j = 0; j < DOF; ++j)
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

            _counter.resize(DOF, 0);
            _motor_input.resize(DOF, 0);
            _last_motor_input.resize(DOF, 0);
        }

        std::vector<double> pos(double t)
        {

            _dt = t - _last_time;
            amp_couple_update(_dt);
            _last_time = t;

            // convert the signal into a monotonous motor input
            for (size_t i = 0; i < DOF; ++i)
            {
                if (_state[i][0] <= -1)
                    _counter[i] += 1;

                double x = get_land_couple()[i];
                if (x < _motor_input[i])
                    x = x + _counter[i] * 2 * PI;

                _last_motor_input[i] = _motor_input[i];
                _motor_input[i] = x;

                if(_motor_input[i] - _last_motor_input[i] > 6)
                {
                    while(_motor_input[i] - _last_motor_input[i] > 6){
                        _motor_input[i] = _motor_input[i] - 2 * PI;
                    }
                }
            }

            for (size_t i = 0; i < DOF; ++i)
                _motor_input[i] += _stance_offset * 2 * PI;

            return _motor_input;
        }

        void amp_couple_update(double dt)
        {
            for (size_t i = 0; i < DOF; ++i)
            {
                double du = _k_weights[i] * \
                (_A*_A - _state[i][0]*_state[i][0] - _state[i][1]*_state[i][1]) * \
                _state[i][0] - 2 * PI * _f * _state[i][1];

                double dv = _k_weights[i] * \
                (_A*_A - _state[i][0]*_state[i][0] - _state[i][1]*_state[i][1]) * \
                _state[i][1] + 2 * PI * _f *_state[i][0];

                _time += dt;

                for (size_t j = 0; j < DOF; ++j)
                {
                    if (i != j && _phase_bias[i][j] != IGNORE)
                        dv += _sigma * _sigma_weights[i][j] * \
                        (_state[j][0] * sin(_phase_bias[i][j]) + \
                         _state[j][1] * cos(_phase_bias[i][j]));
                }

                _state[i][0] += du * dt;
                _state[i][1] += dv * dt;

                if(_state[i][0] >= 1)
                    _swing[i] = true;
                if (_state[i][0] <= -1)
                    _swing[i] = false;
            }

        }

        std::vector<double> get_land_couple()
        {

            std::vector<double> output(DOF, 0);
            double x = 0;
            for (size_t i = 0; i < DOF; ++i)
            {
                if (_swing[i])
                    x = (2 * PI) - (1 - _stance_angle * PI) * \
                            (1 + _state[i][0] / _A);
                else
                    x = (_stance_angle * PI) * \
                            (1 + (_state[i][0] / _A));


                output[i] = x;
            }

            return output;
        }

        const std::vector<std::vector<double> >& parameters() const
        {
            return _state;
        }

    protected:
        double _A;
        double _f;
        double _time;
        double _dt;
        double _sigma;
        double _k;
        double _last_time;
        double _stance_offset;
        double _stance_angle;
        bool _fully_connected;

        std::vector<double> _k_weights;
        std::vector<bool> _swing;
        std::vector<std::vector<double> > _phase_bias;
        std::vector<std::vector<double> > _sigma_weights;
        std::vector<std::vector<double> > _weights;
        std::vector<std::vector<double> > _state;
        std::vector<int> _counter;
        std::vector<double> _last_motor_input;
        std::vector<double> _motor_input;
    };
}

#endif // RHEX_CONTROLLER_HOPF
